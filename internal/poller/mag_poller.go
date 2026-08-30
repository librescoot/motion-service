package poller

import (
	"context"
	"log/slog"
	"math"
	"sync/atomic"
	"time"

	"github.com/librescoot/motion-service/internal/bmx"
	"github.com/librescoot/motion-service/internal/calibration"
	"github.com/librescoot/motion-service/internal/redis"
)

const (
	tiltCompMaxExcessG = 0.05
	// Planar calibration cannot correct vehicle-frame Z hard iron reliably.
	maxPlanarHeadingTiltDeg = 10.0

	headingEMAAlphaFast = 0.50
	headingEMAAlphaMed  = 0.15
	headingEMAAlphaSlow = 0.05

	// The 2.5° data-sheet figure assumes a calibrated system.
	baseAccuracyDeg = 15.0
)

type MagPoller struct {
	mag       *bmx.Magnetometer
	accel     *bmx.Accelerometer
	gyro      *bmx.Gyroscope
	publisher *redis.Publisher
	cache     *bmx.SensorCache
	collector *calibration.Collector
	log       *slog.Logger

	rateHz     atomic.Int32
	rateChange chan struct{}

	emaFastSin, emaFastCos float64
	emaMedSin, emaMedCos   float64
	emaSlowSin, emaSlowCos float64
	emaInit                bool
	quality                headingQuality
	calibrationRevision    uint64

	pollCount int
}

func NewMagPoller(
	mag *bmx.Magnetometer,
	accel *bmx.Accelerometer,
	gyro *bmx.Gyroscope,
	publisher *redis.Publisher,
	cache *bmx.SensorCache,
	collector *calibration.Collector,
	rateHz int,
	log *slog.Logger,
) *MagPoller {
	p := &MagPoller{
		mag:        mag,
		accel:      accel,
		gyro:       gyro,
		publisher:  publisher,
		cache:      cache,
		collector:  collector,
		log:        log,
		rateChange: make(chan struct{}, 1),
	}
	p.rateHz.Store(int32(rateHz))
	return p
}

func (p *MagPoller) SetRate(rateHz int) {
	if int(p.rateHz.Load()) == rateHz {
		return
	}
	p.rateHz.Store(int32(rateHz))
	select {
	case p.rateChange <- struct{}{}:
	default:
	}
	p.log.Info("mag polling rate set", "rate_hz", rateHz)
}

func (p *MagPoller) Run(ctx context.Context) {
	for {
		rate := int(p.rateHz.Load())
		if rate <= 0 {
			p.log.Info("magnetometer poller suspended (rate=0)")
			select {
			case <-ctx.Done():
				p.log.Info("magnetometer poller stopped")
				return
			case <-p.rateChange:
				continue
			}
		}

		p.log.Info("magnetometer poller running", "rate_hz", rate)
		ticker := time.NewTicker(time.Second / time.Duration(rate))

	ticking:
		for {
			select {
			case <-ctx.Done():
				ticker.Stop()
				p.log.Info("magnetometer poller stopped")
				return
			case <-p.rateChange:
				ticker.Stop()
				break ticking
			case <-ticker.C:
				if err := p.poll(ctx); err != nil {
					p.log.Error("failed to poll magnetometer", "error", err)
				}
			}
		}
	}
}

func (p *MagPoller) poll(ctx context.Context) error {
	if p.mag == nil {
		return nil
	}

	rawX, rawY, rawZ, magX, magY, magZ, magMag, dataReady, err := p.mag.ReadAll()
	if err != nil {
		return err
	}

	if p.cache != nil {
		p.cache.StoreMag(bmx.MagSnapshot{
			Timestamp: time.Now(),
			CompX:     rawX, CompY: rawY, CompZ: rawZ,
			X: magX, Y: magY, Z: magZ, Magnitude: magMag,
		})
	}

	orientation := p.mag.Orientation()
	var (
		ax, ay, az, aMag float64
		yawRateDPS       float64
		hasAccel         bool
	)
	var (
		snap   bmx.IMUSnapshot
		cached bool
	)
	if p.cache != nil {
		snap, cached = p.cache.LoadIMU(250 * time.Millisecond)
	}
	if cached {
		ax, ay, az, aMag = snap.AccelX, snap.AccelY, snap.AccelZ, snap.AccelMag
		yawRateDPS = snap.GyroMag
		hasAccel = true
	} else {
		if p.accel != nil {
			var aErr error
			ax, ay, az, aMag, aErr = p.accel.ReadDataInGVehicleFrame(orientation)
			if aErr != nil {
				p.log.Warn("accel read failed, skipping tilt comp", "error", aErr)
			} else {
				hasAccel = true
			}
		}
		if p.gyro != nil {
			_, _, _, gMag, gerr := p.gyro.ReadDataInDPSVehicleFrame(orientation)
			if gerr == nil {

				yawRateDPS = gMag
			}
		}
	}

	rollRad := math.NaN()
	pitchRad := math.NaN()
	tiltDeg := 0.0
	tiltCompensated := false
	excessG := 0.0
	if hasAccel {
		excessG = math.Abs(aMag - 1.0)
		if excessG <= tiltCompMaxExcessG {
			rollRad, pitchRad = tiltFromSpecificForce(ax, ay, az)
			tiltCompensated = true
		}

		horiz := math.Sqrt(ax*ax + ay*ay)
		tiltDeg = math.Atan2(horiz, -az) * 180.0 / math.Pi
	}

	horizontalX, horizontalY := p.mag.LeveledHorizontal(
		magX, magY, magZ, rollRad, pitchRad)
	horizontalField := math.Hypot(horizontalX, horizontalY)
	rawHeading := p.mag.HeadingFromVector(magX, magY, magZ, rollRad, pitchRad)
	if revision := p.mag.CalibrationRevision(); revision != p.calibrationRevision {
		p.calibrationRevision = revision
		p.quality = headingQuality{}
		p.emaInit = false
	}
	headingFast, headingMed, headingSlow := p.smoothHeadings(rawHeading)

	if p.collector != nil {
		p.collector.Add(calibration.Sample{
			Timestamp: time.Now().UnixMilli(),
			CompX:     float64(rawX), CompY: float64(rawY), CompZ: float64(rawZ),
			AccelX: ax, AccelY: ay, AccelZ: az,
			FieldUT: magMag, ExcessG: excessG, TiltDeg: tiltDeg,
		})
	}

	quality := p.quality.evaluate(rawHeading, magMag, horizontalField,
		excessG, yawRateDPS, tiltCompensated)
	calibrationState := p.mag.Calibration().State
	quality = applyCalibrationPolicy(quality, calibrationState, tiltDeg)
	accuracy := estimateAccuracyDeg(tiltDeg, excessG, yawRateDPS, tiltCompensated)
	if quality.Valid {
		accuracy += quality.FieldResidual*30.0 + quality.DispersionDeg*0.25
	} else {
		// Existing consumers only understand accuracy. Keep them from trusting
		// a heading rejected by the richer validity policy.
		accuracy = 180.0
	}

	p.pollCount++
	if p.pollCount >= 25 {
		p.log.Info("mag heading",
			"raw_x", rawX, "raw_y", rawY, "raw_z", rawZ,
			"uT_x", magX, "uT_y", magY, "uT_z", magZ, "uT_mag", magMag,
			"heading_med", headingMed,
			"tilt_deg", tiltDeg,
			"tilt_comp", tiltCompensated,
			"excess_g", excessG,
			"yaw_rate_dps", yawRateDPS,
			"accuracy_deg", accuracy,
			"heading_valid", quality.Valid,
			"invalid_reason", quality.Reason,
			"field_residual", quality.FieldResidual,
			"horizontal_field_ut", horizontalField)
		p.pollCount = 0
	}

	reading := &redis.HeadingReading{
		Timestamp:         time.Now().UnixMilli(),
		HeadingDeg:        headingMed,
		HeadingRawDeg:     rawHeading,
		HeadingFastDeg:    headingFast,
		HeadingSlowDeg:    headingSlow,
		AccuracyDeg:       accuracy,
		HeadingValid:      quality.Valid,
		InvalidReason:     quality.Reason,
		CalibrationState:  calibrationState,
		TiltCompensated:   tiltCompensated,
		TiltDeg:           tiltDeg,
		MagStrengthUT:     magMag,
		HorizontalFieldUT: horizontalField,
		FieldResidual:     quality.FieldResidual,
		HeadingDispersion: quality.DispersionDeg,
		ExcessG:           excessG,
		YawRateDPS:        yawRateDPS,
		DataReady:         dataReady,
	}
	return p.publisher.PublishHeading(ctx, reading)
}

// Acceleration is specific force: level in vehicle NED is (0, 0, -1g).
func tiltFromSpecificForce(ax, ay, az float64) (rollRad, pitchRad float64) {
	return math.Atan2(-ay, -az),
		math.Atan2(ax, math.Sqrt(ay*ay+az*az))
}

func applyCalibrationPolicy(quality headingQualityResult, state string, tiltDeg float64) headingQualityResult {
	if quality.Valid && tiltDeg > maxPlanarHeadingTiltDeg {
		quality.Valid = false
		quality.Reason = "excessive_tilt"
	}
	if state != "calibrated" {
		quality.Valid = false
		quality.Reason = "uncalibrated"
	}
	return quality
}

func estimateAccuracyDeg(tiltDeg, excessG, yawRateDPS float64, tiltCompensated bool) float64 {
	a := baseAccuracyDeg

	if !tiltCompensated {

		a += math.Min(tiltDeg*0.5, 45.0)
	} else {

		a += tiltDeg * 0.05
	}

	a += excessG * 50.0

	a += yawRateDPS * 0.1

	if a < baseAccuracyDeg {
		a = baseAccuracyDeg
	}
	return a
}

func (p *MagPoller) smoothHeadings(newHeading float64) (fast, med, slow float64) {
	rad := newHeading * math.Pi / 180.0
	s := math.Sin(rad)
	c := math.Cos(rad)

	if !p.emaInit {
		p.emaFastSin, p.emaFastCos = s, c
		p.emaMedSin, p.emaMedCos = s, c
		p.emaSlowSin, p.emaSlowCos = s, c
		p.emaInit = true
	} else {
		p.emaFastSin = headingEMAAlphaFast*s + (1-headingEMAAlphaFast)*p.emaFastSin
		p.emaFastCos = headingEMAAlphaFast*c + (1-headingEMAAlphaFast)*p.emaFastCos
		p.emaMedSin = headingEMAAlphaMed*s + (1-headingEMAAlphaMed)*p.emaMedSin
		p.emaMedCos = headingEMAAlphaMed*c + (1-headingEMAAlphaMed)*p.emaMedCos
		p.emaSlowSin = headingEMAAlphaSlow*s + (1-headingEMAAlphaSlow)*p.emaSlowSin
		p.emaSlowCos = headingEMAAlphaSlow*c + (1-headingEMAAlphaSlow)*p.emaSlowCos
	}

	toDeg := func(sn, cs float64) float64 {
		if sn == 0 && cs == 0 {
			return newHeading
		}
		return math.Mod(math.Atan2(sn, cs)*180.0/math.Pi+360.0, 360.0)
	}
	return toDeg(p.emaFastSin, p.emaFastCos),
		toDeg(p.emaMedSin, p.emaMedCos),
		toDeg(p.emaSlowSin, p.emaSlowCos)
}
