package poller

import (
	"context"
	"log/slog"
	"math"
	"time"

	"bmx-service/internal/bmx"
	"bmx-service/internal/redis"
)

const (
	magPollingRateHz        = 5
	headingSmoothingSamples = 3

	// Tilt-comp inputs are unreliable when the body frame is undergoing
	// significant non-gravity acceleration; ay²+az²+ax² ≠ 1g means the accel
	// vector isn't pure gravity. Above this excess we report tilt_compensated
	// = false and fall back to the X/Y-only heading.
	tiltCompMaxExcessG = 0.20

	// Datasheet ±2.5° heading accuracy at the Regular preset; we treat that
	// as the floor and add penalties for tilt, dynamic accel, and yaw rate.
	baseAccuracyDeg = 2.5
)

// MagPoller continuously polls the magnetometer (and accel/gyro for context)
// and publishes a tilt-compensated heading at magPollingRateHz.
type MagPoller struct {
	mag       *bmx.Magnetometer
	accel     *bmx.Accelerometer
	gyro      *bmx.Gyroscope
	publisher *redis.Publisher
	log       *slog.Logger

	headingHistory []float64
	historyIndex   int
	initialized    int
	pollCount      int
}

// NewMagPoller creates a MagPoller. accel and gyro may be nil; without an
// accelerometer the poller falls back to the non-tilt-compensated heading.
func NewMagPoller(
	mag *bmx.Magnetometer,
	accel *bmx.Accelerometer,
	gyro *bmx.Gyroscope,
	publisher *redis.Publisher,
	log *slog.Logger,
) *MagPoller {
	return &MagPoller{
		mag:            mag,
		accel:          accel,
		gyro:           gyro,
		publisher:      publisher,
		log:            log,
		headingHistory: make([]float64, headingSmoothingSamples),
	}
}

// Run starts the magnetometer polling loop at magPollingRateHz.
func (p *MagPoller) Run(ctx context.Context) {
	p.log.Info("starting magnetometer poller", "rate_hz", magPollingRateHz)

	ticker := time.NewTicker(time.Second / magPollingRateHz)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			p.log.Info("magnetometer poller stopped")
			return

		case <-ticker.C:
			if err := p.poll(ctx); err != nil {
				p.log.Error("failed to poll magnetometer", "error", err)
			}
		}
	}
}

func (p *MagPoller) poll(ctx context.Context) error {
	if p.mag == nil {
		return nil
	}

	rawX, rawY, rawZ, err := p.mag.ReadData()
	if err != nil {
		return err
	}

	magX, magY, magZ, magMag, err := p.mag.ReadDataInMicroTesla()
	if err != nil {
		return err
	}

	// Pull accel + gyro for tilt comp and quality estimate. Either failing
	// just means we publish without that input — better than dropping the
	// heading entirely.
	var (
		ax, ay, az, aMag float64
		hasAccel         bool
	)
	if p.accel != nil {
		ax, ay, az, aMag, err = p.accel.ReadDataInG()
		if err != nil {
			p.log.Warn("accel read failed, skipping tilt comp", "error", err)
		} else {
			hasAccel = true
		}
	}

	var yawRateDPS float64
	if p.gyro != nil {
		gx, gy, gz, _, gerr := p.gyro.ReadDataInDPS()
		if gerr == nil {
			// Total angular speed; the gyro Z component alone misses cases
			// where the scooter is rolling/pitching. For "is the heading
			// changing fast right now" the magnitude is the better signal.
			yawRateDPS = math.Sqrt(gx*gx + gy*gy + gz*gz)
		}
	}

	// Compute heading. Tilt-compensate when accel data is plausibly gravity.
	rollRad := math.NaN()
	pitchRad := math.NaN()
	tiltDeg := 0.0
	tiltCompensated := false
	excessG := 0.0
	if hasAccel {
		excessG = math.Abs(aMag - 1.0)
		if excessG <= tiltCompMaxExcessG {
			rollRad = math.Atan2(ay, az)
			pitchRad = math.Atan2(-ax, math.Sqrt(ay*ay+az*az))
			tiltCompensated = true
		}
		// Report tilt regardless of whether comp was applied — the consumer
		// can use it to gate the heading on its own terms.
		r := math.Atan2(ay, az)
		pi := math.Atan2(-ax, math.Sqrt(ay*ay+az*az))
		tiltDeg = math.Max(math.Abs(r), math.Abs(pi)) * 180.0 / math.Pi
	}

	rawHeading := p.mag.HeadingFromVector(magX, magY, magZ, rollRad, pitchRad)
	smoothedHeading := p.smoothHeading(rawHeading)

	accuracy := estimateAccuracyDeg(tiltDeg, excessG, yawRateDPS, tiltCompensated)

	p.pollCount++
	if p.pollCount >= 25 {
		p.log.Info("mag heading",
			"raw_x", rawX, "raw_y", rawY, "raw_z", rawZ,
			"uT_x", magX, "uT_y", magY, "uT_z", magZ, "uT_mag", magMag,
			"heading", smoothedHeading,
			"tilt_deg", tiltDeg,
			"tilt_comp", tiltCompensated,
			"excess_g", excessG,
			"yaw_rate_dps", yawRateDPS,
			"accuracy_deg", accuracy)
		p.pollCount = 0
	}

	// Magnetometer values are already in the bmx:sensors payload at 10 Hz —
	// we don't need a separate bmx:magnetometer channel just for the same
	// data at half the rate. Suppresses ~750 B/s of redundant pub/sub
	// traffic over the USB-ethernet link to the DBC.

	reading := &redis.HeadingReading{
		Timestamp:       time.Now().UnixMilli(),
		HeadingDeg:      smoothedHeading,
		HeadingRawDeg:   rawHeading,
		AccuracyDeg:     accuracy,
		TiltCompensated: tiltCompensated,
		TiltDeg:         tiltDeg,
		MagStrengthUT:   magMag,
		ExcessG:         excessG,
		YawRateDPS:      yawRateDPS,
	}
	return p.publisher.PublishHeading(ctx, reading)
}

// estimateAccuracyDeg returns a 1-σ-ish heading accuracy estimate in degrees.
// Heuristic — not statistically rigorous; consumers should use it as
// "trust this heading more or less" rather than a hard error bar.
func estimateAccuracyDeg(tiltDeg, excessG, yawRateDPS float64, tiltCompensated bool) float64 {
	a := baseAccuracyDeg

	if !tiltCompensated {
		// X/Y-only heading: error grows roughly linearly with tilt.
		// 30° tilt → ~10° heading error; cap so the number stays usable.
		a += math.Min(tiltDeg*0.5, 45.0)
	} else {
		// Tilt comp leaves residual error from accel-derived roll/pitch
		// uncertainty. Smaller penalty.
		a += tiltDeg * 0.05
	}

	// Dynamic acceleration corrupts accel-derived tilt: 0.1 g excess →
	// roughly +5° (gravity vector deflected by atan(0.1)).
	a += excessG * 50.0

	// Heading is changing fast; smoothed value is stale. 50°/s → +5°.
	a += yawRateDPS * 0.1

	// Magnetic noise floor — even with reps, expect at least the datasheet
	// value of 2.5° for a single sample.
	if a < baseAccuracyDeg {
		a = baseAccuracyDeg
	}
	return a
}

// smoothHeading applies a circular-mean filter to heading values.
// A linear mean wraps incorrectly across 0°/360°: averaging 358° and 2°
// gives 180° instead of 0°. Average the unit vectors and atan2 back out.
func (p *MagPoller) smoothHeading(newHeading float64) float64 {
	p.headingHistory[p.historyIndex] = newHeading
	p.historyIndex = (p.historyIndex + 1) % headingSmoothingSamples

	if p.initialized < headingSmoothingSamples {
		p.initialized++
	}

	var sumSin, sumCos float64
	for i := 0; i < p.initialized; i++ {
		rad := p.headingHistory[i] * math.Pi / 180.0
		sumSin += math.Sin(rad)
		sumCos += math.Cos(rad)
	}

	if sumSin == 0 && sumCos == 0 {
		return newHeading
	}

	avgDeg := math.Atan2(sumSin, sumCos) * 180.0 / math.Pi
	avgDeg = math.Mod(avgDeg+360.0, 360.0)
	return avgDeg
}
