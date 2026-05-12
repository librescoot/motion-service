package poller

import (
	"context"
	"log/slog"
	"math"
	"time"

	"github.com/librescoot/motion-service/internal/bmx"
	"github.com/librescoot/motion-service/internal/redis"
)

const (
	magPollingRateHz = 5

	// Tilt-comp inputs are unreliable when the body frame is undergoing
	// significant non-gravity acceleration; ay²+az²+ax² ≠ 1g means the accel
	// vector isn't pure gravity. Above this excess we report tilt_compensated
	// = false and fall back to the X/Y-only heading. Tightened from 0.20 to
	// 0.05 g after observing heading deflection during normal scooter
	// throttle bursts (typical forward acceleration on a 2 kW scooter is
	// 0.1–0.3 g; the looser threshold let those leak into the tilt-comp).
	tiltCompMaxExcessG = 0.05

	// Three EMA smoothing levels for the heading, all on the (sin, cos)
	// unit-vector representation so the 0°/360° wrap is handled
	// naturally. Time constant τ = -dt / ln(1-α). At 5 Hz:
	//   fast α=0.50 → τ ≈ 0.3 s   (responsive, removes single-sample noise)
	//   med  α=0.15 → τ ≈ 1.2 s   (good balance — published as heading_deg)
	//   slow α=0.05 → τ ≈ 3.9 s   (very stable, lags through hard turns)
	headingEMAAlphaFast = 0.50
	headingEMAAlphaMed  = 0.15
	headingEMAAlphaSlow = 0.05

	// Datasheet ±2.5° heading accuracy at the Regular preset; we treat that
	// as the floor and add penalties for tilt, dynamic accel, and yaw rate.
	baseAccuracyDeg = 2.5
)

// MagPoller continuously polls the magnetometer (and accel/gyro for context)
// and publishes a tilt-compensated heading at magPollingRateHz.
type MagPoller struct {
	mag       *bmx.Magnetometer
	accel     *bmx.Accelerometer
	gyro     *bmx.Gyroscope
	publisher *redis.Publisher
	cache     *bmx.SensorCache
	log       *slog.Logger

	// Three EMA states on (sin, cos) unit vectors. Wrap-safe.
	emaFastSin, emaFastCos float64
	emaMedSin, emaMedCos   float64
	emaSlowSin, emaSlowCos float64
	emaInit                bool

	pollCount int
}

// NewMagPoller creates a MagPoller. accel and gyro may be nil; without an
// accelerometer the poller falls back to the non-tilt-compensated heading.
// If `cache` has a recent snapshot from sensor_poller, mag_poller consumes
// it for tilt-comp / quality inputs instead of re-issuing the same I2C
// reads. The direct accel/gyro fallback keeps the poller working when
// sensor_poller is disabled or the cache is stale.
func NewMagPoller(
	mag *bmx.Magnetometer,
	accel *bmx.Accelerometer,
	gyro *bmx.Gyroscope,
	publisher *redis.Publisher,
	cache *bmx.SensorCache,
	log *slog.Logger,
) *MagPoller {
	return &MagPoller{
		mag:       mag,
		accel:     accel,
		gyro:      gyro,
		publisher: publisher,
		cache:     cache,
		log:       log,
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

	// Both representations come from one I2C read — the compensated
	// sensor-frame int16 (for the raw_* log fields) and the calibrated
	// vehicle-frame µT triple (for heading + publish).
	rawX, rawY, rawZ, magX, magY, magZ, magMag, _, err := p.mag.ReadAll()
	if err != nil {
		return err
	}

	// Pull accel + gyro for tilt comp and quality estimate, in vehicle
	// frame. Prefer the shared cache (sensor_poller refreshes it at 10 Hz
	// in the same vehicle frame); only fall back to direct I2C if the
	// cache is empty or stale, so we still work when sensor_poller is
	// disabled. 250 ms maxAge comfortably covers the 5 Hz mag tick + half
	// a sensor-poller tick of slop.
	orientation := p.mag.Orientation()
	var (
		ax, ay, az, aMag float64
		yawRateDPS       float64
		hasAccel         bool
	)
	var (
		snap   bmx.SensorSnapshot
		cached bool
	)
	if p.cache != nil {
		snap, cached = p.cache.Load(250 * time.Millisecond)
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
				// Total angular speed; the gyro Z component alone misses cases
				// where the scooter is rolling/pitching. For "is the heading
				// changing fast right now" the magnitude is the better signal.
				yawRateDPS = gMag
			}
		}
	}

	// Compute heading. Tilt-compensate when accel is plausibly gravity.
	// rollRad/pitchRad below are NED-standard Tait-Bryan from accel — at
	// rest in NED, accel = (0, 0, -g) so atan2(ay, az) = π. The tilt-comp
	// math in HeadingFromVector handles that 180° offset correctly via
	// sin/cos, but it's misleading as a *displayed* tilt magnitude. So
	// we compute tiltDeg separately from the level-aware formula.
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
		// Combined tilt from level: at rest level NED (0,0,-1g), the
		// horizontal magnitude is 0 and -az = +1, so atan2 = 0. As the
		// vehicle tilts away from level the horizontal component grows.
		horiz := math.Sqrt(ax*ax + ay*ay)
		tiltDeg = math.Atan2(horiz, -az) * 180.0 / math.Pi
	}

	rawHeading := p.mag.HeadingFromVector(magX, magY, magZ, rollRad, pitchRad)
	headingFast, headingMed, headingSlow := p.smoothHeadings(rawHeading)

	accuracy := estimateAccuracyDeg(tiltDeg, excessG, yawRateDPS, tiltCompensated)

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
			"accuracy_deg", accuracy)
		p.pollCount = 0
	}

	// Magnetometer values are already in the motion:sensors payload at 10 Hz —
	// we don't need a separate motion:magnetometer channel just for the same
	// data at half the rate. Suppresses ~750 B/s of redundant pub/sub
	// traffic over the USB-ethernet link to the DBC.

	reading := &redis.HeadingReading{
		Timestamp:       time.Now().UnixMilli(),
		HeadingDeg:      headingMed,
		HeadingRawDeg:   rawHeading,
		HeadingFastDeg:  headingFast,
		HeadingSlowDeg:  headingSlow,
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

// smoothHeadings updates all three EMA filters on the (sin, cos) unit
// vectors of the new heading and returns each as a degrees-in-[0,360)
// triple. EMA on the unit vectors handles 0°/360° wrap naturally — a
// linear EMA on degrees would lurch 360° at the wraparound and pollute
// the average for many samples afterwards.
//
// First sample seeds all three filters so there's no warm-up.
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
