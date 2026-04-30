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
	magPollingRateHz = 5

	// Tilt-comp inputs are unreliable when the body frame is undergoing
	// significant non-gravity acceleration; ay²+az²+ax² ≠ 1g means the accel
	// vector isn't pure gravity. Above this excess we report tilt_compensated
	// = false and fall back to the X/Y-only heading. Tightened from 0.20 to
	// 0.05 g after observing heading deflection during normal scooter
	// throttle bursts (typical forward acceleration on a 2 kW scooter is
	// 0.1–0.3 g; the looser threshold let those leak into the tilt-comp).
	tiltCompMaxExcessG = 0.05

	// Heading smoothing α. EMA on unit vectors (sin/cos) of the heading;
	// time constant τ = -1/(rate·ln(1-α)). At 5 Hz with α = 0.15, τ ≈ 1.2 s
	// — heavier than the previous 3-sample circular mean (~0.6 s) but
	// follows fast slews (a hard turn) within ~3 s. Tunable per-vehicle.
	headingEMAAlpha = 0.15

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

	// Heading EMA state, on unit vectors so wrap is handled naturally.
	emaSin, emaCos float64
	emaInit        bool

	pollCount int
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
		mag:       mag,
		accel:     accel,
		gyro:      gyro,
		publisher: publisher,
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

	rawX, rawY, rawZ, err := p.mag.ReadData()
	if err != nil {
		return err
	}

	magX, magY, magZ, magMag, err := p.mag.ReadDataInMicroTesla()
	if err != nil {
		return err
	}

	// Pull accel + gyro for tilt comp and quality estimate, in vehicle
	// frame. Either failing just means we publish without that input.
	orientation := p.mag.Orientation()
	var (
		ax, ay, az, aMag float64
		hasAccel         bool
	)
	if p.accel != nil {
		ax, ay, az, aMag, err = p.accel.ReadDataInGVehicleFrame(orientation)
		if err != nil {
			p.log.Warn("accel read failed, skipping tilt comp", "error", err)
		} else {
			hasAccel = true
		}
	}

	var yawRateDPS float64
	if p.gyro != nil {
		_, _, _, gMag, gerr := p.gyro.ReadDataInDPSVehicleFrame(orientation)
		if gerr == nil {
			// Total angular speed; the gyro Z component alone misses cases
			// where the scooter is rolling/pitching. For "is the heading
			// changing fast right now" the magnitude is the better signal.
			yawRateDPS = gMag
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

// smoothHeading applies an EMA filter on the unit-vector representation
// of the heading. EMA on (sin, cos) handles 0°/360° wrap naturally — at
// the wraparound the components transition smoothly, while a linear EMA
// on degrees would jump 360° and contaminate the average for many samples.
//
// The previous version used a 3-sample circular mean. EMA gives a much
// smoother trace for the same effective time constant, has no warm-up
// (initial sample seeds the state), and is tunable via headingEMAAlpha.
func (p *MagPoller) smoothHeading(newHeading float64) float64 {
	rad := newHeading * math.Pi / 180.0
	s := math.Sin(rad)
	c := math.Cos(rad)

	if !p.emaInit {
		p.emaSin, p.emaCos = s, c
		p.emaInit = true
	} else {
		p.emaSin = headingEMAAlpha*s + (1-headingEMAAlpha)*p.emaSin
		p.emaCos = headingEMAAlpha*c + (1-headingEMAAlpha)*p.emaCos
	}

	if p.emaSin == 0 && p.emaCos == 0 {
		return newHeading
	}

	avgDeg := math.Atan2(p.emaSin, p.emaCos) * 180.0 / math.Pi
	return math.Mod(avgDeg+360.0, 360.0)
}
