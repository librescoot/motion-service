package redis

// SensorAxis is a public motion:sensors JSON value; Unit qualifies all axes.
type SensorAxis struct {
	X         float64 `json:"x"`
	Y         float64 `json:"y"`
	Z         float64 `json:"z"`
	Magnitude float64 `json:"magnitude"`
	Unit      string  `json:"unit"`
}

type SensorReading struct {
	Timestamp int64       `json:"timestamp"`
	Accel     SensorAxis  `json:"accel"`
	Gyro      SensorAxis  `json:"gyro"`
	Mag       *SensorAxis `json:"mag,omitempty"`
}

// MotionEvent is the alarm-facing motion:interrupt contract. Type is "edge"
// or "wake-hibernation"; Engine distinguishes any- from slow-motion.
type MotionEvent struct {
	Type      string `json:"type"`
	Timestamp int64  `json:"timestamp"`
	Engine    string `json:"engine,omitempty"`
}

// HeadingReading is the motion:heading contract. HeadingDeg is the medium EMA;
// consumers should use AccuracyDeg and quality fields rather than treat it as exact.
type HeadingReading struct {
	Timestamp       int64   `json:"timestamp"`
	HeadingDeg      float64 `json:"heading_deg"`
	HeadingRawDeg   float64 `json:"heading_raw_deg"`
	HeadingFastDeg  float64 `json:"heading_fast_deg"`
	HeadingSlowDeg  float64 `json:"heading_slow_deg"`
	AccuracyDeg     float64 `json:"accuracy_deg"`
	TiltCompensated bool    `json:"tilt_compensated"`
	TiltDeg         float64 `json:"tilt_deg"`
	MagStrengthUT   float64 `json:"mag_strength_ut"`
	ExcessG         float64 `json:"excess_g"`
	YawRateDPS      float64 `json:"yaw_rate_dps"`
}
