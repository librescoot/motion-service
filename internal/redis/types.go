package redis

// SensorAxis represents a 3-axis sensor reading
type SensorAxis struct {
	X         float64 `json:"x"`
	Y         float64 `json:"y"`
	Z         float64 `json:"z"`
	Magnitude float64 `json:"magnitude"`
	Unit      string  `json:"unit"`
}

// SensorReading represents all sensor data at a point in time
type SensorReading struct {
	Timestamp int64       `json:"timestamp"`
	Accel     SensorAxis  `json:"accel"`
	Gyro      SensorAxis  `json:"gyro"`
	Mag       *SensorAxis `json:"mag,omitempty"`
}

// InterruptConfig represents the interrupt configuration at time of interrupt
type InterruptConfig struct {
	Threshold   string `json:"threshold"`
	Duration    string `json:"duration"`
	Sensitivity string `json:"sensitivity"`
}

// SensorValues represents sensor values at time of interrupt
type SensorValues struct {
	Accel SensorAxis `json:"accel"`
	Gyro  SensorAxis `json:"gyro"`
}

// InterruptEvent represents a motion interrupt event
type InterruptEvent struct {
	ID              string          `json:"id"`
	Timestamp       int64           `json:"timestamp"`
	Type            string          `json:"type"`
	InterruptStatus string          `json:"interrupt_status"`
	SensorValues    SensorValues    `json:"sensor_values"`
	Config          InterruptConfig `json:"config"`
}

// HeadingReading is the rich magnetic-heading payload published on
// the bmx:heading channel. Consumers should weight HeadingDeg by
// AccuracyDeg (or roll their own gating using ExcessG / YawRateDPS /
// TiltDeg) — this is the data needed to do that, not a verdict.
type HeadingReading struct {
	Timestamp       int64   `json:"timestamp"`
	HeadingDeg      float64 `json:"heading_deg"`      // medium EMA, the canonical "smoothed" value
	HeadingRawDeg   float64 `json:"heading_raw_deg"`  // unsmoothed, this sample only
	HeadingFastDeg  float64 `json:"heading_fast_deg"` // fast EMA, τ ≈ 0.3 s — responsive
	HeadingSlowDeg  float64 `json:"heading_slow_deg"` // slow EMA, τ ≈ 3.9 s — stable
	AccuracyDeg     float64 `json:"accuracy_deg"`     // heuristic 1-σ estimate
	TiltCompensated bool    `json:"tilt_compensated"` // false → X/Y-only fallback
	TiltDeg         float64 `json:"tilt_deg"`         // angle from level (0 at rest)
	MagStrengthUT   float64 `json:"mag_strength_ut"`  // |B| in vehicle frame
	ExcessG         float64 `json:"excess_g"`         // ||a|-1g| — non-gravity accel
	YawRateDPS      float64 `json:"yaw_rate_dps"`     // |gyro| total turn rate
}