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