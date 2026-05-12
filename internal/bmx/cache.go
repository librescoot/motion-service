package bmx

import (
	"sync/atomic"
	"time"
)

// SensorSnapshot is the most recent accel + gyro reading already transformed
// into the vehicle frame. mag_poller's tilt-comp and quality-estimate inputs
// are the same numbers sensor_poller just read 50–100 ms ago, so caching the
// snapshot lets mag_poller reuse it instead of issuing another 12 I2C
// transactions per cycle.
type SensorSnapshot struct {
	Timestamp time.Time

	AccelX, AccelY, AccelZ float64 // g, vehicle frame
	AccelMag               float64 // g, frame-invariant

	GyroX, GyroY, GyroZ float64 // °/s, vehicle frame
	GyroMag             float64 // °/s, frame-invariant
}

// SensorCache holds the most recent vehicle-frame sensor reading. Lockless
// via atomic.Pointer; writers Store a fresh snapshot, readers Load the
// latest pointer and check Timestamp for staleness.
type SensorCache struct {
	p atomic.Pointer[SensorSnapshot]
}

// NewSensorCache returns an empty cache.
func NewSensorCache() *SensorCache {
	return &SensorCache{}
}

// Store atomically replaces the latest snapshot.
func (c *SensorCache) Store(s SensorSnapshot) {
	c.p.Store(&s)
}

// Load returns the most recent snapshot if one was stored within `maxAge`,
// and ok=true. If no snapshot exists or it's older than maxAge, ok is false
// and the caller should fall back to a direct sensor read.
func (c *SensorCache) Load(maxAge time.Duration) (SensorSnapshot, bool) {
	p := c.p.Load()
	if p == nil {
		return SensorSnapshot{}, false
	}
	if time.Since(p.Timestamp) > maxAge {
		return SensorSnapshot{}, false
	}
	return *p, true
}
