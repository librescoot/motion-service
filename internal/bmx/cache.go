package bmx

import (
	"sync/atomic"
	"time"
)

// IMUSnapshot is the most recent accel + gyro reading in vehicle frame.
// sensor_poller produces this on every tick; mag_poller consumes it for
// tilt-comp and quality-estimate inputs without having to re-issue accel
// and gyro reads of its own.
type IMUSnapshot struct {
	Timestamp time.Time

	AccelX, AccelY, AccelZ float64 // g, vehicle frame
	AccelMag               float64 // g, frame-invariant

	GyroX, GyroY, GyroZ float64 // °/s, vehicle frame
	GyroMag             float64 // °/s, frame-invariant
}

// MagSnapshot is the most recent magnetometer reading, in both forms the
// mag-poller path uses: the compensated sensor-frame int16 triple (the
// "raw_*" log fields) and the calibrated vehicle-frame µT triple. The
// magnetometer is a comparatively expensive read (8-byte block, full
// trim-data temperature compensation), so caching it lets the two
// pollers share the cost of one read per tick.
type MagSnapshot struct {
	Timestamp time.Time

	CompX, CompY, CompZ int16 // sensor frame, compensated
	X, Y, Z             float64 // vehicle frame, µT
	Magnitude           float64
}

// SensorCache holds the most recent IMU and mag snapshots. The two slots
// are independent atomic.Pointer so sensor_poller and mag_poller can each
// publish whichever sensors they natively read without racing on a shared
// struct.
type SensorCache struct {
	imu atomic.Pointer[IMUSnapshot]
	mag atomic.Pointer[MagSnapshot]
}

// NewSensorCache returns an empty cache.
func NewSensorCache() *SensorCache {
	return &SensorCache{}
}

// StoreIMU atomically replaces the latest accel + gyro snapshot.
func (c *SensorCache) StoreIMU(s IMUSnapshot) {
	c.imu.Store(&s)
}

// LoadIMU returns the most recent IMU snapshot if one was stored within
// `maxAge`, and ok=true. If no snapshot exists or it's older than maxAge,
// ok is false and the caller should fall back to a direct sensor read.
func (c *SensorCache) LoadIMU(maxAge time.Duration) (IMUSnapshot, bool) {
	p := c.imu.Load()
	if p == nil || time.Since(p.Timestamp) > maxAge {
		return IMUSnapshot{}, false
	}
	return *p, true
}

// StoreMag atomically replaces the latest mag snapshot.
func (c *SensorCache) StoreMag(s MagSnapshot) {
	c.mag.Store(&s)
}

// LoadMag returns the most recent mag snapshot if within `maxAge`,
// ok=true. Falls through to false otherwise so the caller can read
// directly.
func (c *SensorCache) LoadMag(maxAge time.Duration) (MagSnapshot, bool) {
	p := c.mag.Load()
	if p == nil || time.Since(p.Timestamp) > maxAge {
		return MagSnapshot{}, false
	}
	return *p, true
}
