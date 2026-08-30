package bmx

import (
	"sync/atomic"
	"time"
)

type IMUSnapshot struct {
	Timestamp time.Time

	AccelX, AccelY, AccelZ float64
	AccelMag               float64

	GyroX, GyroY, GyroZ float64
	GyroMag             float64
}

type MagSnapshot struct {
	Timestamp time.Time

	CompX, CompY, CompZ int16
	X, Y, Z             float64
	Magnitude           float64
}

type SensorCache struct {
	imu atomic.Pointer[IMUSnapshot]
	mag atomic.Pointer[MagSnapshot]
}

func NewSensorCache() *SensorCache {
	return &SensorCache{}
}

func (c *SensorCache) StoreIMU(s IMUSnapshot) {
	c.imu.Store(&s)
}

func (c *SensorCache) LoadIMU(maxAge time.Duration) (IMUSnapshot, bool) {
	p := c.imu.Load()
	if p == nil || time.Since(p.Timestamp) > maxAge {
		return IMUSnapshot{}, false
	}
	return *p, true
}

func (c *SensorCache) StoreMag(s MagSnapshot) {
	c.mag.Store(&s)
}

func (c *SensorCache) LoadMag(maxAge time.Duration) (MagSnapshot, bool) {
	p := c.mag.Load()
	if p == nil || time.Since(p.Timestamp) > maxAge {
		return MagSnapshot{}, false
	}
	return *p, true
}
