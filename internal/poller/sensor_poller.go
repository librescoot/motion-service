package poller

import (
	"context"
	"log/slog"
	"sync"
	"time"

	"bmx-service/internal/bmx"
	"bmx-service/internal/redis"
)

// SensorPoller continuously polls sensors and publishes data
type SensorPoller struct {
	accel     *bmx.Accelerometer
	gyro      *bmx.Gyroscope
	mag       *bmx.Magnetometer
	publisher *redis.Publisher
	rateHz    int
	enabled   bool
	mu        sync.RWMutex
	log       *slog.Logger
}

// NewSensorPoller creates a new SensorPoller
func NewSensorPoller(
	accel *bmx.Accelerometer,
	gyro *bmx.Gyroscope,
	mag *bmx.Magnetometer,
	publisher *redis.Publisher,
	rateHz int,
	log *slog.Logger,
) *SensorPoller {
	return &SensorPoller{
		accel:     accel,
		gyro:      gyro,
		mag:       mag,
		publisher: publisher,
		rateHz:    rateHz,
		enabled:   false,
		log:       log,
	}
}

// Enable enables sensor streaming
func (p *SensorPoller) Enable() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.enabled = true
	p.log.Info("sensor streaming enabled")
}

// Disable disables sensor streaming
func (p *SensorPoller) Disable() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.enabled = false
	p.log.Info("sensor streaming disabled")
}

// IsEnabled returns whether sensor streaming is enabled
func (p *SensorPoller) IsEnabled() bool {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.enabled
}

// SetRate updates the polling rate
func (p *SensorPoller) SetRate(rateHz int) {
	p.rateHz = rateHz
	p.log.Info("sensor polling rate updated", "rate_hz", rateHz)
}

// Run starts the sensor polling loop
func (p *SensorPoller) Run(ctx context.Context) {
	p.log.Info("starting sensor poller", "rate_hz", p.rateHz)

	ticker := time.NewTicker(time.Second / time.Duration(p.rateHz))
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			p.log.Info("sensor poller stopped")
			return

		case <-ticker.C:
			if p.IsEnabled() {
				if err := p.poll(ctx); err != nil {
					p.log.Error("failed to poll sensors", "error", err)
				}
			}
		}
	}
}

// poll reads all sensors and publishes the data
func (p *SensorPoller) poll(ctx context.Context) error {
	accelX, accelY, accelZ, accelMag, err := p.accel.ReadDataInG()
	if err != nil {
		return err
	}

	gyroX, gyroY, gyroZ, gyroMag, err := p.gyro.ReadDataInDPS()
	if err != nil {
		return err
	}

	reading := &redis.SensorReading{
		Timestamp: time.Now().UnixMilli(),
		Accel: redis.SensorAxis{
			X:         accelX,
			Y:         accelY,
			Z:         accelZ,
			Magnitude: accelMag,
			Unit:      "g",
		},
		Gyro: redis.SensorAxis{
			X:         gyroX,
			Y:         gyroY,
			Z:         gyroZ,
			Magnitude: gyroMag,
			Unit:      "deg/s",
		},
	}

	if p.mag != nil {
		magX, magY, magZ, magMag, err := p.mag.ReadDataInMicroTesla()
		if err == nil {
			reading.Mag = &redis.SensorAxis{
				X:         magX,
				Y:         magY,
				Z:         magZ,
				Magnitude: magMag,
				Unit:      "uT",
			}
		}
	}

	return p.publisher.PublishSensorData(ctx, reading)
}