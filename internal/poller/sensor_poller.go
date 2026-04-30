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
		// Default on — bmx-service is the primary IMU producer in the
		// post-split architecture, and downstream consumers (the debug
		// screen, future heading code in scootui-qt) need a continuous
		// stream out of the box. The streaming:disable command is still
		// available for callers that want to silence it.
		enabled: true,
		log:     log,
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

// poll reads all sensors in vehicle frame and publishes the data. The
// orientation comes from the magnetometer's calibration (the only place
// it lives) — falls back to identity if no mag is wired so the service
// still works without it.
func (p *SensorPoller) poll(ctx context.Context) error {
	orientation := bmx.Orientation{
		AxisOrder: [3]int{0, 1, 2},
		AxisSign:  [3]float64{1, 1, 1},
	}
	if p.mag != nil {
		orientation = p.mag.Orientation()
	}

	accelX, accelY, accelZ, accelMag, err := p.accel.ReadDataInGVehicleFrame(orientation)
	if err != nil {
		return err
	}

	gyroX, gyroY, gyroZ, gyroMag, err := p.gyro.ReadDataInDPSVehicleFrame(orientation)
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