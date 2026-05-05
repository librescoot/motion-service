package poller

import (
	"context"
	"fmt"
	"log/slog"
	"time"

	"github.com/librescoot/motion-service/internal/bmx"
	"github.com/librescoot/motion-service/internal/redis"
)

// InterruptPoller monitors for motion interrupts
type InterruptPoller struct {
	accel      *bmx.Accelerometer
	gyro       *bmx.Gyroscope
	publisher  *redis.Publisher
	log        *slog.Logger
	config     InterruptConfig
	enabled    bool
}

// InterruptConfig holds current interrupt configuration
type InterruptConfig struct {
	Threshold   byte
	Duration    byte
	Sensitivity string
}

// NewInterruptPoller creates a new InterruptPoller
func NewInterruptPoller(
	accel *bmx.Accelerometer,
	gyro *bmx.Gyroscope,
	publisher *redis.Publisher,
	log *slog.Logger,
) *InterruptPoller {
	return &InterruptPoller{
		accel:     accel,
		gyro:      gyro,
		publisher: publisher,
		log:       log,
		config: InterruptConfig{
			Threshold:   0x00,
			Duration:    0x00,
			Sensitivity: "none",
		},
		enabled: false,
	}
}

// SetConfig updates the interrupt configuration
func (p *InterruptPoller) SetConfig(threshold, duration byte, sensitivity string) {
	p.config.Threshold = threshold
	p.config.Duration = duration
	p.config.Sensitivity = sensitivity
}

// Enable enables interrupt monitoring
func (p *InterruptPoller) Enable() {
	p.enabled = true
	p.log.Info("interrupt monitoring enabled")
}

// Disable disables interrupt monitoring
func (p *InterruptPoller) Disable() {
	p.enabled = false
	p.log.Info("interrupt monitoring disabled")
}

// Run starts the interrupt polling loop
func (p *InterruptPoller) Run(ctx context.Context) {
	p.log.Info("starting interrupt poller")

	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			p.log.Info("interrupt poller stopped")
			return

		case <-ticker.C:
			if p.enabled {
				if err := p.checkInterrupt(ctx); err != nil {
					p.log.Error("failed to check interrupt", "error", err)
				}
			}
		}
	}
}

// checkInterrupt checks if an interrupt has occurred
func (p *InterruptPoller) checkInterrupt(ctx context.Context) error {
	triggered, err := p.accel.GetInterruptStatus()
	if err != nil {
		return err
	}

	if !triggered {
		return nil
	}

	p.log.Info("motion interrupt detected")

	accelX, accelY, accelZ, accelMag, err := p.accel.ReadDataInG()
	if err != nil {
		return err
	}

	gyroX, gyroY, gyroZ, gyroMag, err := p.gyro.ReadDataInDPS()
	if err != nil {
		return err
	}

	event := &redis.InterruptEvent{
		Timestamp:       time.Now().UnixMilli(),
		Type:            "slow_motion",
		InterruptStatus: "0x08",
		SensorValues: redis.SensorValues{
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
		},
		Config: redis.InterruptConfig{
			Threshold:   fmt.Sprintf("0x%02X", p.config.Threshold),
			Duration:    fmt.Sprintf("0x%02X", p.config.Duration),
			Sensitivity: p.config.Sensitivity,
		},
	}

	if err := p.publisher.PublishInterrupt(ctx, event); err != nil {
		return err
	}

	if err := p.publisher.UpdateLastInterruptTime(ctx); err != nil {
		p.log.Warn("failed to update last interrupt time", "error", err)
	}

	if err := p.accel.ClearLatchedInterrupt(); err != nil {
		return fmt.Errorf("failed to clear latched interrupt: %w", err)
	}

	return nil
}