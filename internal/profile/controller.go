package profile

import (
	"context"
	"fmt"
	"log/slog"
	"sync"
	"time"

	"github.com/librescoot/motion-service/internal/bmx"
)

type InterruptSource interface {
	Enable()
	Disable()
}

type Publisher interface {
	UpdateStatusField(ctx context.Context, field, value string) error
}

type Controller struct {
	accel     *bmx.Accelerometer
	gyro      *bmx.Gyroscope
	poller    InterruptSource
	watcher   InterruptSource
	publisher Publisher
	log       *slog.Logger

	mu       sync.Mutex
	current  Profile
	hasFirst bool
}

func New(accel *bmx.Accelerometer, gyro *bmx.Gyroscope, poller InterruptSource, watcher InterruptSource, publisher Publisher, log *slog.Logger) *Controller {
	return &Controller{
		accel:     accel,
		gyro:      gyro,
		poller:    poller,
		watcher:   watcher,
		publisher: publisher,
		log:       log,
	}
}

func (c *Controller) Current() Profile {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.current
}

// Invalidate is required after an out-of-band register write (especially reset),
// otherwise idempotence would leave an armed chip with motion detection erased.
func (c *Controller) Invalidate() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.hasFirst = false
}

// Apply gates both interrupt sources while reset, engine configuration, and pin
// routing are in flight; only a fully configured profile may publish edges.
func (c *Controller) Apply(ctx context.Context, p Profile) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if c.hasFirst && c.current == p {
		c.log.Debug("profile unchanged, skipping apply", "profile", p.String())
		return nil
	}

	spec := Configs(p)
	c.log.Info("applying profile",
		"profile", p.String(),
		"mode", spec.Sensor.Mode.String(),
		"bw", fmt.Sprintf("0x%02X", spec.Sensor.Bandwidth),
		"threshold", fmt.Sprintf("0x%02X", spec.Sensor.Threshold),
		"duration", fmt.Sprintf("0x%02X", spec.Sensor.Duration),
		"pin", spec.InterruptPin.String(),
		"enable", spec.EnableInterrupt,
	)

	// Prevent reset and filter-settle transients from becoming motion events.
	c.poller.Disable()
	if c.watcher != nil {
		c.watcher.Disable()
	}

	if err := c.accel.SoftReset(); err != nil {
		return fmt.Errorf("accel soft reset: %w", err)
	}
	if err := c.gyro.SoftReset(); err != nil {
		return fmt.Errorf("gyro soft reset: %w", err)
	}

	// Reset restores ±2000°/s defaults, not the reader's configured scale.
	if err := c.gyro.Configure(); err != nil {
		return fmt.Errorf("configure gyro: %w", err)
	}

	if err := c.accel.SetBandwidth(spec.Sensor.Bandwidth); err != nil {
		return fmt.Errorf("set bandwidth: %w", err)
	}

	switch spec.Sensor.Mode {
	case bmx.InterruptModeAnyMotion:
		if err := c.accel.DisableSlowNoMotionInterrupt(); err != nil {
			return fmt.Errorf("disable slow-motion: %w", err)
		}
		if err := c.accel.EnableAnyMotionInterrupt(spec.Sensor.Threshold, spec.Sensor.Duration); err != nil {
			return fmt.Errorf("enable any-motion: %w", err)
		}
	case bmx.InterruptModeSlowMotion:
		if err := c.accel.DisableAnyMotionInterrupt(); err != nil {
			return fmt.Errorf("disable any-motion: %w", err)
		}
		if err := c.accel.ConfigureSlowNoMotion(spec.Sensor.Threshold, spec.Sensor.Duration); err != nil {
			return fmt.Errorf("configure slow-motion: %w", err)
		}
	}

	if spec.InterruptPin != bmx.InterruptPinNone {
		if err := c.accel.ConfigureInterruptPins(spec.InterruptPin, true); err != nil {
			return fmt.Errorf("configure interrupt pins: %w", err)
		}
	}

	// Clear before and after the 100 ms bandwidth/filter settle period.
	if err := c.accel.ClearLatchedInterrupt(); err != nil {
		c.log.Warn("first latch-clear failed", "error", err)
	}
	// The accelerometer can reassert a latch while its reset settles.
	time.Sleep(100 * time.Millisecond)
	if err := c.accel.ClearLatchedInterrupt(); err != nil {
		c.log.Warn("second latch-clear failed", "error", err)
	}

	if spec.EnableInterrupt {
		switch spec.Sensor.Mode {
		case bmx.InterruptModeAnyMotion:
			if err := c.accel.MapAnyMotionToPins(spec.InterruptPin); err != nil {
				return fmt.Errorf("map any-motion: %w", err)
			}
		case bmx.InterruptModeSlowMotion:
			if err := c.accel.MapInterruptToPins(spec.InterruptPin); err != nil {
				return fmt.Errorf("map slow-motion: %w", err)
			}
			if err := c.accel.EnableSlowNoMotionInterrupt(true); err != nil {
				return fmt.Errorf("enable slow-motion interrupt: %w", err)
			}
		}
	} else {
		if err := c.accel.DisableInterruptMapping(); err != nil {
			return fmt.Errorf("disable interrupt mapping: %w", err)
		}
	}

	if spec.EnableInterrupt {
		c.poller.Enable()
		if c.watcher != nil {
			c.watcher.Enable()
		}
	}

	c.current = p
	c.hasFirst = true

	// Report register reality, not stale manual-command state.
	if c.publisher != nil {
		for field, value := range map[string]string{
			"current-profile": p.String(),
			"mode":            spec.Sensor.Mode.String(),
			"bandwidth":       fmt.Sprintf("0x%02X", spec.Sensor.Bandwidth),
			"threshold":       fmt.Sprintf("0x%02X", spec.Sensor.Threshold),
			"duration":        fmt.Sprintf("0x%02X", spec.Sensor.Duration),
			"pin":             spec.InterruptPin.String(),
			"interrupt":       map[bool]string{true: "enabled", false: "disabled"}[spec.EnableInterrupt],
		} {
			_ = c.publisher.UpdateStatusField(ctx, field, value)
		}
	}

	c.log.Info("profile applied", "profile", p.String())
	return nil
}
