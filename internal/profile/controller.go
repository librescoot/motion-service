package profile

import (
	"context"
	"fmt"
	"log/slog"
	"sync"
	"time"

	"github.com/librescoot/motion-service/internal/bmx"
)

// InterruptSource is something that can be gated in lockstep with chip
// reconfiguration — both the I2C poller and the evdev watcher implement it.
type InterruptSource interface {
	Enable()
	Disable()
}

// Publisher is the subset of redis.Publisher the controller needs for
// status reporting. Defined here to keep the controller package free of
// its own redis import dependency cycle.
type Publisher interface {
	UpdateStatusField(ctx context.Context, field, value string) error
}

// Controller owns the BMX055 chip configuration. It applies profiles
// idempotently (re-applying the same profile is a no-op) and keeps the
// poller + watcher gated in lockstep with the engine state.
type Controller struct {
	accel     *bmx.Accelerometer
	gyro      *bmx.Gyroscope
	poller    InterruptSource
	watcher   InterruptSource // nil if the evdev device wasn't available at startup
	publisher Publisher
	log       *slog.Logger

	mu       sync.Mutex
	current  Profile
	hasFirst bool // true once any profile has been applied at least once
}

// New returns a Controller. watcher may be nil — the I2C poller alone is
// sufficient on hardware where the gpio-keys evdev device isn't wired up.
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

// Current returns the most recently applied profile. Returns Idle if no
// profile has been applied yet.
func (c *Controller) Current() Profile {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.current
}

// Apply reconfigures the chip for the given profile. Soft-reset → set
// bandwidth → configure motion engine → 100 ms settle + double-clear-latch
// → map to INT pins → enable interrupt sources. Idempotent — re-applying
// the same profile after the first call is a no-op unless force is true.
//
// The 100 ms settle window is load-bearing: the bandwidth change kicks off
// a low-pass filter settle that can produce a transient slope large enough
// to set the status bit. If the pin mapping is in place during that
// transient, the INT line spikes from a stale status bit and the gpio-keys
// edge fires before the status is cleared — false wake + the real first
// edge gets hidden. Clear twice across the settle delay BEFORE adding the
// engine to the pin map.
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

	// Disable poller + watcher before reconfiguring so they don't observe
	// transient INT activity from the bandwidth-settle period.
	c.poller.Disable()
	if c.watcher != nil {
		c.watcher.Disable()
	}

	// Step 1: soft-reset both accel and gyro. Gyro reset verifies chip
	// responsiveness itself (see Gyroscope.SoftReset) and its chip-ID poll
	// waits >= 10 ms, which also covers the accel's ~2 ms restart before
	// the reconfiguration below.
	if err := c.accel.SoftReset(); err != nil {
		return fmt.Errorf("accel soft reset: %w", err)
	}
	if err := c.gyro.SoftReset(); err != nil {
		return fmt.Errorf("gyro soft reset: %w", err)
	}

	// Step 1b: re-apply the gyro config the soft reset wiped (range/filter).
	// Otherwise ReadDataInDPS's ±500°/s scale no longer matches the chip's
	// default ±2000°/s and gyro rates read 4x too small.
	if err := c.gyro.Configure(); err != nil {
		return fmt.Errorf("configure gyro: %w", err)
	}

	// Step 2: bandwidth.
	if err := c.accel.SetBandwidth(spec.Sensor.Bandwidth); err != nil {
		return fmt.Errorf("set bandwidth: %w", err)
	}

	// Step 3: configure the active engine, disable the other one.
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

	// Step 4: pin output config (active-high, latched).
	if spec.InterruptPin != bmx.InterruptPinNone {
		if err := c.accel.ConfigureInterruptPins(spec.InterruptPin, true); err != nil {
			return fmt.Errorf("configure interrupt pins: %w", err)
		}
	}

	// Step 5: 100 ms settle + double-clear-latch.
	if err := c.accel.ClearLatchedInterrupt(); err != nil {
		c.log.Warn("first latch-clear failed", "error", err)
	}
	time.Sleep(100 * time.Millisecond)
	if err := c.accel.ClearLatchedInterrupt(); err != nil {
		c.log.Warn("second latch-clear failed", "error", err)
	}

	// Step 6: route the engine to the configured pin (or disable mapping).
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

	// Step 7: re-arm poller + watcher only if interrupt is enabled.
	if spec.EnableInterrupt {
		c.poller.Enable()
		if c.watcher != nil {
			c.watcher.Enable()
		}
	}

	c.current = p
	c.hasFirst = true

	if c.publisher != nil {
		_ = c.publisher.UpdateStatusField(ctx, "current-profile", p.String())
	}

	c.log.Info("profile applied", "profile", p.String())
	return nil
}
