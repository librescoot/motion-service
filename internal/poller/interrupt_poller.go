package poller

import (
	"context"
	"fmt"
	"log/slog"
	"sync/atomic"
	"time"

	"github.com/librescoot/motion-service/internal/bmx"
	"github.com/librescoot/motion-service/internal/redis"
)

// InterruptPoller is the I2C-status watchdog: at a slow tick (100 ms), it
// reads INT_STATUS_0 and emits a MotionEvent if either the slope or
// slow-no-motion engine has fired. Pairs with InterruptWatcher (evdev fast
// path) — the watcher gets there first when the kernel notices the GPIO
// edge; the poller catches anything the watcher missed.
type InterruptPoller struct {
	accel     *bmx.Accelerometer
	publisher *redis.Publisher
	log       *slog.Logger
	enabled   atomic.Bool
}

// NewInterruptPoller creates a new InterruptPoller.
func NewInterruptPoller(
	accel *bmx.Accelerometer,
	publisher *redis.Publisher,
	log *slog.Logger,
) *InterruptPoller {
	return &InterruptPoller{
		accel:     accel,
		publisher: publisher,
		log:       log,
	}
}

// Enable arms the poller. Events that arrive while disabled are dropped.
func (p *InterruptPoller) Enable() {
	p.enabled.Store(true)
	p.log.Info("interrupt poller enabled")
}

// Disable stops the poller from publishing events.
func (p *InterruptPoller) Disable() {
	p.enabled.Store(false)
	p.log.Info("interrupt poller disabled")
}

// Run polls until ctx is cancelled.
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
			if !p.enabled.Load() {
				continue
			}
			if err := p.checkInterrupt(ctx); err != nil {
				p.log.Error("failed to check interrupt", "error", err)
			}
		}
	}
}

// checkInterrupt reads INT_STATUS_0, decides which engine fired, and
// publishes a MotionEvent. Clears the latch afterwards.
func (p *InterruptPoller) checkInterrupt(ctx context.Context) error {
	status, err := p.accel.ReadByteData(bmx.ACCEL_INT_STATUS_0)
	if err != nil {
		return fmt.Errorf("read INT_STATUS_0: %w", err)
	}

	slope := (status & bmx.ACCEL_INT_STATUS_SLOPE) != 0
	slow := (status & bmx.ACCEL_INT_STATUS_SLOW_NO_MOT) != 0
	if !slope && !slow {
		return nil
	}

	engine := engineNameFor(slope, slow)
	ts := time.Now().UnixMilli()
	p.log.Info("motion interrupt detected", "engine", engine, "timestamp", ts)

	if err := p.publisher.PublishMotionEvent(ctx, &redis.MotionEvent{
		Type:      "edge",
		Timestamp: ts,
		Engine:    engine,
	}); err != nil {
		p.log.Error("failed to publish motion event", "error", err)
	}

	if err := p.publisher.UpdateLastInterruptTime(ctx); err != nil {
		p.log.Warn("failed to update last interrupt time", "error", err)
	}

	if err := p.accel.ClearLatchedInterrupt(); err != nil {
		return fmt.Errorf("clear latched interrupt: %w", err)
	}

	return nil
}

// engineNameFor returns the canonical engine string. If both bits are set
// (a quick re-fire across the read window), prefer slope as it's the more
// responsive engine.
func engineNameFor(slope, slow bool) string {
	if slope {
		return "any-motion"
	}
	if slow {
		return "slow-motion"
	}
	return ""
}
