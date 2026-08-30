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

type InterruptPoller struct {
	accel     *bmx.Accelerometer
	publisher *redis.Publisher
	log       *slog.Logger
	enabled   atomic.Bool
}

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

func (p *InterruptPoller) Enable() {
	p.enabled.Store(true)
	p.log.Info("interrupt poller enabled")
}

func (p *InterruptPoller) Disable() {
	p.enabled.Store(false)
	p.log.Info("interrupt poller disabled")
}

func (p *InterruptPoller) Run(ctx context.Context) {
	p.log.Info("starting interrupt poller")

	ticker := time.NewTicker(1 * time.Second)
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

func engineNameFor(slope, slow bool) string {
	if slope {
		return "any-motion"
	}
	if slow {
		return "slow-motion"
	}
	return ""
}
