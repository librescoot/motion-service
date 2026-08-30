package redis

import (
	"context"
	"encoding/json"
	"fmt"
	"log/slog"
	"math"
	"time"

	ipc "github.com/librescoot/redis-ipc"
)

var legacyStatusFields = []string{"sensitivity"}

type Publisher struct {
	client *ipc.Client
	hash   *ipc.HashPublisher
	log    *slog.Logger
}

func NewPublisher(client *ipc.Client, log *slog.Logger) *Publisher {
	return &Publisher{
		client: client,
		hash:   client.NewHashPublisher("motion"),
		log:    log,
	}
}

func (p *Publisher) PruneLegacyFields(ctx context.Context) {
	for _, f := range legacyStatusFields {

		if _, err := p.client.Do("HDEL", "motion", f); err != nil {
			p.log.Debug("prune legacy status field failed", "field", f, "error", err)
		}
	}
}

func (p *Publisher) PublishSensorData(ctx context.Context, reading *SensorReading) error {
	data, err := json.Marshal(reading)
	if err != nil {
		return fmt.Errorf("failed to marshal sensor data: %w", err)
	}

	if _, err := p.client.Publish("motion:sensors", string(data), ipc.Sync()); err != nil {
		return fmt.Errorf("failed to publish sensor data: %w", err)
	}

	return nil
}

func (p *Publisher) PublishMotionEvent(ctx context.Context, event *MotionEvent) error {
	data, err := json.Marshal(event)
	if err != nil {
		return fmt.Errorf("failed to marshal motion event: %w", err)
	}
	if _, err := p.client.Publish("motion:interrupt", string(data), ipc.Sync()); err != nil {
		return fmt.Errorf("failed to publish motion event: %w", err)
	}
	return nil
}

func (p *Publisher) PublishReady(ctx context.Context) error {
	if _, err := p.client.Publish("motion:ready", fmt.Sprintf("%d", time.Now().UnixMilli()), ipc.Sync()); err != nil {
		return fmt.Errorf("failed to publish ready: %w", err)
	}
	return nil
}

func (p *Publisher) PublishStatus(ctx context.Context, status map[string]string) error {
	values := make(map[string]any, len(status))
	for k, v := range status {
		values[k] = v
	}

	if err := p.hash.SetMany(values, ipc.NoPublish(), ipc.Sync()); err != nil {
		return fmt.Errorf("failed to publish status: %w", err)
	}

	return nil
}

func (p *Publisher) UpdateStatusField(ctx context.Context, field string, value string) error {
	if err := p.hash.Set(field, value, ipc.NoPublish(), ipc.Sync()); err != nil {
		return fmt.Errorf("failed to update status field %s: %w", field, err)
	}
	return nil
}

func (p *Publisher) UpdateLastInterruptTime(ctx context.Context) error {
	timestamp := time.Now().UnixMilli()
	return p.UpdateStatusField(ctx, "last-interrupt-timestamp", fmt.Sprintf("%d", timestamp))
}

func (p *Publisher) IncrementErrorCount(ctx context.Context, errorMsg string) error {
	if err := p.UpdateStatusField(ctx, "last-error", errorMsg); err != nil {
		return err
	}

	countStr, err := p.hash.Get("error-count")
	if err != nil {
		countStr = "0"
	}

	var count int
	_, _ = fmt.Sscanf(countStr, "%d", &count)
	count++

	return p.UpdateStatusField(ctx, "error-count", fmt.Sprintf("%d", count))
}

func (p *Publisher) PublishHeading(ctx context.Context, reading *HeadingReading) error {
	data, err := json.Marshal(reading)
	if err != nil {
		return fmt.Errorf("failed to marshal heading: %w", err)
	}

	if _, err := p.client.Publish("motion:heading", string(data), ipc.Sync()); err != nil {
		return fmt.Errorf("failed to publish heading: %w", err)
	}

	hash := map[string]any{
		"heading":           fmt.Sprintf("%d", int(math.Mod(reading.HeadingDeg+360.0, 360.0))),
		"heading-deg":       fmt.Sprintf("%.2f", reading.HeadingDeg),
		"heading-accuracy":  fmt.Sprintf("%.2f", reading.AccuracyDeg),
		"heading-tilt":      fmt.Sprintf("%.2f", reading.TiltDeg),
		"heading-tilt-comp": map[bool]string{true: "true", false: "false"}[reading.TiltCompensated],
	}
	if err := p.hash.SetMany(hash, ipc.NoPublish(), ipc.Sync()); err != nil {
		return fmt.Errorf("failed to update heading hash: %w", err)
	}
	return nil
}
