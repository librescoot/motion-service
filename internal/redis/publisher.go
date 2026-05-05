package redis

import (
	"context"
	"encoding/json"
	"fmt"
	"log/slog"
	"math"
	"time"
)

// Publisher handles publishing sensor data and events to Redis
type Publisher struct {
	client *Client
	log    *slog.Logger
}

// NewPublisher creates a new Publisher
func NewPublisher(client *Client, log *slog.Logger) *Publisher {
	return &Publisher{
		client: client,
		log:    log,
	}
}

// PublishSensorData publishes sensor readings to the motion:sensors channel
func (p *Publisher) PublishSensorData(ctx context.Context, reading *SensorReading) error {
	data, err := json.Marshal(reading)
	if err != nil {
		return fmt.Errorf("failed to marshal sensor data: %w", err)
	}

	if err := p.client.Publish(ctx, "motion:sensors", string(data)); err != nil {
		return fmt.Errorf("failed to publish sensor data: %w", err)
	}

	return nil
}

// PublishMotionEvent publishes a slim envelope on the motion:interrupt
// channel. This is the consumer-facing interface for alarm-service and
// any other motion-event subscriber.
func (p *Publisher) PublishMotionEvent(ctx context.Context, event *MotionEvent) error {
	data, err := json.Marshal(event)
	if err != nil {
		return fmt.Errorf("failed to marshal motion event: %w", err)
	}
	if err := p.client.Publish(ctx, "motion:interrupt", string(data)); err != nil {
		return fmt.Errorf("failed to publish motion event: %w", err)
	}
	return nil
}

// PublishReady fires once at startup after the first profile has been
// programmed. Observability + lets consumers know the chip is in a
// known state.
func (p *Publisher) PublishReady(ctx context.Context) error {
	if err := p.client.Publish(ctx, "motion:ready", fmt.Sprintf("%d", time.Now().UnixMilli())); err != nil {
		return fmt.Errorf("failed to publish ready: %w", err)
	}
	return nil
}

// PublishStatus publishes status information to the motion hash
func (p *Publisher) PublishStatus(ctx context.Context, status map[string]string) error {
	values := make(map[string]interface{})
	for k, v := range status {
		values[k] = v
	}

	if err := p.client.HSetMultiple(ctx, "motion", values); err != nil {
		return fmt.Errorf("failed to publish status: %w", err)
	}

	return nil
}

// UpdateStatusField updates a single field in the motion hash
func (p *Publisher) UpdateStatusField(ctx context.Context, field string, value string) error {
	if err := p.client.HSet(ctx, "motion", field, value); err != nil {
		return fmt.Errorf("failed to update status field %s: %w", field, err)
	}
	return nil
}

// UpdateLastInterruptTime updates the last interrupt timestamp
func (p *Publisher) UpdateLastInterruptTime(ctx context.Context) error {
	timestamp := time.Now().UnixMilli()
	return p.UpdateStatusField(ctx, "last-interrupt-timestamp", fmt.Sprintf("%d", timestamp))
}

// IncrementErrorCount increments the error counter
func (p *Publisher) IncrementErrorCount(ctx context.Context, errorMsg string) error {
	if err := p.UpdateStatusField(ctx, "last-error", errorMsg); err != nil {
		return err
	}

	countStr, err := p.client.HGet(ctx, "motion", "error-count")
	if err != nil {
		countStr = "0"
	}

	var count int
	fmt.Sscanf(countStr, "%d", &count)
	count++

	return p.UpdateStatusField(ctx, "error-count", fmt.Sprintf("%d", count))
}


// PublishHeading publishes a tilt-compensated heading reading on the
// motion:heading PUBSUB channel (JSON) and updates the motion hash with both the
// fractional heading and the legacy integer "heading" field, plus the
// quality fields a consumer needs to gate on.
func (p *Publisher) PublishHeading(ctx context.Context, reading *HeadingReading) error {
	data, err := json.Marshal(reading)
	if err != nil {
		return fmt.Errorf("failed to marshal heading: %w", err)
	}

	if err := p.client.Publish(ctx, "motion:heading", string(data)); err != nil {
		return fmt.Errorf("failed to publish heading: %w", err)
	}

	hash := map[string]interface{}{
		"heading":          fmt.Sprintf("%d", int(math.Mod(reading.HeadingDeg+360.0, 360.0))),
		"heading-deg":      fmt.Sprintf("%.2f", reading.HeadingDeg),
		"heading-accuracy": fmt.Sprintf("%.2f", reading.AccuracyDeg),
		"heading-tilt":     fmt.Sprintf("%.2f", reading.TiltDeg),
		"heading-tilt-comp": map[bool]string{true: "true", false: "false"}[reading.TiltCompensated],
	}
	if err := p.client.HSetMultiple(ctx, "motion", hash); err != nil {
		return fmt.Errorf("failed to update heading hash: %w", err)
	}
	return nil
}