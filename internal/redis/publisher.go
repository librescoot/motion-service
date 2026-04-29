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

// PublishSensorData publishes sensor readings to the bmx:sensors channel
func (p *Publisher) PublishSensorData(ctx context.Context, reading *SensorReading) error {
	data, err := json.Marshal(reading)
	if err != nil {
		return fmt.Errorf("failed to marshal sensor data: %w", err)
	}

	if err := p.client.Publish(ctx, "bmx:sensors", string(data)); err != nil {
		return fmt.Errorf("failed to publish sensor data: %w", err)
	}

	return nil
}

// PublishInterrupt publishes an interrupt event to both PUBSUB and Stream
func (p *Publisher) PublishInterrupt(ctx context.Context, event *InterruptEvent) error {
	data, err := json.Marshal(event)
	if err != nil {
		return fmt.Errorf("failed to marshal interrupt event: %w", err)
	}

	if err := p.client.Publish(ctx, "bmx:interrupt", string(data)); err != nil {
		p.log.Error("failed to publish interrupt to PUBSUB", "error", err)
	}

	values := map[string]interface{}{
		"timestamp":        event.Timestamp,
		"type":             event.Type,
		"interrupt_status": event.InterruptStatus,
		"data":             string(data),
	}

	streamID, err := p.client.XAdd(ctx, "bmx:events", values)
	if err != nil {
		return fmt.Errorf("failed to add interrupt to stream: %w", err)
	}

	p.log.Debug("published interrupt event", "stream_id", streamID)
	return nil
}

// PublishStatus publishes status information to the bmx hash
func (p *Publisher) PublishStatus(ctx context.Context, status map[string]string) error {
	values := make(map[string]interface{})
	for k, v := range status {
		values[k] = v
	}

	if err := p.client.HSetMultiple(ctx, "bmx", values); err != nil {
		return fmt.Errorf("failed to publish status: %w", err)
	}

	return nil
}

// UpdateStatusField updates a single field in the bmx hash
func (p *Publisher) UpdateStatusField(ctx context.Context, field string, value string) error {
	if err := p.client.HSet(ctx, "bmx", field, value); err != nil {
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

	countStr, err := p.client.HGet(ctx, "bmx", "error-count")
	if err != nil {
		countStr = "0"
	}

	var count int
	fmt.Sscanf(countStr, "%d", &count)
	count++

	return p.UpdateStatusField(ctx, "error-count", fmt.Sprintf("%d", count))
}


// PublishHeading publishes a tilt-compensated heading reading on the
// bmx:heading PUBSUB channel (JSON) and updates the bmx hash with both the
// fractional heading and the legacy integer "heading" field, plus the
// quality fields a consumer needs to gate on.
func (p *Publisher) PublishHeading(ctx context.Context, reading *HeadingReading) error {
	data, err := json.Marshal(reading)
	if err != nil {
		return fmt.Errorf("failed to marshal heading: %w", err)
	}

	if err := p.client.Publish(ctx, "bmx:heading", string(data)); err != nil {
		return fmt.Errorf("failed to publish heading: %w", err)
	}

	hash := map[string]interface{}{
		"heading":          fmt.Sprintf("%d", int(math.Mod(reading.HeadingDeg+360.0, 360.0))),
		"heading-deg":      fmt.Sprintf("%.2f", reading.HeadingDeg),
		"heading-accuracy": fmt.Sprintf("%.2f", reading.AccuracyDeg),
		"heading-tilt":     fmt.Sprintf("%.2f", reading.TiltDeg),
		"heading-tilt-comp": map[bool]string{true: "true", false: "false"}[reading.TiltCompensated],
	}
	if err := p.client.HSetMultiple(ctx, "bmx", hash); err != nil {
		return fmt.Errorf("failed to update heading hash: %w", err)
	}
	return nil
}