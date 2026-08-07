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

// legacyStatusFields are motion-hash fields that only the retired
// scooter:motion command handlers ever wrote. They are deleted once at
// startup so units upgrading from an older build don't keep serving a
// stale "sensitivity: none" next to a live current-profile.
var legacyStatusFields = []string{"sensitivity"}

// Publisher writes motion state to Redis: the `motion` hash plus the
// motion:* pub/sub channels.
//
// Everything rides the shared redis-ipc client, so the hash writes and
// channel publishes reuse the one connection pool the watchers and the
// RPC server already use.
type Publisher struct {
	client *ipc.Client
	hash   *ipc.HashPublisher
	log    *slog.Logger
}

// NewPublisher creates a Publisher bound to the shared redis-ipc client.
func NewPublisher(client *ipc.Client, log *slog.Logger) *Publisher {
	return &Publisher{
		client: client,
		hash:   client.NewHashPublisher("motion"),
		log:    log,
	}
}

// PruneLegacyFields removes motion-hash fields that no longer have a
// writer. Best-effort: a failure here is cosmetic.
func (p *Publisher) PruneLegacyFields(ctx context.Context) {
	for _, f := range legacyStatusFields {
		// Raw HDEL rather than HashPublisher.Delete: the latter always
		// publishes the field name on the hash's channel, and nothing
		// should see a notification for a field being retired.
		if _, err := p.client.Do("HDEL", "motion", f); err != nil {
			p.log.Debug("prune legacy status field failed", "field", f, "error", err)
		}
	}
}

// PublishSensorData publishes sensor readings to the motion:sensors channel
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

// PublishMotionEvent publishes a slim envelope on the motion:interrupt
// channel. This is the consumer-facing interface for alarm-service and
// any other motion-event subscriber.
//
// Published synchronously: alarm-service branches its FSM on this, so a
// dropped event matters more here than the latency does.
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

// PublishReady fires once at startup after the first profile has been
// programmed. Observability + lets consumers know the chip is in a
// known state.
func (p *Publisher) PublishReady(ctx context.Context) error {
	if _, err := p.client.Publish("motion:ready", fmt.Sprintf("%d", time.Now().UnixMilli()), ipc.Sync()); err != nil {
		return fmt.Errorf("failed to publish ready: %w", err)
	}
	return nil
}

// PublishStatus publishes status information to the motion hash
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

// UpdateStatusField updates a single field in the motion hash
func (p *Publisher) UpdateStatusField(ctx context.Context, field string, value string) error {
	if err := p.hash.Set(field, value, ipc.NoPublish(), ipc.Sync()); err != nil {
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

	countStr, err := p.hash.Get("error-count")
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
