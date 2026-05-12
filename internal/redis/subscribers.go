package redis

import (
	"context"
	"fmt"
	"log/slog"
	"sync"
	"time"

	ipc "github.com/librescoot/redis-ipc"

	"github.com/librescoot/motion-service/internal/profile"
)

// ProfileApplier is the subset of profile.Controller the Subscriber needs.
// Defined here to avoid import cycles and to make testing easy.
type ProfileApplier interface {
	Apply(ctx context.Context, p profile.Profile) error
}

// RateSetter is the subset of poller.SensorPoller the Subscriber drives.
// SetRate(0) means "suspend"; positive values are Hz.
type RateSetter interface {
	SetRate(rateHz int)
}

// Vehicle-state → sensor-poller rate mapping. "parked" and
// "ready-to-drive" are the only states with a downstream consumer
// actively watching the live stream (Qt dashboard tilt indicator, alarm
// classifier confirmation). Everything else (stand-by, shutting-down,
// suspending, etc.) gets the slow heartbeat — the chip's own motion
// engines handle wake-on-motion via the alarm-driven interrupt path,
// so the host doesn't need a continuous stream.
const (
	FastPollRateHz = 5
	SlowPollRateHz = 1
)

// Subscriber watches the `alarm`, `power-manager`, and `vehicle` hashes
// and reactively (a) reconfigures the BMX055 by deriving the appropriate
// profile from the (alarm.status, power-manager.state) pair, and
// (b) adjusts the sensor poller's cadence based on vehicle.state. Uses
// redis-ipc HashWatcher with a 50 ms debounce so rapid status changes
// don't thrash the chip.
type Subscriber struct {
	ipcClient  *ipc.Client
	controller ProfileApplier
	rate       RateSetter // optional; nil disables vehicle-state-driven rate
	log        *slog.Logger

	alarmW   *ipc.HashWatcher
	pmW      *ipc.HashWatcher
	vehicleW *ipc.HashWatcher

	mu          sync.Mutex
	alarmStatus string
	pmState     string
}

// NewSubscriber returns a Subscriber bound to the given redis-ipc client,
// profile controller, and rate setter. `rate` may be nil to skip the
// vehicle-state subscription.
func NewSubscriber(ipcClient *ipc.Client, controller ProfileApplier, rate RateSetter, log *slog.Logger) *Subscriber {
	return &Subscriber{
		ipcClient:  ipcClient,
		controller: controller,
		rate:       rate,
		log:        log,
	}
}

// Start subscribes to both hashes. StartWithSync issues an HGETALL on each
// so the very first apply reflects the current vehicle state, not just
// future updates.
func (s *Subscriber) Start() error {
	s.alarmW = s.ipcClient.NewHashWatcher("alarm")
	s.alarmW.SetDebounce(50 * time.Millisecond)
	s.alarmW.OnField("status", s.handleAlarmStatus)
	if err := s.alarmW.StartWithSync(); err != nil {
		return fmt.Errorf("start alarm watcher: %w", err)
	}

	s.pmW = s.ipcClient.NewHashWatcher("power-manager")
	s.pmW.SetDebounce(50 * time.Millisecond)
	s.pmW.OnField("state", s.handlePmState)
	if err := s.pmW.StartWithSync(); err != nil {
		s.alarmW.Stop()
		return fmt.Errorf("start power-manager watcher: %w", err)
	}

	if s.rate != nil {
		s.vehicleW = s.ipcClient.NewHashWatcher("vehicle")
		s.vehicleW.SetDebounce(50 * time.Millisecond)
		s.vehicleW.OnField("state", s.handleVehicleState)
		if err := s.vehicleW.StartWithSync(); err != nil {
			s.alarmW.Stop()
			s.pmW.Stop()
			return fmt.Errorf("start vehicle watcher: %w", err)
		}
	}

	s.log.Info("alarm + power-manager + vehicle state watchers started")
	return nil
}

// Stop tears down all watchers.
func (s *Subscriber) Stop() {
	if s.alarmW != nil {
		s.alarmW.Stop()
	}
	if s.pmW != nil {
		s.pmW.Stop()
	}
	if s.vehicleW != nil {
		s.vehicleW.Stop()
	}
}

func (s *Subscriber) handleAlarmStatus(value string) error {
	s.mu.Lock()
	prev := s.alarmStatus
	s.alarmStatus = value
	p := profile.Derive(s.alarmStatus, s.pmState)
	s.mu.Unlock()

	if prev != value {
		s.log.Info("alarm status changed", "from", prev, "to", value, "profile", p.String())
	}
	return s.apply(p)
}

func (s *Subscriber) handlePmState(value string) error {
	s.mu.Lock()
	prev := s.pmState
	s.pmState = value
	p := profile.Derive(s.alarmStatus, s.pmState)
	s.mu.Unlock()

	if prev != value {
		s.log.Info("power-manager state changed", "from", prev, "to", value, "profile", p.String())
	}
	return s.apply(p)
}

func (s *Subscriber) handleVehicleState(value string) error {
	rate := SlowPollRateHz
	switch value {
	case "parked", "ready-to-drive":
		rate = FastPollRateHz
	}
	s.log.Info("vehicle state changed", "state", value, "sensor_rate_hz", rate)
	s.rate.SetRate(rate)
	return nil
}

func (s *Subscriber) apply(p profile.Profile) error {
	// Use a short timeout so a wedged chip-write doesn't stall the
	// HashWatcher dispatch loop. The controller's Apply is normally
	// well under a second.
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	if err := s.controller.Apply(ctx, p); err != nil {
		s.log.Error("profile apply failed", "profile", p.String(), "error", err)
		return err
	}
	return nil
}
