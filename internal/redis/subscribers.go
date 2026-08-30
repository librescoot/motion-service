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

type ProfileApplier interface {
	Apply(ctx context.Context, p profile.Profile) error
}

type RateSetter interface {
	SetRate(rateHz int)
}

// Only parked and driving states need the live sensor stream; alarm wake uses
// chip interrupts, so all other states use the low-rate heartbeat.
const (
	FastPollRateHz = 5
	SlowPollRateHz = 1
)

type Subscriber struct {
	ipcClient  *ipc.Client
	controller ProfileApplier
	rate       RateSetter
	log        *slog.Logger

	alarmW   *ipc.HashWatcher
	pmW      *ipc.HashWatcher
	vehicleW *ipc.HashWatcher

	mu          sync.Mutex
	alarmStatus string
	pmState     string
}

func NewSubscriber(ipcClient *ipc.Client, controller ProfileApplier, rate RateSetter, log *slog.Logger) *Subscriber {
	return &Subscriber{
		ipcClient:  ipcClient,
		controller: controller,
		rate:       rate,
		log:        log,
	}
}

// Start uses a 50 ms debounce so rapid alarm/power updates do not thrash chip
// profiles; StartWithSync ensures the first profile reflects existing state.
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
		if stopErr := s.alarmW.Stop(); stopErr != nil {
			s.log.Warn("failed to stop alarm watcher during rollback", "error", stopErr)
		}
		return fmt.Errorf("start power-manager watcher: %w", err)
	}

	if s.rate != nil {
		s.vehicleW = s.ipcClient.NewHashWatcher("vehicle")
		s.vehicleW.SetDebounce(50 * time.Millisecond)
		s.vehicleW.OnField("state", s.handleVehicleState)
		if err := s.vehicleW.StartWithSync(); err != nil {
			if stopErr := s.alarmW.Stop(); stopErr != nil {
				s.log.Warn("failed to stop alarm watcher during rollback", "error", stopErr)
			}
			if stopErr := s.pmW.Stop(); stopErr != nil {
				s.log.Warn("failed to stop power-manager watcher during rollback", "error", stopErr)
			}
			return fmt.Errorf("start vehicle watcher: %w", err)
		}
	}

	s.log.Info("alarm + power-manager + vehicle state watchers started")
	return nil
}

func (s *Subscriber) Stop() {
	if s.alarmW != nil {
		if err := s.alarmW.Stop(); err != nil {
			s.log.Warn("failed to stop alarm watcher", "error", err)
		}
	}
	if s.pmW != nil {
		if err := s.pmW.Stop(); err != nil {
			s.log.Warn("failed to stop power-manager watcher", "error", err)
		}
	}
	if s.vehicleW != nil {
		if err := s.vehicleW.Stop(); err != nil {
			s.log.Warn("failed to stop vehicle watcher", "error", err)
		}
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

// A wedged hardware write must not block HashWatcher dispatch indefinitely.
func (s *Subscriber) apply(p profile.Profile) error {

	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	if err := s.controller.Apply(ctx, p); err != nil {
		s.log.Error("profile apply failed", "profile", p.String(), "error", err)
		return err
	}
	return nil
}
