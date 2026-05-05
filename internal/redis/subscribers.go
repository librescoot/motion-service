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

// Subscriber watches the `alarm` and `power-manager` hashes and reactively
// reconfigures the BMX055 by deriving the appropriate profile from the
// (alarm.status, power-manager.state) pair. Uses redis-ipc HashWatcher
// with a 50 ms debounce so rapid status changes don't thrash the chip.
type Subscriber struct {
	ipcClient  *ipc.Client
	controller ProfileApplier
	log        *slog.Logger

	alarmW *ipc.HashWatcher
	pmW    *ipc.HashWatcher

	mu          sync.Mutex
	alarmStatus string
	pmState     string
}

// NewSubscriber returns a Subscriber bound to the given redis-ipc client
// and profile controller.
func NewSubscriber(ipcClient *ipc.Client, controller ProfileApplier, log *slog.Logger) *Subscriber {
	return &Subscriber{
		ipcClient:  ipcClient,
		controller: controller,
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

	s.log.Info("alarm + power-manager state watchers started")
	return nil
}

// Stop tears down both watchers.
func (s *Subscriber) Stop() {
	if s.alarmW != nil {
		s.alarmW.Stop()
	}
	if s.pmW != nil {
		s.pmW.Stop()
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
