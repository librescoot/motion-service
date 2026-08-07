package rpc

import (
	"context"
	"fmt"
	"log/slog"
	"time"

	ipc "github.com/librescoot/redis-ipc"

	"github.com/librescoot/motion-service/internal/bmx"
	"github.com/librescoot/motion-service/internal/profile"
)

// Channel is the per-service RPC request channel. Method dispatch happens
// inside the CallServer based on the envelope's method field.
const Channel = "motion:rpc"

// Method names for the RPCs motion-service exposes.
const (
	MethodPrepareHibernation = "prepare-hibernation"
	MethodGetCalibration     = "get-calibration"
	MethodClearLatch         = "clear-latch"
	MethodSoftReset          = "soft-reset"
	MethodSetPolling         = "set-polling"
	MethodSetStreaming       = "set-streaming"
)

// PrepareHibernationReq names the profile alarm-service wants confirmed
// before pm-service is allowed to suspend. In practice this is always
// "armed-hibernation" today, but the field is explicit so the protocol can
// evolve without an RPC-shape change.
type PrepareHibernationReq struct {
	Profile string `json:"profile"`
}

// PrepareHibernationResp confirms the requested profile is now in registers.
type PrepareHibernationResp struct {
	Programmed bool   `json:"programmed"`
	Profile    string `json:"profile"`
}

// GetCalibrationReq is empty — the response carries the data.
type GetCalibrationReq struct{}

// CalibrationResp returns hard-iron offsets, axis remap, and yaw offset
// applied to the magnetometer pipeline. Useful for diagnostics + a
// sanity-check companion to motion-calibrate.
type CalibrationResp struct {
	HardIronOffset [3]int16   `json:"hard_iron_offset"`
	AxisOrder      [3]int     `json:"axis_order"`
	AxisSign       [3]float64 `json:"axis_sign"`
	YawOffsetDeg   float64    `json:"yaw_offset_deg"`
}

// EmptyReq / EmptyResp for ping-style RPC methods.
type EmptyReq struct{}
type EmptyResp struct {
	OK bool `json:"ok"`
}

// SetPollingReq sets the telemetry poll rate in Hz. Note this is an
// override, not a setting: the vehicle-state watcher re-derives the rate
// on every vehicle.state change, so a manual rate lasts until the scooter
// next changes state.
type SetPollingReq struct {
	RateHz int `json:"rate_hz"`
}

// SetStreamingReq enables or disables the sensor telemetry stream.
type SetStreamingReq struct {
	Enabled bool `json:"enabled"`
}

// RateSetter is the subset of the pollers set-polling drives. Both the
// sensor and magnetometer pollers implement it and are driven in unison,
// matching what the vehicle-state watcher does.
type RateSetter interface {
	SetRate(rateHz int)
}

// Streamer is the subset of the sensor poller set-streaming drives. Only
// the sensor poller can be gated; the magnetometer poller has no such
// switch, so set-streaming affects the sensor stream alone.
type Streamer interface {
	Enable()
	Disable()
}

// Server bundles the dependencies the handlers need and owns the
// CallServer that dispatches requests by method.
type Server struct {
	controller *profile.Controller
	accel      *bmx.Accelerometer
	gyro       *bmx.Gyroscope
	rate       RateSetter
	stream     Streamer
	publisher  StatusWriter
	log        *slog.Logger

	srv *ipc.CallServer
}

// StatusWriter is the subset of the publisher these handlers need to
// reflect a change back into the motion hash.
type StatusWriter interface {
	UpdateStatusField(ctx context.Context, field, value string) error
}

// New returns a Server. Call Start to register the handlers and begin
// processing, Stop to drain.
func New(ipcClient *ipc.Client, controller *profile.Controller, accel *bmx.Accelerometer, gyro *bmx.Gyroscope, rate RateSetter, stream Streamer, publisher StatusWriter, log *slog.Logger) *Server {
	s := &Server{
		controller: controller,
		accel:      accel,
		gyro:       gyro,
		rate:       rate,
		stream:     stream,
		publisher:  publisher,
		log:        log,
	}
	s.srv = ipc.NewCallServer(ipcClient, Channel)
	ipc.RegisterCall[PrepareHibernationReq, PrepareHibernationResp](s.srv, MethodPrepareHibernation, s.prepareHibernation)
	ipc.RegisterCall[GetCalibrationReq, CalibrationResp](s.srv, MethodGetCalibration, s.getCalibration)
	ipc.RegisterCall[EmptyReq, EmptyResp](s.srv, MethodClearLatch, s.clearLatch)
	ipc.RegisterCall[EmptyReq, EmptyResp](s.srv, MethodSoftReset, s.softReset)
	ipc.RegisterCall[SetPollingReq, EmptyResp](s.srv, MethodSetPolling, s.setPolling)
	ipc.RegisterCall[SetStreamingReq, EmptyResp](s.srv, MethodSetStreaming, s.setStreaming)
	return s
}

// Start begins dispatching. One BRPOP loop on the per-service channel
// regardless of method count.
func (s *Server) Start() {
	s.srv.Start()
}

// Stop drains in-flight handlers and stops accepting new requests.
func (s *Server) Stop() {
	s.srv.Stop()
}

// prepareHibernation synchronously applies the armed-hibernation profile
// and returns once the registers are programmed. alarm-service Calls this
// before releasing pm-service's suspend inhibitor.
func (s *Server) prepareHibernation(req PrepareHibernationReq) (PrepareHibernationResp, error) {
	want := req.Profile
	if want == "" {
		want = profile.ArmedHibernation.String()
	}
	if want != profile.ArmedHibernation.String() {
		return PrepareHibernationResp{}, fmt.Errorf("unsupported profile for hibernation: %q", want)
	}

	ctx, cancel := context.WithTimeout(context.Background(), 1500*1000*1000) // 1.5 s
	defer cancel()
	if err := s.controller.Apply(ctx, profile.ArmedHibernation); err != nil {
		return PrepareHibernationResp{}, fmt.Errorf("apply armed-hibernation: %w", err)
	}
	return PrepareHibernationResp{Programmed: true, Profile: profile.ArmedHibernation.String()}, nil
}

// getCalibration reports the magnetometer calibration applied by the
// motion service.
func (s *Server) getCalibration(_ GetCalibrationReq) (CalibrationResp, error) {
	cal := bmx.DefaultCalibration
	return CalibrationResp{
		HardIronOffset: cal.HardIronOffset,
		AxisOrder:      cal.Orientation.AxisOrder,
		AxisSign:       cal.Orientation.AxisSign,
		YawOffsetDeg:   cal.YawOffsetDeg,
	}, nil
}

// clearLatch clears the BMX055 latched interrupt. Useful for support if
// the chip is stuck in an asserted state for some reason.
func (s *Server) clearLatch(_ EmptyReq) (EmptyResp, error) {
	if err := s.accel.ClearLatchedInterrupt(); err != nil {
		return EmptyResp{}, err
	}
	return EmptyResp{OK: true}, nil
}

// softReset performs a soft reset on accel + gyro and then reprograms the
// currently-applied profile, so the chip ends up back in a known state
// rather than at register defaults.
//
// Leaving it reset is not a safe option: a soft reset wipes the motion
// engine, and on an armed scooter that means no motion detection. The
// controller caches the applied profile and skips a re-apply of the same
// one, so the reset has to invalidate that cache or the chip would stay
// dead until the profile happened to change.
func (s *Server) softReset(_ EmptyReq) (EmptyResp, error) {
	if err := s.accel.SoftReset(); err != nil {
		return EmptyResp{}, fmt.Errorf("accel: %w", err)
	}
	if err := s.gyro.SoftReset(); err != nil {
		return EmptyResp{}, fmt.Errorf("gyro: %w", err)
	}

	// The registers no longer match what the controller thinks it wrote.
	s.controller.Invalidate()

	current := s.controller.Current()
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	if err := s.controller.Apply(ctx, current); err != nil {
		return EmptyResp{}, fmt.Errorf("reapply %s after reset: %w", current.String(), err)
	}
	s.log.Info("soft reset complete, profile reprogrammed", "profile", current.String())
	return EmptyResp{OK: true}, nil
}

// setPolling overrides the telemetry poll rate. Replaces the old
// "LPUSH scooter:motion polling:N" command, with a reply so the caller
// learns about a rejected rate instead of only finding it in the log.
func (s *Server) setPolling(req SetPollingReq) (EmptyResp, error) {
	if req.RateHz < 1 || req.RateHz > 100 {
		return EmptyResp{}, fmt.Errorf("rate_hz out of range: %d (want 1..100)", req.RateHz)
	}
	s.rate.SetRate(req.RateHz)
	_ = s.publisher.UpdateStatusField(context.Background(), "polling-rate-hz", fmt.Sprintf("%d", req.RateHz))
	s.log.Info("polling rate overridden via rpc", "rate_hz", req.RateHz)
	return EmptyResp{OK: true}, nil
}

// setStreaming enables or disables the sensor telemetry stream. Replaces
// the old "LPUSH scooter:motion streaming:enable|disable" command.
func (s *Server) setStreaming(req SetStreamingReq) (EmptyResp, error) {
	state := "disabled"
	if req.Enabled {
		s.stream.Enable()
		state = "enabled"
	} else {
		s.stream.Disable()
	}
	_ = s.publisher.UpdateStatusField(context.Background(), "streaming", state)
	s.log.Info("sensor streaming set via rpc", "state", state)
	return EmptyResp{OK: true}, nil
}
