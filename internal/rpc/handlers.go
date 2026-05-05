package rpc

import (
	"context"
	"fmt"
	"log/slog"

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

// Server bundles the dependencies the handlers need and owns the
// CallServer that dispatches requests by method.
type Server struct {
	controller *profile.Controller
	accel      *bmx.Accelerometer
	gyro      *bmx.Gyroscope
	log        *slog.Logger

	srv *ipc.CallServer
}

// New returns a Server. Call Start to register the handlers and begin
// processing, Stop to drain.
func New(ipcClient *ipc.Client, controller *profile.Controller, accel *bmx.Accelerometer, gyro *bmx.Gyroscope, log *slog.Logger) *Server {
	s := &Server{
		controller: controller,
		accel:      accel,
		gyro:       gyro,
		log:        log,
	}
	s.srv = ipc.NewCallServer(ipcClient, Channel)
	ipc.RegisterCall[PrepareHibernationReq, PrepareHibernationResp](s.srv, MethodPrepareHibernation, s.prepareHibernation)
	ipc.RegisterCall[GetCalibrationReq, CalibrationResp](s.srv, MethodGetCalibration, s.getCalibration)
	ipc.RegisterCall[EmptyReq, EmptyResp](s.srv, MethodClearLatch, s.clearLatch)
	ipc.RegisterCall[EmptyReq, EmptyResp](s.srv, MethodSoftReset, s.softReset)
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

// softReset performs a soft reset on accel + gyro. The currently-applied
// profile is intentionally NOT re-applied — caller is responsible for
// triggering a re-apply (typically by writing to the alarm hash).
func (s *Server) softReset(_ EmptyReq) (EmptyResp, error) {
	if err := s.accel.SoftReset(); err != nil {
		return EmptyResp{}, fmt.Errorf("accel: %w", err)
	}
	if err := s.gyro.SoftReset(); err != nil {
		return EmptyResp{}, fmt.Errorf("gyro: %w", err)
	}
	return EmptyResp{OK: true}, nil
}
