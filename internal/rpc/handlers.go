package rpc

import (
	"context"
	"fmt"
	"log/slog"
	"time"

	ipc "github.com/librescoot/redis-ipc"

	"github.com/librescoot/motion-service/internal/bmx"
	"github.com/librescoot/motion-service/internal/calibration"
	"github.com/librescoot/motion-service/internal/profile"
)

// Channel is the public request queue; method names below are part of its IPC contract.
const Channel = "motion:rpc"

const (
	MethodPrepareHibernation = "prepare-hibernation"
	MethodGetCalibration     = "get-calibration"
	MethodClearLatch         = "clear-latch"
	MethodSoftReset          = "soft-reset"
	MethodSetPolling         = "set-polling"
	MethodSetStreaming       = "set-streaming"
	MethodCalibrationStart   = "calibration-start"
	MethodCalibrationStatus  = "calibration-status"
	MethodCalibrationFinish  = "calibration-finish"
	MethodCalibrationCancel  = "calibration-cancel"
	MethodCalibrationReset   = "calibration-reset"
)

// PrepareHibernationReq explicitly names the requested profile for protocol evolution.
type PrepareHibernationReq struct {
	Profile string `json:"profile"`
}

type PrepareHibernationResp struct {
	Programmed bool   `json:"programmed"`
	Profile    string `json:"profile"`
}

type GetCalibrationReq struct{}

type CalibrationResp struct {
	HardIronOffset [3]float64    `json:"hard_iron_offset"`
	SoftIronXY     [2][2]float64 `json:"soft_iron_xy"`
	AxisOrder      [3]int        `json:"axis_order"`
	AxisSign       [3]float64    `json:"axis_sign"`
	YawOffsetDeg   float64       `json:"yaw_offset_deg"`
	State          string        `json:"state"`
}

type EmptyReq struct{}
type CalibrationStatusResp struct {
	calibration.Status
}

type EmptyResp struct {
	OK bool `json:"ok"`
}

// SetPollingReq is a temporary override; vehicle-state changes re-derive the rate.
type SetPollingReq struct {
	RateHz int `json:"rate_hz"`
}

type SetStreamingReq struct {
	Enabled bool `json:"enabled"`
}

type RateSetter interface {
	SetRate(rateHz int)
}

type Streamer interface {
	Enable()
	Disable()
}

type Server struct {
	controller  *profile.Controller
	accel       *bmx.Accelerometer
	gyro        *bmx.Gyroscope
	mag         *bmx.Magnetometer
	rate        RateSetter
	stream      Streamer
	calibration *calibration.Collector
	publisher   StatusWriter
	log         *slog.Logger

	srv *ipc.CallServer
}

type StatusWriter interface {
	UpdateStatusField(ctx context.Context, field, value string) error
}

func New(ipcClient *ipc.Client, controller *profile.Controller, accel *bmx.Accelerometer, gyro *bmx.Gyroscope, mag *bmx.Magnetometer, rate RateSetter, stream Streamer, collector *calibration.Collector, publisher StatusWriter, log *slog.Logger) *Server {
	s := &Server{
		controller:  controller,
		accel:       accel,
		gyro:        gyro,
		mag:         mag,
		rate:        rate,
		stream:      stream,
		calibration: collector,
		publisher:   publisher,
		log:         log,
	}
	s.srv = ipc.NewCallServer(ipcClient, Channel)
	ipc.RegisterCall[PrepareHibernationReq, PrepareHibernationResp](s.srv, MethodPrepareHibernation, s.prepareHibernation)
	ipc.RegisterCall[GetCalibrationReq, CalibrationResp](s.srv, MethodGetCalibration, s.getCalibration)
	ipc.RegisterCall[EmptyReq, EmptyResp](s.srv, MethodClearLatch, s.clearLatch)
	ipc.RegisterCall[EmptyReq, EmptyResp](s.srv, MethodSoftReset, s.softReset)
	ipc.RegisterCall[SetPollingReq, EmptyResp](s.srv, MethodSetPolling, s.setPolling)
	ipc.RegisterCall[SetStreamingReq, EmptyResp](s.srv, MethodSetStreaming, s.setStreaming)
	ipc.RegisterCall[EmptyReq, CalibrationStatusResp](s.srv, MethodCalibrationStart, s.calibrationStart)
	ipc.RegisterCall[EmptyReq, CalibrationStatusResp](s.srv, MethodCalibrationStatus, s.calibrationStatus)
	ipc.RegisterCall[EmptyReq, CalibrationStatusResp](s.srv, MethodCalibrationFinish, s.calibrationFinish)
	ipc.RegisterCall[EmptyReq, CalibrationStatusResp](s.srv, MethodCalibrationCancel, s.calibrationCancel)
	ipc.RegisterCall[EmptyReq, CalibrationStatusResp](s.srv, MethodCalibrationReset, s.calibrationReset)
	return s
}

func (s *Server) Start() {
	s.srv.Start()
}

func (s *Server) Stop() {
	s.srv.Stop()
}

// prepareHibernation returns only after the suspend-safe profile is in registers.
func (s *Server) prepareHibernation(req PrepareHibernationReq) (PrepareHibernationResp, error) {
	want := req.Profile
	if want == "" {
		want = profile.ArmedHibernation.String()
	}
	if want != profile.ArmedHibernation.String() {
		return PrepareHibernationResp{}, fmt.Errorf("unsupported profile for hibernation: %q", want)
	}

	ctx, cancel := context.WithTimeout(context.Background(), 1500*1000*1000)
	defer cancel()
	if err := s.controller.Apply(ctx, profile.ArmedHibernation); err != nil {
		return PrepareHibernationResp{}, fmt.Errorf("apply armed-hibernation: %w", err)
	}
	return PrepareHibernationResp{Programmed: true, Profile: profile.ArmedHibernation.String()}, nil
}

func (s *Server) getCalibration(_ GetCalibrationReq) (CalibrationResp, error) {
	if s.mag == nil {
		return CalibrationResp{}, fmt.Errorf("magnetometer unavailable")
	}
	cal := s.mag.Calibration()
	return CalibrationResp{
		HardIronOffset: cal.HardIronOffset,
		SoftIronXY:     cal.SoftIronXY,
		AxisOrder:      cal.Orientation.AxisOrder,
		AxisSign:       cal.Orientation.AxisSign,
		YawOffsetDeg:   cal.YawOffsetDeg,
		State:          cal.State,
	}, nil
}

func (s *Server) clearLatch(_ EmptyReq) (EmptyResp, error) {
	if err := s.accel.ClearLatchedInterrupt(); err != nil {
		return EmptyResp{}, err
	}
	return EmptyResp{OK: true}, nil
}

// softReset must invalidate then reapply the current profile: reset hardware is
// unsafe while armed because it has no motion engine configured.
func (s *Server) softReset(_ EmptyReq) (EmptyResp, error) {
	if err := s.accel.SoftReset(); err != nil {
		return EmptyResp{}, fmt.Errorf("accel: %w", err)
	}
	if err := s.gyro.SoftReset(); err != nil {
		return EmptyResp{}, fmt.Errorf("gyro: %w", err)
	}

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

func (s *Server) setPolling(req SetPollingReq) (EmptyResp, error) {
	if req.RateHz < 1 || req.RateHz > 100 {
		return EmptyResp{}, fmt.Errorf("rate_hz out of range: %d (want 1..100)", req.RateHz)
	}
	s.rate.SetRate(req.RateHz)
	_ = s.publisher.UpdateStatusField(context.Background(), "polling-rate-hz", fmt.Sprintf("%d", req.RateHz))
	s.log.Info("polling rate overridden via rpc", "rate_hz", req.RateHz)
	return EmptyResp{OK: true}, nil
}

func (s *Server) calibrationStart(_ EmptyReq) (CalibrationStatusResp, error) {
	if s.calibration == nil {
		return CalibrationStatusResp{}, fmt.Errorf("magnetometer calibration unavailable")
	}
	return CalibrationStatusResp{Status: s.calibration.Start()}, nil
}

func (s *Server) calibrationStatus(_ EmptyReq) (CalibrationStatusResp, error) {
	if s.calibration == nil {
		return CalibrationStatusResp{}, fmt.Errorf("magnetometer calibration unavailable")
	}
	return CalibrationStatusResp{Status: s.calibration.Status()}, nil
}

func (s *Server) calibrationFinish(_ EmptyReq) (CalibrationStatusResp, error) {
	if s.calibration == nil {
		return CalibrationStatusResp{}, fmt.Errorf("magnetometer calibration unavailable")
	}
	status, err := s.calibration.Finish()
	return CalibrationStatusResp{Status: status}, err
}

func (s *Server) calibrationCancel(_ EmptyReq) (CalibrationStatusResp, error) {
	if s.calibration == nil {
		return CalibrationStatusResp{}, fmt.Errorf("magnetometer calibration unavailable")
	}
	return CalibrationStatusResp{Status: s.calibration.Cancel()}, nil
}

func (s *Server) calibrationReset(_ EmptyReq) (CalibrationStatusResp, error) {
	if s.calibration == nil {
		return CalibrationStatusResp{}, fmt.Errorf("magnetometer calibration unavailable")
	}
	status, err := s.calibration.Reset()
	return CalibrationStatusResp{Status: status}, err
}

// setStreaming controls sensor telemetry only; interrupt wake handling remains live.
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
