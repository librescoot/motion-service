package app

import (
	"context"
	"fmt"
	"log/slog"
	"strconv"
	"strings"
	"time"

	"bmx-service/internal/bmx"
	"bmx-service/internal/driver"
	"bmx-service/internal/poller"
	"bmx-service/internal/redis"
)

// Config holds application configuration
type Config struct {
	I2CBus      string
	RedisAddr   string
	PollingRate int
	Logger      *slog.Logger
}

// App represents the bmx-service application
type App struct {
	cfg       *Config
	log       *slog.Logger
	redis     *redis.Client
	publisher *redis.Publisher
	accel     *bmx.Accelerometer
	gyro      *bmx.Gyroscope
	mag       *bmx.Magnetometer

	sensorPoller    *poller.SensorPoller
	interruptPoller *poller.InterruptPoller
	magPoller       *poller.MagPoller
}

// New creates a new App
func New(cfg *Config) *App {
	return &App{
		cfg: cfg,
		log: cfg.Logger,
	}
}

// Run runs the application
func (a *App) Run(ctx context.Context) error {
	a.log.Info("starting bmx-service",
		"i2c_bus", a.cfg.I2CBus,
		"redis_addr", a.cfg.RedisAddr,
		"polling_rate", a.cfg.PollingRate)

	if err := a.unbindDrivers(); err != nil {
		return fmt.Errorf("unbind drivers: %w", err)
	}

	a.redis = redis.NewClient(a.cfg.RedisAddr, a.log)
	if err := a.redis.Connect(ctx); err != nil {
		return fmt.Errorf("connect to redis: %w", err)
	}
	defer a.redis.Close()

	a.publisher = redis.NewPublisher(a.redis, a.log)

	if err := a.initSensors(); err != nil {
		return fmt.Errorf("init sensors: %w", err)
	}
	defer a.closeSensors()

	if err := a.publishInitialStatus(ctx); err != nil {
		a.log.Warn("failed to publish initial status", "error", err)
	}

	a.sensorPoller = poller.NewSensorPoller(
		a.accel, a.gyro, a.mag, a.publisher, a.cfg.PollingRate, a.log)
	go a.sensorPoller.Run(ctx)

	a.interruptPoller = poller.NewInterruptPoller(
		a.accel, a.gyro, a.publisher, a.log)
	go a.interruptPoller.Run(ctx)

	if a.mag != nil {
		a.magPoller = poller.NewMagPoller(a.mag, a.accel, a.gyro, a.publisher, a.log)
		go a.magPoller.Run(ctx)
	}

	cmdHandler := redis.NewCommandHandler(a.redis, a.log, a.handleCommand)
	go cmdHandler.Run(ctx)

	<-ctx.Done()
	a.log.Info("shutting down")
	return nil
}

// unbindDrivers unbinds kernel drivers
func (a *App) unbindDrivers() error {
	a.log.Info("unbinding kernel drivers")

	if err := driver.UnbindBMX055(); err != nil {
		a.log.Warn("failed to unbind BMX055 drivers", "error", err)
	}

	time.Sleep(100 * time.Millisecond)
	return nil
}

// initSensors initializes all sensors
func (a *App) initSensors() error {
	var err error

	a.log.Info("initializing accelerometer")
	a.accel, err = bmx.NewAccelerometer(a.cfg.I2CBus)
	if err != nil {
		return fmt.Errorf("init accelerometer: %w", err)
	}

	a.log.Info("initializing gyroscope")
	a.gyro, err = bmx.NewGyroscope(a.cfg.I2CBus)
	if err != nil {
		return fmt.Errorf("init gyroscope: %w", err)
	}

	a.log.Info("calibrating gyroscope (keep scooter stationary)")
	if err := a.gyro.Calibrate(100); err != nil {
		a.log.Warn("failed to calibrate gyroscope", "error", err)
	} else {
		a.log.Info("gyroscope calibration complete")
	}

	a.log.Info("initializing magnetometer")
	a.mag, err = bmx.NewMagnetometer(a.cfg.I2CBus)
	if err != nil {
		a.log.Warn("magnetometer not available", "error", err)
		a.mag = nil
	}

	a.log.Info("all sensors initialized")
	return nil
}

// closeSensors closes all sensors
func (a *App) closeSensors() {
	if a.accel != nil {
		a.accel.Close()
	}
	if a.gyro != nil {
		a.gyro.Close()
	}
	if a.mag != nil {
		a.mag.Close()
	}
}

// publishInitialStatus publishes initial status to Redis
func (a *App) publishInitialStatus(ctx context.Context) error {
	status := map[string]string{
		"initialized":              "true",
		"polling-rate-hz":          fmt.Sprintf("%d", a.cfg.PollingRate),
		"streaming":                "disabled",
		"interrupt":                "disabled",
		"pin":                      "none",
		"threshold":                "0x00",
		"duration":                 "0x00",
		"sensitivity":              "none",
		"last-interrupt-timestamp": "0",
		"error-count":              "0",
		"last-error":               "",
	}

	return a.publisher.PublishStatus(ctx, status)
}

// handleCommand handles a command from Redis
func (a *App) handleCommand(action, param string) {
	ctx := context.Background()

	switch action {
	case "sensitivity":
		a.handleSetSensitivity(ctx, param)
	case "pin":
		a.handleSetInterruptPin(ctx, param)
	case "interrupt":
		a.handleInterruptToggle(ctx, param)
	case "reset":
		a.handleSoftReset(ctx)
	case "polling":
		a.handleSetPollingRate(ctx, param)
	case "streaming":
		a.handleStreamingToggle(ctx, param)
	default:
		a.log.Warn("unknown command", "command", action)
	}
}

// handleSetSensitivity handles the sensitivity command
func (a *App) handleSetSensitivity(ctx context.Context, level string) {
	sens := bmx.ParseSensitivity(level)
	threshold := sens.GetThreshold()
	duration := sens.GetDuration()

	if err := a.accel.ConfigureSlowNoMotion(threshold, duration); err != nil {
		a.log.Error("failed to configure slow/no-motion", "error", err)
		return
	}

	a.interruptPoller.SetConfig(threshold, duration, level)

	a.publisher.UpdateStatusField(ctx, "sensitivity", level)
	a.publisher.UpdateStatusField(ctx, "threshold", fmt.Sprintf("0x%02X", threshold))
	a.publisher.UpdateStatusField(ctx, "duration", fmt.Sprintf("0x%02X", duration))

	a.log.Info("sensitivity updated", "level", level, "threshold", threshold, "duration", duration)
}

// handleSetInterruptPin handles the pin command
func (a *App) handleSetInterruptPin(ctx context.Context, pinStr string) {
	pin := bmx.ParseInterruptPin(pinStr)

	if pin == bmx.InterruptPinNone {
		if err := a.accel.DisableInterruptMapping(); err != nil {
			a.log.Error("failed to disable interrupt mapping", "error", err)
			return
		}
	} else {
		useInt2 := pin == bmx.InterruptPinINT2
		if err := a.accel.ConfigureInterruptPin(useInt2, true); err != nil {
			a.log.Error("failed to configure interrupt pin", "error", err)
			return
		}
		if err := a.accel.MapInterruptToPin(useInt2); err != nil {
			a.log.Error("failed to map interrupt to pin", "error", err)
			return
		}
	}

	a.publisher.UpdateStatusField(ctx, "pin", pinStr)
	a.log.Info("interrupt pin updated", "pin", pinStr)
}

// handleInterruptToggle handles the interrupt enable/disable command
func (a *App) handleInterruptToggle(ctx context.Context, state string) {
	if state == "enable" {
		if err := a.accel.EnableSlowNoMotionInterrupt(true); err != nil {
			a.log.Error("failed to enable interrupt", "error", err)
			return
		}
		a.interruptPoller.Enable()
		a.publisher.UpdateStatusField(ctx, "interrupt", "enabled")
		a.log.Info("interrupt enabled")
	} else if state == "disable" {
		if err := a.accel.DisableSlowNoMotionInterrupt(); err != nil {
			a.log.Error("failed to disable interrupt", "error", err)
			return
		}
		a.interruptPoller.Disable()
		a.publisher.UpdateStatusField(ctx, "interrupt", "disabled")
		a.log.Info("interrupt disabled")
	}
}

// handleSoftReset handles the reset command
func (a *App) handleSoftReset(ctx context.Context) {
	a.log.Info("performing soft reset")

	if err := a.accel.SoftReset(); err != nil {
		a.log.Error("failed to reset accelerometer", "error", err)
	}

	if err := a.gyro.SoftReset(); err != nil {
		a.log.Error("failed to reset gyroscope", "error", err)
	}

	time.Sleep(10 * time.Millisecond)

	a.publisher.UpdateStatusField(ctx, "interrupt", "disabled")
	a.publisher.UpdateStatusField(ctx, "sensitivity", "none")
	a.log.Info("soft reset complete")
}

// handleSetPollingRate handles the polling rate command
func (a *App) handleSetPollingRate(ctx context.Context, rateStr string) {
	rate, err := strconv.Atoi(strings.TrimSpace(rateStr))
	if err != nil || rate < 1 || rate > 100 {
		a.log.Error("invalid polling rate", "rate", rateStr)
		return
	}

	a.sensorPoller.SetRate(rate)
	a.publisher.UpdateStatusField(ctx, "polling-rate-hz", fmt.Sprintf("%d", rate))
	a.log.Info("polling rate updated", "rate_hz", rate)
}

// handleStreamingToggle handles the streaming enable/disable command
func (a *App) handleStreamingToggle(ctx context.Context, state string) {
	if state == "enable" {
		a.sensorPoller.Enable()
		a.publisher.UpdateStatusField(ctx, "streaming", "enabled")
		a.log.Info("sensor streaming enabled")
	} else if state == "disable" {
		a.sensorPoller.Disable()
		a.publisher.UpdateStatusField(ctx, "streaming", "disabled")
		a.log.Info("sensor streaming disabled")
	}
}