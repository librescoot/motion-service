package app

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	ipc "github.com/librescoot/redis-ipc"

	"github.com/librescoot/motion-service/internal/bmx"
	"github.com/librescoot/motion-service/internal/calibration"
	"github.com/librescoot/motion-service/internal/driver"
	"github.com/librescoot/motion-service/internal/poller"
	"github.com/librescoot/motion-service/internal/profile"
	"github.com/librescoot/motion-service/internal/redis"
	rpcpkg "github.com/librescoot/motion-service/internal/rpc"
)

// Config holds application configuration
type Config struct {
	I2CBus       string
	RedisAddr    string
	PollingRate  int
	EvdevDevice  string
	EvdevKeycode uint16
	Logger       *slog.Logger
}

// App represents the motion-service application
type App struct {
	cfg                  *Config
	log                  *slog.Logger
	ipcClient            *ipc.Client
	publisher            *redis.Publisher
	accel                *bmx.Accelerometer
	gyro                 *bmx.Gyroscope
	mag                  *bmx.Magnetometer
	calibrationCollector *calibration.Collector

	sensorPoller     *poller.SensorPoller
	interruptPoller  *poller.InterruptPoller
	interruptWatcher *poller.InterruptWatcher
	magPoller        *poller.MagPoller

	controller *profile.Controller
	subscriber *redis.Subscriber
	rpcServer  *rpcpkg.Server
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
	a.log.Info("starting motion-service",
		"i2c_bus", a.cfg.I2CBus,
		"redis_addr", a.cfg.RedisAddr,
		"polling_rate", a.cfg.PollingRate)

	if err := a.unbindDrivers(); err != nil {
		return fmt.Errorf("unbind drivers: %w", err)
	}

	// One redis-ipc client for everything: publisher, hash watchers and
	// the RPC server. redis-ipc multiplexes all subscriptions onto a
	// single pub/sub connection and all queues onto a single BRPOP, so
	// the default pool is enough.
	host, port := splitHostPort(a.cfg.RedisAddr)
	ipcClient, err := ipc.New(
		ipc.WithAddress(host),
		ipc.WithPort(port),
		ipc.WithLogger(a.log),
	)
	if err != nil {
		return fmt.Errorf("connect to redis: %w", err)
	}
	a.ipcClient = ipcClient
	defer ipcClient.Close()

	a.publisher = redis.NewPublisher(a.ipcClient, a.log)
	a.publisher.PruneLegacyFields(ctx)

	if err := a.initSensors(); err != nil {
		return fmt.Errorf("init sensors: %w", err)
	}
	defer a.closeSensors()

	if a.mag != nil {
		modelPath := filepath.Join("/data", calibration.ModelFilename)
		model, err := calibration.LoadModel(modelPath)
		switch {
		case err == nil:
			a.applyMagCalibration(&model)
			a.log.Info("loaded planar magnetometer calibration", "path", modelPath,
				"residual_rms", model.ResidualRMS, "condition", model.Condition)
		case errors.Is(err, os.ErrNotExist):
			a.applyMagCalibration(nil)
			a.log.Warn("no per-vehicle magnetometer calibration; heading disabled")
		default:
			a.applyMagCalibration(nil)
			a.log.Error("invalid magnetometer calibration; heading disabled",
				"path", modelPath, "error", err)
		}
	}

	if err := a.publishInitialStatus(ctx); err != nil {
		a.log.Warn("failed to publish initial status", "error", err)
	}

	// Sensor + magnetometer telemetry pollers run unconditionally —
	// they're independent of the alarm-driven interrupt path. The shared
	// cache lets mag_poller reuse sensor_poller's accel + gyro reads
	// instead of duplicating ~12 I2C transactions per cycle.
	sensorCache := bmx.NewSensorCache()
	a.sensorPoller = poller.NewSensorPoller(
		a.accel, a.gyro, a.mag, a.publisher, sensorCache, a.cfg.PollingRate, a.log)
	go a.sensorPoller.Run(ctx)

	if a.mag != nil {
		a.calibrationCollector = calibration.NewCollector("/data", a.applyMagCalibration)
		// Match mag_poller's initial rate to sensor_poller's so both
		// pollers come up at the same cadence; the subscriber will
		// re-set both in unison when vehicle:state arrives.
		a.magPoller = poller.NewMagPoller(a.mag, a.accel, a.gyro, a.publisher,
			sensorCache, a.calibrationCollector, a.cfg.PollingRate, a.log)
		go a.magPoller.Run(ctx)
	}

	// Interrupt poller (I2C-status watchdog) and watcher (evdev fast path).
	// Both start disabled; the profile controller arms them when a profile
	// that uses interrupts is applied.
	a.interruptPoller = poller.NewInterruptPoller(a.accel, a.publisher, a.log)
	go a.interruptPoller.Run(ctx)

	var watcherSource profile.InterruptSource
	if a.cfg.EvdevDevice != "" {
		w := poller.NewInterruptWatcher(a.cfg.EvdevDevice, a.cfg.EvdevKeycode, a.accel, a.publisher, a.log)
		if err := w.Open(); err != nil {
			a.log.Warn("evdev watcher unavailable, falling back to I2C poller only", "error", err)
		} else {
			a.interruptWatcher = w
			watcherSource = w
			go w.Run(ctx)
		}
	}

	// Wake-cause detection: if the chip already has a latched interrupt at
	// startup, we likely came up from hibernation due to a motion edge.
	// Read INT_STATUS_0 BEFORE the controller's first Apply (which soft-
	// resets and clears the latch).
	wakeFromHibernation := false
	if status, err := a.accel.ReadByteData(bmx.ACCEL_INT_STATUS_0); err == nil {
		if (status & (bmx.ACCEL_INT_STATUS_SLOPE | bmx.ACCEL_INT_STATUS_SLOW_NO_MOT)) != 0 {
			wakeFromHibernation = true
			a.log.Info("startup latched interrupt detected — wake-from-hibernation",
				"int_status_0", fmt.Sprintf("0x%02X", status))
		}
	}

	// Profile controller — owns chip configuration. Apply Idle so the chip
	// is in a known state before subscribers can drive it.
	a.controller = profile.New(a.accel, a.gyro, a.interruptPoller, watcherSource, a.publisher, a.log)
	if err := a.controller.Apply(ctx, profile.Idle); err != nil {
		return fmt.Errorf("apply initial idle profile: %w", err)
	}

	// If we woke from a hibernation motion, emit the sentinel event so
	// alarm-service can branch its FSM on it. Doing this AFTER the chip
	// has been put into Idle state so consumers see a chip in a known
	// configuration when they react.
	if wakeFromHibernation {
		if err := a.publisher.PublishMotionEvent(ctx, &redis.MotionEvent{
			Type:      "wake-hibernation",
			Timestamp: time.Now().UnixMilli(),
		}); err != nil {
			a.log.Warn("publish wake-hibernation failed", "error", err)
		}
	}

	// Subscribe to the alarm hash + power-manager hash. StartWithSync issues
	// HGETALL on each so the very first apply reflects current vehicle state.
	a.subscriber = redis.NewSubscriber(a.ipcClient, a.controller, pollerGroup{a.sensorPoller, a.magPoller}, a.log)
	if err := a.subscriber.Start(); err != nil {
		return fmt.Errorf("start subscribers: %w", err)
	}
	defer a.subscriber.Stop()

	// Register RPC handlers (prepare-hibernation, get-calibration, ...)
	a.rpcServer = rpcpkg.New(a.ipcClient, a.controller, a.accel, a.gyro, a.mag,
		pollerGroup{a.sensorPoller, a.magPoller}, a.sensorPoller,
		a.calibrationCollector, a.publisher, a.log)
	a.rpcServer.Start()
	defer a.rpcServer.Stop()

	if err := a.publisher.PublishReady(ctx); err != nil {
		a.log.Warn("publish ready failed", "error", err)
	}

	<-ctx.Done()
	a.log.Info("shutting down")
	return nil
}

func (a *App) applyMagCalibration(model *calibration.PlanarModel) {
	if a.mag == nil {
		return
	}
	cal := bmx.DefaultCalibration
	if model != nil {
		cal.HardIronOffset[0] = model.Offset[0]
		cal.HardIronOffset[1] = model.Offset[1]
		cal.SoftIronXY = model.Matrix
		cal.State = "calibrated"
	}
	a.mag.SetCalibration(cal)
}

// splitHostPort splits "host:port" with a sensible default port if absent.
func splitHostPort(addr string) (string, int) {
	if i := strings.LastIndex(addr, ":"); i >= 0 {
		host := addr[:i]
		if port, err := strconv.Atoi(addr[i+1:]); err == nil {
			return host, port
		}
		return host, 6379
	}
	return addr, 6379
}

// unbindDrivers unbinds kernel drivers. driver.Unbind now blocks until the
// kernel has actually released each device, so no extra settle sleep is
// needed afterwards.
func (a *App) unbindDrivers() error {
	a.log.Info("unbinding kernel drivers")

	if err := driver.UnbindBMX055(); err != nil {
		a.log.Warn("failed to unbind BMX055 drivers", "error", err)
	}
	return nil
}

// sensorInitAttempts and sensorInitBackoff bound the retry-with-backoff
// applied to the first chip-ID read for each BMX055 sensor. The kernel can
// briefly fail the i2c transaction immediately after unbind even though the
// unbind sysfs path reports the device is gone; retrying handles that
// transient ENXIO without crashing the service.
const (
	sensorInitAttempts = 5
	sensorInitBackoff  = 200 * time.Millisecond
)

// initWithRetry runs the sensor constructor up to sensorInitAttempts times,
// backing off between attempts. The first attempt's failure is the common
// case on cold boot right after unbinding the kernel driver.
func initWithRetry[T any](sensor string, log *slog.Logger, ctor func() (*T, error)) (*T, error) {
	var lastErr error
	for i := 0; i < sensorInitAttempts; i++ {
		v, err := ctor()
		if err == nil {
			return v, nil
		}
		lastErr = err
		if i+1 < sensorInitAttempts {
			log.Warn("sensor init failed, retrying",
				"sensor", sensor,
				"attempt", i+1,
				"of", sensorInitAttempts,
				"backoff", sensorInitBackoff,
				"error", err)
			time.Sleep(sensorInitBackoff)
		}
	}
	return nil, lastErr
}

// initSensors initializes all sensors
func (a *App) initSensors() error {
	a.log.Info("initializing accelerometer")
	accel, err := initWithRetry("accelerometer", a.log, func() (*bmx.Accelerometer, error) {
		return bmx.NewAccelerometer(a.cfg.I2CBus)
	})
	if err != nil {
		return fmt.Errorf("init accelerometer: %w", err)
	}
	a.accel = accel

	a.log.Info("initializing gyroscope")
	gyro, err := initWithRetry("gyroscope", a.log, func() (*bmx.Gyroscope, error) {
		return bmx.NewGyroscope(a.cfg.I2CBus)
	})
	if err != nil {
		return fmt.Errorf("init gyroscope: %w", err)
	}
	a.gyro = gyro

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
		if err := a.accel.Close(); err != nil {
			a.log.Warn("failed to close accelerometer", "error", err)
		}
	}
	if a.gyro != nil {
		if err := a.gyro.Close(); err != nil {
			a.log.Warn("failed to close gyroscope", "error", err)
		}
	}
	if a.mag != nil {
		if err := a.mag.Close(); err != nil {
			a.log.Warn("failed to close magnetometer", "error", err)
		}
	}
}

// publishInitialStatus publishes initial status to Redis
func (a *App) publishInitialStatus(ctx context.Context) error {
	// Deliberately does not seed interrupt / pin / threshold / duration:
	// profile.Controller writes those from the spec it just programmed, so
	// seeding them here would publish a chip state that is not true yet.
	status := map[string]string{
		"initialized":              "true",
		"polling-rate-hz":          fmt.Sprintf("%d", a.cfg.PollingRate),
		"streaming":                "enabled",
		"last-interrupt-timestamp": "0",
		"error-count":              "0",
		"last-error":               "",
	}

	return a.publisher.PublishStatus(ctx, status)
}

// pollerGroup fans a single SetRate call out to multiple pollers so the
// subscriber doesn't need to know how many it's driving. Each entry
// receives the same rate.
type pollerGroup []redis.RateSetter

func (g pollerGroup) SetRate(rateHz int) {
	for _, p := range g {
		p.SetRate(rateHz)
	}
}
