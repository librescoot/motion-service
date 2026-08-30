package main

import (
	"context"
	"flag"
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"github.com/librescoot/motion-service/internal/app"
)

var (
	gitRevision = "unknown"
	buildTime   = "unknown"
)

func main() {
	i2cBus := flag.String("i2c-bus", "/dev/i2c-3", "I2C bus device path")
	redisAddr := flag.String("redis", "localhost:6379", "Redis address")
	logLevel := flag.String("log-level", "info", "Log level: debug, info, warn, error")
	pollingRate := flag.Int("polling-rate", 5, "Sensor polling rate (Hz)")
	evdevDevice := flag.String("evdev-device", "/dev/input/by-path/platform-gpio-keys-event", "Input device for the BMX055 INT gpio-keys edge (empty to disable and use I2C poller only)")
	evdevKeycode := flag.Int("evdev-keycode", 0x2b, "Keycode from gpio-keys device that corresponds to the BMX055 INT line")
	version := flag.Bool("version", false, "Print version and exit")
	flag.Parse()

	if *version {
		println("motion-service")
		println("  Revision:", gitRevision)
		println("  Built:", buildTime)
		os.Exit(0)
	}

	level := parseLogLevel(*logLevel)
	logger := slog.New(slog.NewTextHandler(os.Stdout, &slog.HandlerOptions{
		Level: level,
	}))
	slog.SetDefault(logger)

	logger.Info("motion-service starting",
		"revision", gitRevision,
		"build_time", buildTime,
		"i2c_bus", *i2cBus,
		"redis", *redisAddr,
		"polling_rate", *pollingRate,
		"log_level", *logLevel)

	application := app.New(&app.Config{
		I2CBus:       *i2cBus,
		RedisAddr:    *redisAddr,
		PollingRate:  *pollingRate,
		EvdevDevice:  *evdevDevice,
		EvdevKeycode: uint16(*evdevKeycode),
		Logger:       logger,
	})

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)

	errChan := make(chan error, 1)
	go func() {
		errChan <- application.Run(ctx)
	}()

	select {
	case sig := <-sigChan:
		logger.Info("received signal", "signal", sig)
		cancel()
		<-errChan

	case err := <-errChan:
		if err != nil {
			logger.Error("application error", "error", err)
			os.Exit(1)
		}
	}

	logger.Info("motion-service stopped")
}

func parseLogLevel(level string) slog.Level {
	switch level {
	case "debug":
		return slog.LevelDebug
	case "warn":
		return slog.LevelWarn
	case "error":
		return slog.LevelError
	default:
		return slog.LevelInfo
	}
}
