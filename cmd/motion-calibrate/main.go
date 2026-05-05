// bmx-calibrate captures raw BMX055 sensor data to a CSV file for offline
// magnetometer calibration. It is intended to be run as a one-shot systemd
// unit that conflicts with librescoot-alarm and librescoot-bmx so the BMX055
// is exclusively owned during capture.
//
// Output is /data/bmx-cal-<unix-ts>.csv by default, with a header line and
// one row per sample:
//
//	timestamp_ms,mag_raw_x,mag_raw_y,mag_raw_z,
//	  ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps
//
// Raw mag values are post-temperature-compensation int16 LSB, before any
// hard-iron correction — that's what an offline calibration fit needs.
//
// Live progress prints running min/max and the implied hard-iron offset
// every progress-every. On exit a JSON calibration summary is printed for
// drop-in use as the magnetometer Calibration struct.
package main

import (
	"bufio"
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"log/slog"
	"math"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	"github.com/librescoot/motion-service/internal/bmx"
	"github.com/librescoot/motion-service/internal/driver"
)

func main() {
	bus := flag.String("i2c-bus", "/dev/i2c-3", "I2C bus device path")
	output := flag.String("output", "", "CSV output path (default /data/bmx-cal-<unix>.csv)")
	rateHz := flag.Int("rate", 20, "Sampling rate in Hz")
	duration := flag.Duration("duration", 0, "Stop after this duration (0 = until SIGINT/SIGTERM)")
	preset := flag.String("preset", "highacc", "Magnetometer preset: regular|enhanced|highacc")
	progressEvery := flag.Duration("progress-every", 2*time.Second, "Live progress interval")
	flag.Parse()

	logger := newLogger()
	slog.SetDefault(logger)

	if *output == "" {
		*output = fmt.Sprintf("/data/bmx-cal-%d.csv", time.Now().Unix())
	}
	if err := os.MkdirAll(filepath.Dir(*output), 0o755); err != nil {
		logger.Error("failed to create output directory", "path", filepath.Dir(*output), "error", err)
		os.Exit(1)
	}

	logger.Info("bmx-calibrate starting",
		"i2c", *bus,
		"output", *output,
		"rate_hz", *rateHz,
		"preset", *preset,
		"duration", *duration)

	if err := driver.UnbindBMX055(); err != nil {
		logger.Warn("kernel driver unbind reported error (continuing)", "error", err)
	}
	time.Sleep(100 * time.Millisecond)

	accel, err := bmx.NewAccelerometer(*bus)
	if err != nil {
		logger.Error("accelerometer init failed", "error", err)
		os.Exit(1)
	}
	defer accel.Close()

	gyro, err := bmx.NewGyroscope(*bus)
	if err != nil {
		logger.Error("gyroscope init failed", "error", err)
		os.Exit(1)
	}
	defer gyro.Close()

	mag, err := bmx.NewMagnetometer(*bus)
	if err != nil {
		logger.Error("magnetometer init failed", "error", err)
		os.Exit(1)
	}
	defer mag.Close()

	if err := applyMagPreset(mag, *preset); err != nil {
		logger.Error("failed to apply mag preset", "preset", *preset, "error", err)
		os.Exit(2)
	}

	// Disable hard-iron / orientation baked into Magnetometer so anything
	// that reads µT here represents the raw chip output. We don't actually
	// log µT to the CSV, but a future revision might, and this keeps
	// downstream consumers honest.
	mag.SetCalibration(bmx.Calibration{
		HardIronOffset: [3]int16{0, 0, 0},
		Orientation: bmx.Orientation{
			AxisOrder: [3]int{0, 1, 2},
			AxisSign:  [3]float64{1, 1, 1},
		},
		YawOffsetDeg: 0,
	})

	f, err := os.Create(*output)
	if err != nil {
		logger.Error("failed to open output file", "path", *output, "error", err)
		os.Exit(1)
	}
	defer f.Close()

	w := bufio.NewWriter(f)
	defer w.Flush()
	if _, err := fmt.Fprintln(w, "timestamp_ms,mag_raw_x,mag_raw_y,mag_raw_z,mag_rhall,mag_drdy,mag_comp_x,mag_comp_y,mag_comp_z,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps"); err != nil {
		logger.Error("failed to write CSV header", "error", err)
		os.Exit(1)
	}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-sigCh
		logger.Info("signal received, stopping")
		cancel()
	}()

	if *duration > 0 {
		time.AfterFunc(*duration, func() {
			logger.Info("duration reached, stopping")
			cancel()
		})
	}

	st := newStats()
	tick := time.NewTicker(time.Second / time.Duration(*rateHz))
	defer tick.Stop()
	progress := time.NewTicker(*progressEvery)
	defer progress.Stop()

	start := time.Now()
	samples := 0

loop:
	for {
		select {
		case <-ctx.Done():
			break loop

		case <-progress.C:
			st.print(logger, samples, time.Since(start))

		case t := <-tick.C:
			rawX, rawY, rawZ, rhall, drdy, err := mag.ReadRaw()
			if err != nil {
				logger.Warn("mag read failed", "error", err)
				continue
			}
			compX, compY, compZ, err := mag.ReadData()
			if err != nil {
				logger.Warn("mag compensated read failed", "error", err)
				continue
			}
			ax, ay, az, _, err := accel.ReadDataInG()
			if err != nil {
				logger.Warn("accel read failed", "error", err)
				continue
			}
			gx, gy, gz, _, err := gyro.ReadDataInDPS()
			if err != nil {
				logger.Warn("gyro read failed", "error", err)
				continue
			}

			st.add(rawX, rawY, rawZ)
			samples++

			drdyInt := 0
			if drdy {
				drdyInt = 1
			}
			if _, werr := fmt.Fprintf(w,
				"%d,%d,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n",
				t.UnixMilli(), rawX, rawY, rawZ, rhall, drdyInt,
				compX, compY, compZ,
				ax, ay, az, gx, gy, gz); werr != nil {
				logger.Error("CSV write failed; aborting", "error", werr)
				cancel()
			}
		}
	}

	if err := w.Flush(); err != nil {
		logger.Warn("CSV flush failed", "error", err)
	}

	dt := time.Since(start)
	logger.Info("capture complete",
		"samples", samples,
		"duration", dt.String(),
		"output", *output)

	summary := st.summary(samples, dt)
	logger.Info("suggested calibration follows on stdout")
	fmt.Println(summary)
}

func newLogger() *slog.Logger {
	opts := &slog.HandlerOptions{Level: slog.LevelInfo}
	if os.Getenv("JOURNAL_STREAM") != "" {
		opts.ReplaceAttr = func(_ []string, a slog.Attr) slog.Attr {
			if a.Key == slog.TimeKey {
				return slog.Attr{}
			}
			return a
		}
	}
	return slog.New(slog.NewTextHandler(os.Stdout, opts))
}

func applyMagPreset(mag *bmx.Magnetometer, preset string) error {
	var repxy, repz, odr byte
	switch preset {
	case "regular":
		repxy, repz, odr = bmx.MAG_REPXY_REGULAR, bmx.MAG_REPZ_REGULAR, byte(bmx.MAG_ODR_10HZ)
	case "enhanced":
		repxy, repz, odr = bmx.MAG_REPXY_ENHANCED, bmx.MAG_REPZ_ENHANCED, byte(bmx.MAG_ODR_10HZ)
	case "highacc", "high":
		repxy, repz, odr = bmx.MAG_REPXY_HIGHACC, bmx.MAG_REPZ_HIGHACC, byte(bmx.MAG_ODR_20HZ)
	default:
		return fmt.Errorf("unknown preset %q (want regular|enhanced|highacc)", preset)
	}

	if err := mag.WriteByteData(bmx.MAG_REPXY, repxy); err != nil {
		return err
	}
	if err := mag.WriteByteData(bmx.MAG_REPZ, repz); err != nil {
		return err
	}
	return mag.WriteByteData(bmx.MAG_OPMODE_ODR, byte(bmx.MAG_OPMODE_NORMAL)|odr)
}

type stats struct {
	minX, minY, minZ int16
	maxX, maxY, maxZ int16
}

func newStats() *stats {
	return &stats{
		minX: math.MaxInt16, minY: math.MaxInt16, minZ: math.MaxInt16,
		maxX: math.MinInt16, maxY: math.MinInt16, maxZ: math.MinInt16,
	}
}

func (s *stats) add(x, y, z int16) {
	if x < s.minX {
		s.minX = x
	}
	if x > s.maxX {
		s.maxX = x
	}
	if y < s.minY {
		s.minY = y
	}
	if y > s.maxY {
		s.maxY = y
	}
	if z < s.minZ {
		s.minZ = z
	}
	if z > s.maxZ {
		s.maxZ = z
	}
}

func (s *stats) hardIron() (int16, int16, int16) {
	return int16((int32(s.maxX) + int32(s.minX)) / 2),
		int16((int32(s.maxY) + int32(s.minY)) / 2),
		int16((int32(s.maxZ) + int32(s.minZ)) / 2)
}

func (s *stats) print(log *slog.Logger, n int, dt time.Duration) {
	if n == 0 {
		log.Info("progress: no samples yet")
		return
	}
	hX, hY, hZ := s.hardIron()
	rate := float64(n) / dt.Seconds()
	log.Info("progress",
		"samples", n,
		"rate_hz", fmt.Sprintf("%.1f", rate),
		"x", fmt.Sprintf("[%d,%d] span=%d", s.minX, s.maxX, s.maxX-s.minX),
		"y", fmt.Sprintf("[%d,%d] span=%d", s.minY, s.maxY, s.maxY-s.minY),
		"z", fmt.Sprintf("[%d,%d] span=%d", s.minZ, s.maxZ, s.maxZ-s.minZ),
		"hard_iron_xyz", fmt.Sprintf("[%d,%d,%d]", hX, hY, hZ))
}

type calOut struct {
	HardIronOffset [3]int16   `json:"hard_iron_offset"`
	AxisSign       [3]float64 `json:"axis_sign"`
	YawOffsetDeg   float64    `json:"yaw_offset_deg"`
	Samples        int        `json:"samples"`
	DurationS      float64    `json:"duration_s"`
	CapturedAt     string     `json:"captured_at"`
	Note           string     `json:"note"`
}

func (s *stats) summary(n int, dt time.Duration) string {
	hX, hY, hZ := s.hardIron()
	out := calOut{
		HardIronOffset: [3]int16{hX, hY, hZ},
		AxisSign:       [3]float64{1, 1, 1},
		YawOffsetDeg:   0,
		Samples:        n,
		DurationS:      dt.Seconds(),
		CapturedAt:     time.Now().UTC().Format(time.RFC3339),
		Note: "axis_sign and yaw_offset_deg are placeholders — set them by " +
			"the spin-direction and known-North checks described in README.md.",
	}
	b, _ := json.MarshalIndent(out, "", "  ")
	return string(b)
}
