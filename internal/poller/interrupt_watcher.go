package poller

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"log/slog"
	"os"
	"sync/atomic"
	"time"

	"github.com/librescoot/motion-service/internal/bmx"
	"github.com/librescoot/motion-service/internal/redis"
)

const (
	evdevTypeKey   = 0x01
	evdevEventSize = 16
)

// InterruptWatcher reads the gpio-keys input event device for a specific
// keycode and publishes motion events the moment the BMX055 INT line rises,
// clearing the latched interrupt on the chip side. Runs alongside
// InterruptPoller — the watcher is the zero-latency primary path; the
// poller is a slow-tick watchdog for any missed edges.
type InterruptWatcher struct {
	devicePath string
	keycode    uint16
	accel      *bmx.Accelerometer
	publisher  *redis.Publisher
	log        *slog.Logger

	file    *os.File
	enabled atomic.Bool
}

// NewInterruptWatcher returns a watcher that opens devicePath and filters
// for key-press events matching keycode.
func NewInterruptWatcher(
	devicePath string,
	keycode uint16,
	accel *bmx.Accelerometer,
	publisher *redis.Publisher,
	log *slog.Logger,
) *InterruptWatcher {
	return &InterruptWatcher{
		devicePath: devicePath,
		keycode:    keycode,
		accel:      accel,
		publisher:  publisher,
		log:        log.With("evdev", devicePath, "keycode", keycode),
	}
}

// Open opens the input device. Returns an error if missing so the caller
// can fall back to polling-only.
func (w *InterruptWatcher) Open() error {
	f, err := os.OpenFile(w.devicePath, os.O_RDONLY, 0)
	if err != nil {
		return fmt.Errorf("open %s: %w", w.devicePath, err)
	}
	w.file = f
	w.log.Info("interrupt watcher opened")
	return nil
}

// Close releases the input device and unblocks any outstanding read.
func (w *InterruptWatcher) Close() {
	if w.file != nil {
		if err := w.file.Close(); err != nil {
			w.log.Warn("failed to close interrupt watcher device", "error", err)
		}
		w.file = nil
		w.log.Info("interrupt watcher closed")
	}
}

// Enable starts publishing motion events for incoming edges.
func (w *InterruptWatcher) Enable() {
	w.enabled.Store(true)
	w.log.Info("interrupt watcher enabled")
}

// Disable stops publishing motion events for incoming edges. The chip latch
// is still cleared on each edge so the line returns to idle.
func (w *InterruptWatcher) Disable() {
	w.enabled.Store(false)
	w.log.Info("interrupt watcher disabled")
}

// Run reads input events until ctx is cancelled or the device closes.
func (w *InterruptWatcher) Run(ctx context.Context) {
	w.log.Info("starting interrupt watcher")
	defer w.Close()

	buf := make([]byte, evdevEventSize)
	for {
		if ctx.Err() != nil {
			return
		}
		n, err := w.file.Read(buf)
		if err != nil {
			if errors.Is(err, io.EOF) || errors.Is(err, os.ErrClosed) || ctx.Err() != nil {
				return
			}
			w.log.Error("input read failed", "error", err)
			time.Sleep(100 * time.Millisecond)
			continue
		}
		if n != evdevEventSize {
			w.log.Warn("short evdev read", "bytes", n)
			continue
		}

		typ := binary.LittleEndian.Uint16(buf[8:10])
		code := binary.LittleEndian.Uint16(buf[10:12])
		val := int32(binary.LittleEndian.Uint32(buf[12:16]))

		if typ != evdevTypeKey || code != w.keycode || val != 1 {
			continue
		}

		if !w.enabled.Load() {
			w.log.Debug("dropping interrupt edge (watcher disabled)")
			continue
		}

		w.handleEdge(ctx)
	}
}

// handleEdge reads INT_STATUS_0 to determine which engine fired,
// publishes the MotionEvent, then clears the latch. Read-before-clear so
// we can tell any-motion vs slow-motion in the published event.
func (w *InterruptWatcher) handleEdge(ctx context.Context) {
	ts := time.Now().UnixMilli()

	engine := ""
	if status, err := w.accel.ReadByteData(bmx.ACCEL_INT_STATUS_0); err != nil {
		w.log.Warn("read INT_STATUS_0 failed", "error", err)
	} else {
		engine = engineNameFor(
			(status&bmx.ACCEL_INT_STATUS_SLOPE) != 0,
			(status&bmx.ACCEL_INT_STATUS_SLOW_NO_MOT) != 0,
		)
	}

	w.log.Info("motion interrupt edge", "timestamp", ts, "engine", engine)

	if err := w.publisher.PublishMotionEvent(ctx, &redis.MotionEvent{
		Type:      "edge",
		Timestamp: ts,
		Engine:    engine,
	}); err != nil {
		w.log.Error("failed to publish motion event", "error", err)
	}

	if err := w.accel.ClearLatchedInterrupt(); err != nil {
		w.log.Warn("failed to clear latched interrupt", "error", err)
	}
}
