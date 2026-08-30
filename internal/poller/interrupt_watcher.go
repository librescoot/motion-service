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

// InterruptWatcher is the low-latency GPIO path; InterruptPoller is the
// watchdog fallback for edges the evdev device misses.
type InterruptWatcher struct {
	devicePath string
	keycode    uint16
	accel      *bmx.Accelerometer
	publisher  *redis.Publisher
	log        *slog.Logger

	file    *os.File
	enabled atomic.Bool
}

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

// Open failure permits polling-only operation on hardware without this evdev node.
func (w *InterruptWatcher) Open() error {
	f, err := os.OpenFile(w.devicePath, os.O_RDONLY, 0)
	if err != nil {
		return fmt.Errorf("open %s: %w", w.devicePath, err)
	}
	w.file = f
	w.log.Info("interrupt watcher opened")
	return nil
}

func (w *InterruptWatcher) Close() {
	if w.file != nil {
		if err := w.file.Close(); err != nil {
			w.log.Warn("failed to close interrupt watcher device", "error", err)
		}
		w.file = nil
		w.log.Info("interrupt watcher closed")
	}
}

func (w *InterruptWatcher) Enable() {
	w.enabled.Store(true)
	w.log.Info("interrupt watcher enabled")
}

func (w *InterruptWatcher) Disable() {
	w.enabled.Store(false)
	w.log.Info("interrupt watcher disabled")
}

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

// Read status before clearing the latch so the public event retains its engine.
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
