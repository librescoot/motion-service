package poller

import (
	"context"
	"log/slog"
	"sync"
	"sync/atomic"
	"time"

	"github.com/librescoot/motion-service/internal/bmx"
	"github.com/librescoot/motion-service/internal/redis"
)

type SensorPoller struct {
	accel     *bmx.Accelerometer
	gyro      *bmx.Gyroscope
	mag       *bmx.Magnetometer
	publisher *redis.Publisher
	cache     *bmx.SensorCache
	log       *slog.Logger

	rateHz     atomic.Int32
	rateChange chan struct{}
	enabled    bool
	mu         sync.RWMutex
}

func NewSensorPoller(
	accel *bmx.Accelerometer,
	gyro *bmx.Gyroscope,
	mag *bmx.Magnetometer,
	publisher *redis.Publisher,
	cache *bmx.SensorCache,
	rateHz int,
	log *slog.Logger,
) *SensorPoller {
	p := &SensorPoller{
		accel:     accel,
		gyro:      gyro,
		mag:       mag,
		publisher: publisher,
		cache:     cache,
		log:       log,

		enabled:    true,
		rateChange: make(chan struct{}, 1),
	}
	p.rateHz.Store(int32(rateHz))
	return p
}

func (p *SensorPoller) Enable() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.enabled = true
	p.log.Info("sensor streaming enabled")
}

func (p *SensorPoller) Disable() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.enabled = false
	p.log.Info("sensor streaming disabled")
}

func (p *SensorPoller) IsEnabled() bool {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.enabled
}

func (p *SensorPoller) SetRate(rateHz int) {
	if int(p.rateHz.Load()) == rateHz {
		return
	}
	p.rateHz.Store(int32(rateHz))
	select {
	case p.rateChange <- struct{}{}:
	default:
	}
	p.log.Info("sensor polling rate set", "rate_hz", rateHz)
}

func (p *SensorPoller) Run(ctx context.Context) {
	for {
		rate := int(p.rateHz.Load())
		if rate <= 0 {
			p.log.Info("sensor poller suspended (rate=0)")
			select {
			case <-ctx.Done():
				p.log.Info("sensor poller stopped")
				return
			case <-p.rateChange:
				continue
			}
		}

		p.log.Info("sensor poller running", "rate_hz", rate)
		ticker := time.NewTicker(time.Second / time.Duration(rate))

	ticking:
		for {
			select {
			case <-ctx.Done():
				ticker.Stop()
				p.log.Info("sensor poller stopped")
				return
			case <-p.rateChange:
				ticker.Stop()
				break ticking
			case <-ticker.C:
				if p.IsEnabled() {
					if err := p.poll(ctx); err != nil {
						p.log.Error("failed to poll sensors", "error", err)
					}
				}
			}
		}
	}
}

func (p *SensorPoller) poll(ctx context.Context) error {
	orientation := bmx.Orientation{
		AxisOrder: [3]int{0, 1, 2},
		AxisSign:  [3]float64{1, 1, 1},
	}
	if p.mag != nil {
		orientation = p.mag.Orientation()
	}

	accelX, accelY, accelZ, accelMag, err := p.accel.ReadDataInGVehicleFrame(orientation)
	if err != nil {
		return err
	}

	gyroX, gyroY, gyroZ, gyroMag, err := p.gyro.ReadDataInDPSVehicleFrame(orientation)
	if err != nil {
		return err
	}

	if p.cache != nil {
		p.cache.StoreIMU(bmx.IMUSnapshot{
			Timestamp: time.Now(),
			AccelX:    accelX, AccelY: accelY, AccelZ: accelZ, AccelMag: accelMag,
			GyroX: gyroX, GyroY: gyroY, GyroZ: gyroZ, GyroMag: gyroMag,
		})
	}

	reading := &redis.SensorReading{
		Timestamp: time.Now().UnixMilli(),
		Accel: redis.SensorAxis{
			X:         accelX,
			Y:         accelY,
			Z:         accelZ,
			Magnitude: accelMag,
			Unit:      "g",
		},
		Gyro: redis.SensorAxis{
			X:         gyroX,
			Y:         gyroY,
			Z:         gyroZ,
			Magnitude: gyroMag,
			Unit:      "deg/s",
		},
	}

	if p.mag != nil {

		var (
			mSnap  bmx.MagSnapshot
			cached bool
		)
		if p.cache != nil {
			mSnap, cached = p.cache.LoadMag(150 * time.Millisecond)
		}
		if cached {
			reading.Mag = &redis.SensorAxis{
				X:         mSnap.X,
				Y:         mSnap.Y,
				Z:         mSnap.Z,
				Magnitude: mSnap.Magnitude,
				Unit:      "uT",
			}
		} else {
			compX, compY, compZ, magX, magY, magZ, magMag, _, mErr := p.mag.ReadAll()
			if mErr == nil {
				reading.Mag = &redis.SensorAxis{
					X:         magX,
					Y:         magY,
					Z:         magZ,
					Magnitude: magMag,
					Unit:      "uT",
				}
				if p.cache != nil {
					p.cache.StoreMag(bmx.MagSnapshot{
						Timestamp: time.Now(),
						CompX:     compX, CompY: compY, CompZ: compZ,
						X: magX, Y: magY, Z: magZ, Magnitude: magMag,
					})
				}
			}
		}
	}

	return p.publisher.PublishSensorData(ctx, reading)
}
