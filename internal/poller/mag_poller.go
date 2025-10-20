package poller

import (
	"context"
	"log/slog"
	"time"

	"bmx-service/internal/bmx"
	"bmx-service/internal/redis"
)

const (
	magPollingRateHz = 5
	// Number of samples to average for smoothing (500ms at 5Hz = ~2.5 samples)
	headingSmoothingSamples = 3
)

// MagPoller continuously polls magnetometer and publishes data at 5Hz
type MagPoller struct {
	mag            *bmx.Magnetometer
	publisher      *redis.Publisher
	log            *slog.Logger
	headingHistory []float64
	historyIndex   int
	initialized    int // Number of samples collected
	pollCount      int // Counter for periodic logging
}

// NewMagPoller creates a new MagPoller
func NewMagPoller(
	mag *bmx.Magnetometer,
	publisher *redis.Publisher,
	log *slog.Logger,
) *MagPoller {
	return &MagPoller{
		mag:            mag,
		publisher:      publisher,
		log:            log,
		headingHistory: make([]float64, headingSmoothingSamples),
		historyIndex:   0,
	}
}

// Run starts the magnetometer polling loop at 5Hz
func (p *MagPoller) Run(ctx context.Context) {
	p.log.Info("starting magnetometer poller", "rate_hz", magPollingRateHz)

	ticker := time.NewTicker(time.Second / magPollingRateHz)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			p.log.Info("magnetometer poller stopped")
			return

		case <-ticker.C:
			if err := p.poll(ctx); err != nil {
				p.log.Error("failed to poll magnetometer", "error", err)
			}
		}
	}
}

// poll reads magnetometer and publishes the data and heading
func (p *MagPoller) poll(ctx context.Context) error {
	if p.mag == nil {
		return nil
	}

	// Read raw values for logging
	rawX, rawY, rawZ, err := p.mag.ReadData()
	if err != nil {
		return err
	}

	magX, magY, magZ, magMag, err := p.mag.ReadDataInMicroTesla()
	if err != nil {
		return err
	}

	// Log raw and compensated values every 5 seconds (25 polls at 5Hz)
	p.pollCount++
	if p.pollCount >= 25 {
		// Calculate compensated raw values for logging
		const hardIronX int16 = -441
		const hardIronY int16 = -259
		const hardIronZ int16 = -1164
		compX := rawX - hardIronX
		compY := rawY - hardIronY
		compZ := rawZ - hardIronZ

		p.log.Info("magnetometer values",
			"raw_x", rawX,
			"raw_y", rawY,
			"raw_z", rawZ,
			"comp_x", compX,
			"comp_y", compY,
			"comp_z", compZ,
			"uT_x", magX,
			"uT_y", magY,
			"uT_z", magZ)
		p.pollCount = 0
	}

	magData := &redis.SensorAxis{
		X:         magX,
		Y:         magY,
		Z:         magZ,
		Magnitude: magMag,
		Unit:      "uT",
	}

	if err := p.publisher.PublishMagnetometerData(ctx, magData); err != nil {
		p.log.Error("failed to publish magnetometer data", "error", err)
	}

	// Read and smooth heading
	heading, err := p.mag.ReadHeading()
	if err != nil {
		return err
	}

	smoothedHeading := p.smoothHeading(heading)

	// Log heading calculation details periodically
	if p.pollCount == 0 {
		p.log.Info("heading calculation",
			"raw_heading", heading,
			"smoothed", smoothedHeading)
	}

	return p.publisher.PublishMagnetometerHeading(ctx, smoothedHeading)
}

// smoothHeading applies a moving average filter to heading values
func (p *MagPoller) smoothHeading(newHeading float64) float64 {
	// Store new heading in circular buffer
	p.headingHistory[p.historyIndex] = newHeading
	p.historyIndex = (p.historyIndex + 1) % headingSmoothingSamples

	// Track initialization
	if p.initialized < headingSmoothingSamples {
		p.initialized++
	}

	// Calculate average
	var sum float64
	samplestoUse := p.initialized

	for i := 0; i < samplestoUse; i++ {
		sum += p.headingHistory[i]
	}

	return sum / float64(samplestoUse)
}
