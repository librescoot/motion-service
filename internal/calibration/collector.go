package calibration

import (
	"encoding/csv"
	"errors"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"sync"
	"time"
)

const (
	coverageBinCount = 36
	minSamples       = 180
	minCoverageBins  = 30
	minAxisSpan      = 300.0 // compensated LSB, about 19 uT
	maxSamples       = 12000
)

var ErrNotCollecting = errors.New("calibration capture is not running")
var ErrInsufficientCoverage = errors.New("calibration capture has insufficient coverage")

type Sample struct {
	Timestamp                 int64
	CompX, CompY, CompZ       float64
	AccelX, AccelY, AccelZ    float64
	FieldUT, ExcessG, TiltDeg float64
}

type Status struct {
	State           string  `json:"state"`
	AcceptedSamples int     `json:"accepted_samples"`
	RejectedSamples int     `json:"rejected_samples"`
	CoverageBins    int     `json:"coverage_bins"`
	RequiredBins    int     `json:"required_bins"`
	SpanX           float64 `json:"span_x"`
	SpanY           float64 `json:"span_y"`
	Ready           bool    `json:"ready"`
	ResidualRMS     float64 `json:"residual_rms,omitempty"`
	Condition       float64 `json:"condition,omitempty"`
	OutputPath      string  `json:"output_path,omitempty"`
	ModelPath       string  `json:"model_path,omitempty"`
}

type Collector struct {
	mu         sync.Mutex
	outputDir  string
	modelPath  string
	apply      func(*PlanarModel)
	collecting bool
	samples    []Sample
	rejected   int
	lastOutput string
	lastModel  *PlanarModel
}

func NewCollector(outputDir string, apply ...func(*PlanarModel)) *Collector {
	collector := &Collector{
		outputDir: outputDir,
		modelPath: filepath.Join(outputDir, ModelFilename),
	}
	if len(apply) != 0 {
		collector.apply = apply[0]
	}
	if model, err := LoadModel(collector.modelPath); err == nil {
		collector.lastModel = &model
	}
	return collector
}

func (c *Collector) Start() Status {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.collecting = true
	c.samples = c.samples[:0]
	c.rejected = 0
	c.lastOutput = ""
	return c.statusLocked()
}

func (c *Collector) Add(sample Sample) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if !c.collecting {
		return
	}
	if !acceptable(sample) || len(c.samples) >= maxSamples {
		c.rejected++
		return
	}
	c.samples = append(c.samples, sample)
}

func acceptable(sample Sample) bool {
	return finite(sample.CompX) && finite(sample.CompY) && finite(sample.CompZ) &&
		finite(sample.FieldUT) && sample.FieldUT >= 10 && sample.FieldUT <= 100 &&
		finite(sample.ExcessG) && sample.ExcessG <= 0.05 &&
		finite(sample.TiltDeg) && sample.TiltDeg <= 25
}

func (c *Collector) Status() Status {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.statusLocked()
}

func (c *Collector) Cancel() Status {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.collecting = false
	c.samples = nil
	return c.statusLocked()
}

func (c *Collector) Reset() (Status, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := os.Remove(c.modelPath); err != nil && !errors.Is(err, os.ErrNotExist) {
		return c.statusLocked(), err
	}
	c.collecting = false
	c.samples = nil
	c.rejected = 0
	c.lastModel = nil
	c.lastOutput = ""
	if c.apply != nil {
		c.apply(nil)
	}
	return c.statusLocked(), nil
}

func (c *Collector) Finish() (Status, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if !c.collecting {
		return c.statusLocked(), ErrNotCollecting
	}
	status := c.statusLocked()
	if !status.Ready {
		return status, ErrInsufficientCoverage
	}
	model, err := FitPlanar(c.samples)
	if err != nil {
		return status, err
	}
	if err := os.MkdirAll(c.outputDir, 0o755); err != nil {
		return status, err
	}
	path := filepath.Join(c.outputDir,
		fmt.Sprintf("motion-calibration-%d.csv", time.Now().Unix()))
	if err := writeCSVAtomic(path, c.samples); err != nil {
		return status, err
	}
	// Save only after fitting and validating succeeds. Atomic replacement
	// preserves the previous working model on every failure path above.
	if err := SaveModel(c.modelPath, model); err != nil {
		return status, err
	}
	if c.apply != nil {
		c.apply(&model)
	}
	c.collecting = false
	c.lastOutput = path
	c.lastModel = &model
	status = c.statusLocked()
	return status, nil
}

func (c *Collector) statusLocked() Status {
	state := "uncalibrated"
	if c.lastModel != nil {
		state = "calibrated"
	}
	if c.collecting {
		state = "collecting"
	}
	status := Status{
		State: state, AcceptedSamples: len(c.samples), RejectedSamples: c.rejected,
		RequiredBins: minCoverageBins, OutputPath: c.lastOutput,
	}
	if c.lastModel != nil {
		status.ResidualRMS = c.lastModel.ResidualRMS
		status.Condition = c.lastModel.Condition
		status.ModelPath = c.modelPath
	}
	status.CoverageBins, status.SpanX, status.SpanY = coverage(c.samples)
	status.Ready = len(c.samples) >= minSamples &&
		status.CoverageBins >= minCoverageBins &&
		status.SpanX >= minAxisSpan && status.SpanY >= minAxisSpan
	return status
}

func coverage(samples []Sample) (occupied int, spanX, spanY float64) {
	if len(samples) == 0 {
		return 0, 0, 0
	}
	minX, maxX := samples[0].CompX, samples[0].CompX
	minY, maxY := samples[0].CompY, samples[0].CompY
	for _, sample := range samples[1:] {
		minX, maxX = math.Min(minX, sample.CompX), math.Max(maxX, sample.CompX)
		minY, maxY = math.Min(minY, sample.CompY), math.Max(maxY, sample.CompY)
	}
	centerX, centerY := (minX+maxX)/2, (minY+maxY)/2
	bins := [coverageBinCount]bool{}
	for _, sample := range samples {
		angle := math.Atan2(sample.CompY-centerY, sample.CompX-centerX)
		index := int(math.Floor((angle + math.Pi) / (2 * math.Pi) * coverageBinCount))
		if index == coverageBinCount {
			index = 0
		}
		bins[index] = true
	}
	for _, used := range bins {
		if used {
			occupied++
		}
	}
	return occupied, maxX - minX, maxY - minY
}

func writeCSVAtomic(path string, samples []Sample) error {
	tmp, err := os.CreateTemp(filepath.Dir(path), ".motion-calibration-*.tmp")
	if err != nil {
		return err
	}
	tmpPath := tmp.Name()
	ok := false
	defer func() {
		_ = tmp.Close()
		if !ok {
			_ = os.Remove(tmpPath)
		}
	}()
	writer := csv.NewWriter(tmp)
	if err := writer.Write([]string{"timestamp_ms", "mag_comp_x", "mag_comp_y", "mag_comp_z", "ax_g", "ay_g", "az_g", "field_ut", "excess_g", "tilt_deg"}); err != nil {
		return err
	}
	for _, sample := range samples {
		row := []string{
			strconv.FormatInt(sample.Timestamp, 10),
			formatFloat(sample.CompX), formatFloat(sample.CompY), formatFloat(sample.CompZ),
			formatFloat(sample.AccelX), formatFloat(sample.AccelY), formatFloat(sample.AccelZ),
			formatFloat(sample.FieldUT), formatFloat(sample.ExcessG), formatFloat(sample.TiltDeg),
		}
		if err := writer.Write(row); err != nil {
			return err
		}
	}
	writer.Flush()
	if err := writer.Error(); err != nil {
		return err
	}
	if err := tmp.Sync(); err != nil {
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	if err := os.Rename(tmpPath, path); err != nil {
		return err
	}
	ok = true
	return nil
}

func formatFloat(value float64) string { return strconv.FormatFloat(value, 'f', 6, 64) }
func finite(value float64) bool        { return !math.IsNaN(value) && !math.IsInf(value, 0) }
