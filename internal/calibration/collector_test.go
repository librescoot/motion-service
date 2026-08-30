package calibration

import (
	"errors"
	"math"
	"os"
	"path/filepath"
	"testing"
)

func TestCollectorRequiresDirectionalCoverage(t *testing.T) {
	collector := NewCollector(t.TempDir())
	collector.Start()
	for i := 0; i < minSamples; i++ {
		collector.Add(goodSample(100, 200))
	}
	status := collector.Status()
	if status.Ready || status.CoverageBins > 2 {
		t.Fatalf("repeated orientation unexpectedly ready: %+v", status)
	}
	if _, err := collector.Finish(); !errors.Is(err, ErrInsufficientCoverage) {
		t.Fatalf("Finish error = %v, want insufficient coverage", err)
	}
}

func TestCollectorFinishesCoveredCaptureAtomically(t *testing.T) {
	dir := t.TempDir()
	var applied *PlanarModel
	collector := NewCollector(dir, func(model *PlanarModel) { applied = model })
	collector.Start()
	for i := 0; i < 720; i++ {
		angle := 2 * math.Pi * float64(i) / 720
		collector.Add(goodSample(40+360*math.Cos(angle), 320+300*math.Sin(angle)))
	}
	status := collector.Status()
	if !status.Ready || status.CoverageBins < minCoverageBins {
		t.Fatalf("covered capture not ready: %+v", status)
	}
	status, err := collector.Finish()
	if err != nil {
		t.Fatal(err)
	}
	if status.State != "calibrated" || status.OutputPath == "" {
		t.Fatalf("unexpected finish status: %+v", status)
	}
	if _, err := os.Stat(status.OutputPath); err != nil {
		t.Fatalf("output missing: %v", err)
	}
	if applied == nil || status.ModelPath == "" {
		t.Fatalf("model was not applied: %+v", status)
	}
	if _, err := os.Stat(status.ModelPath); err != nil {
		t.Fatalf("model missing: %v", err)
	}
	status, err = collector.Reset()
	if err != nil || applied != nil || status.ModelPath != "" {
		t.Fatalf("reset failed: status=%+v applied=%v err=%v", status, applied, err)
	}
}

func TestFailedCapturePreservesPreviousModel(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, ModelFilename)
	previous := PlanarModel{
		Version: modelVersion, Offset: [2]float64{12, -34},
		Matrix: [2][2]float64{{1, 0}, {0, 1}}, ResidualRMS: 0.02,
		Condition: 1, Samples: 500, CoverageBins: 36,
	}
	if err := SaveModel(path, previous); err != nil {
		t.Fatal(err)
	}
	collector := NewCollector(dir)
	collector.Start()
	for i := 0; i < minSamples; i++ {
		collector.Add(goodSample(100, 200))
	}
	if _, err := collector.Finish(); !errors.Is(err, ErrInsufficientCoverage) {
		t.Fatalf("Finish error = %v", err)
	}
	loaded, err := LoadModel(path)
	if err != nil {
		t.Fatal(err)
	}
	if loaded.Offset != previous.Offset || loaded.Matrix != previous.Matrix {
		t.Fatalf("previous model changed: %+v", loaded)
	}
}

func TestCollectorRejectsDynamicSamples(t *testing.T) {
	collector := NewCollector(t.TempDir())
	collector.Start()
	sample := goodSample(0, 0)
	sample.ExcessG = 0.2
	collector.Add(sample)
	status := collector.Status()
	if status.AcceptedSamples != 0 || status.RejectedSamples != 1 {
		t.Fatalf("unexpected status: %+v", status)
	}
}

func goodSample(x, y float64) Sample {
	return Sample{
		Timestamp: 1, CompX: x, CompY: y, CompZ: 900,
		AccelZ: -1, FieldUT: 48, ExcessG: 0.01, TiltDeg: 3,
	}
}
