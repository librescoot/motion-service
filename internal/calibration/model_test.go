package calibration

import (
	"math"
	"path/filepath"
	"testing"
)

func TestFitPlanarRecoversOffsetAndCircularizesEllipse(t *testing.T) {
	const centerX, centerY = 137.0, -284.0
	const radiusX, radiusY = 520.0, 310.0
	var samples []Sample
	for i := 0; i < 720; i++ {
		angle := 2 * math.Pi * float64(i) / 720
		// Rotate the ellipse to exercise cross-axis correction.
		ex, ey := radiusX*math.Cos(angle), radiusY*math.Sin(angle)
		rotation := 23.0 * math.Pi / 180
		x := centerX + ex*math.Cos(rotation) - ey*math.Sin(rotation)
		y := centerY + ex*math.Sin(rotation) + ey*math.Cos(rotation)
		samples = append(samples, goodSample(x, y))
	}
	model, err := FitPlanar(samples)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(model.Offset[0]-centerX) > 1e-6 || math.Abs(model.Offset[1]-centerY) > 1e-6 {
		t.Fatalf("offset = %v, want [%v %v]", model.Offset, centerX, centerY)
	}
	if model.ResidualRMS > 1e-8 || model.Condition < 1.5 || model.Condition > 1.8 {
		t.Fatalf("unexpected fit quality: %+v", model)
	}
}

func TestFitPlanarRejectsDistortedCapture(t *testing.T) {
	var samples []Sample
	for i := 0; i < 720; i++ {
		angle := 2 * math.Pi * float64(i) / 720
		radius := 400.0
		if i%7 == 0 {
			radius = 700 // repeatable directional outliers, not ellipse noise
		}
		samples = append(samples, goodSample(radius*math.Cos(angle), radius*math.Sin(angle)))
	}
	if _, err := FitPlanar(samples); err == nil {
		t.Fatal("distorted capture unexpectedly accepted")
	}
}

func TestModelRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "calibration.json")
	model := PlanarModel{
		Version: modelVersion, Offset: [2]float64{10, -20},
		Matrix:      [2][2]float64{{1.2, 0.1}, {0.1, 0.85}},
		ResidualRMS: 0.03, Condition: 1.4, Samples: 500, CoverageBins: 36,
	}
	if err := SaveModel(path, model); err != nil {
		t.Fatal(err)
	}
	loaded, err := LoadModel(path)
	if err != nil {
		t.Fatal(err)
	}
	if loaded.Offset != model.Offset || loaded.Matrix != model.Matrix {
		t.Fatalf("loaded model differs: %+v", loaded)
	}
}
