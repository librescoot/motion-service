package calibration

import (
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"time"
)

const (
	modelVersion  = 1
	ModelFilename = "motion-magnetometer-calibration.json"
)

var ErrInvalidFit = errors.New("invalid planar magnetometer fit")

// PlanarModel corrects hard-iron displacement and in-plane soft-iron
// distortion in the sensor's factory-compensated X/Y units. Matrix has unit
// determinant, preserving the field scale while mapping the fitted ellipse to
// a circle.
type PlanarModel struct {
	Version      int           `json:"version"`
	CreatedAt    time.Time     `json:"created_at"`
	Offset       [2]float64    `json:"offset_xy"`
	Matrix       [2][2]float64 `json:"matrix_xy"`
	ResidualRMS  float64       `json:"residual_rms"`
	Condition    float64       `json:"condition"`
	Samples      int           `json:"samples"`
	CoverageBins int           `json:"coverage_bins"`
}

func FitPlanar(samples []Sample) (PlanarModel, error) {
	if len(samples) < minSamples {
		return PlanarModel{}, fmt.Errorf("%w: only %d samples", ErrInvalidFit, len(samples))
	}
	bins, spanX, spanY := coverage(samples)
	if bins < minCoverageBins || spanX < minAxisSpan || spanY < minAxisSpan {
		return PlanarModel{}, fmt.Errorf("%w: coverage %d/%d, spans %.1f/%.1f", ErrInvalidFit, bins, minCoverageBins, spanX, spanY)
	}

	var meanX, meanY float64
	for _, sample := range samples {
		meanX += sample.CompX
		meanY += sample.CompY
	}
	meanX /= float64(len(samples))
	meanY /= float64(len(samples))
	var variance float64
	for _, sample := range samples {
		dx, dy := sample.CompX-meanX, sample.CompY-meanY
		variance += dx*dx + dy*dy
	}
	scale := math.Sqrt(variance / float64(len(samples)))
	if !finite(scale) || scale < 1 {
		return PlanarModel{}, fmt.Errorf("%w: degenerate scale", ErrInvalidFit)
	}

	// Least-squares conic in normalized coordinates:
	// A*u² + B*u*v + C*v² + D*u + E*v = 1.
	var normal [5][5]float64
	var rhs [5]float64
	for _, sample := range samples {
		u, v := (sample.CompX-meanX)/scale, (sample.CompY-meanY)/scale
		row := [5]float64{u * u, u * v, v * v, u, v}
		for i := range row {
			rhs[i] += row[i]
			for j := range row {
				normal[i][j] += row[i] * row[j]
			}
		}
	}
	params, ok := solve5(normal, rhs)
	if !ok {
		return PlanarModel{}, fmt.Errorf("%w: singular conic", ErrInvalidFit)
	}
	a, b, c, d, e := params[0], params[1]/2, params[2], params[3], params[4]
	det := a*c - b*b
	if a <= 0 || c <= 0 || det <= 1e-9 {
		return PlanarModel{}, fmt.Errorf("%w: conic is not an ellipse", ErrInvalidFit)
	}
	centerU := -(c*d - b*e) / (2 * det)
	centerV := -(-b*d + a*e) / (2 * det)
	k := 1 + a*centerU*centerU + 2*b*centerU*centerV + c*centerV*centerV
	if !finite(k) || k <= 0 {
		return PlanarModel{}, fmt.Errorf("%w: invalid ellipse radius", ErrInvalidFit)
	}

	trace := a + c
	disc := math.Sqrt(math.Max(0, (a-c)*(a-c)+4*b*b))
	lambdaHigh, lambdaLow := (trace+disc)/2, (trace-disc)/2
	if lambdaLow <= 0 {
		return PlanarModel{}, fmt.Errorf("%w: non-positive ellipse axis", ErrInvalidFit)
	}
	condition := math.Sqrt(lambdaHigh / lambdaLow)
	if !finite(condition) || condition > 4 {
		return PlanarModel{}, fmt.Errorf("%w: condition %.2f exceeds 4", ErrInvalidFit, condition)
	}

	// Symmetric square root of Q. The target geometric-mean radius makes the
	// correction determinant one, retaining compensated-LSB scale.
	sqrtQ := symmetricSqrt(a, b, c, lambdaHigh, lambdaLow)
	target := math.Sqrt(k / math.Sqrt(lambdaHigh*lambdaLow))
	factor := target / math.Sqrt(k)
	matrix := [2][2]float64{
		{factor * sqrtQ[0][0], factor * sqrtQ[0][1]},
		{factor * sqrtQ[1][0], factor * sqrtQ[1][1]},
	}
	offset := [2]float64{meanX + scale*centerU, meanY + scale*centerV}

	var squared float64
	for _, sample := range samples {
		x, y := applyPlanar(matrix, sample.CompX-offset[0], sample.CompY-offset[1])
		radius := math.Hypot(x, y) / (scale * target)
		delta := radius - 1
		squared += delta * delta
	}
	residual := math.Sqrt(squared / float64(len(samples)))
	if !finite(residual) || residual > 0.10 {
		return PlanarModel{}, fmt.Errorf("%w: radial RMS %.1f%% exceeds 10%%", ErrInvalidFit, residual*100)
	}

	return PlanarModel{
		Version: modelVersion, CreatedAt: time.Now().UTC(), Offset: offset,
		Matrix: matrix, ResidualRMS: residual, Condition: condition,
		Samples: len(samples), CoverageBins: bins,
	}, nil
}

func symmetricSqrt(a, b, c, high, low float64) [2][2]float64 {
	angle := 0.5 * math.Atan2(2*b, a-c)
	co, si := math.Cos(angle), math.Sin(angle)
	h, l := math.Sqrt(high), math.Sqrt(low)
	return [2][2]float64{
		{h*co*co + l*si*si, (h - l) * co * si},
		{(h - l) * co * si, h*si*si + l*co*co},
	}
}

func applyPlanar(matrix [2][2]float64, x, y float64) (float64, float64) {
	return matrix[0][0]*x + matrix[0][1]*y,
		matrix[1][0]*x + matrix[1][1]*y
}

func solve5(matrix [5][5]float64, rhs [5]float64) ([5]float64, bool) {
	var augmented [5][6]float64
	for i := range matrix {
		copy(augmented[i][:5], matrix[i][:])
		augmented[i][5] = rhs[i]
	}
	for column := 0; column < 5; column++ {
		pivot := column
		for row := column + 1; row < 5; row++ {
			if math.Abs(augmented[row][column]) > math.Abs(augmented[pivot][column]) {
				pivot = row
			}
		}
		if math.Abs(augmented[pivot][column]) < 1e-12 {
			return [5]float64{}, false
		}
		augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
		divisor := augmented[column][column]
		for j := column; j < 6; j++ {
			augmented[column][j] /= divisor
		}
		for row := 0; row < 5; row++ {
			if row == column {
				continue
			}
			factor := augmented[row][column]
			for j := column; j < 6; j++ {
				augmented[row][j] -= factor * augmented[column][j]
			}
		}
	}
	var result [5]float64
	for i := range result {
		result[i] = augmented[i][5]
	}
	return result, true
}

func LoadModel(path string) (PlanarModel, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return PlanarModel{}, err
	}
	var model PlanarModel
	if err := json.Unmarshal(data, &model); err != nil {
		return PlanarModel{}, err
	}
	if err := validateModel(model); err != nil {
		return PlanarModel{}, err
	}
	return model, nil
}

func SaveModel(path string, model PlanarModel) error {
	if err := validateModel(model); err != nil {
		return err
	}
	data, err := json.MarshalIndent(model, "", "  ")
	if err != nil {
		return err
	}
	data = append(data, '\n')
	return writeAtomic(path, data)
}

func writeAtomic(path string, data []byte) error {
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	tmp, err := os.CreateTemp(filepath.Dir(path), ".motion-calibration-*.tmp")
	if err != nil {
		return err
	}
	name := tmp.Name()
	ok := false
	defer func() {
		_ = tmp.Close()
		if !ok {
			_ = os.Remove(name)
		}
	}()
	if _, err := tmp.Write(data); err != nil {
		return err
	}
	if err := tmp.Sync(); err != nil {
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	if err := os.Rename(name, path); err != nil {
		return err
	}
	dir, err := os.Open(filepath.Dir(path))
	if err != nil {
		return err
	}
	if err := dir.Sync(); err != nil {
		_ = dir.Close()
		return err
	}
	if err := dir.Close(); err != nil {
		return err
	}
	ok = true
	return nil
}

func validateModel(model PlanarModel) error {
	m := model.Matrix
	det := m[0][0]*m[1][1] - m[0][1]*m[1][0]
	trace := m[0][0] + m[1][1]
	disc := math.Sqrt(math.Max(0, (m[0][0]-m[1][1])*(m[0][0]-m[1][1])+4*m[0][1]*m[0][1]))
	high, low := (trace+disc)/2, (trace-disc)/2
	matrixCondition := math.Inf(1)
	if low > 0 {
		matrixCondition = high / low
	}
	valid := model.Version == modelVersion &&
		finite(model.Offset[0]) && finite(model.Offset[1]) &&
		finite(m[0][0]) && finite(m[0][1]) && finite(m[1][0]) && finite(m[1][1]) &&
		math.Abs(m[0][1]-m[1][0]) < 1e-6 && det > 0.5 && det < 2 &&
		matrixCondition <= 4 && model.ResidualRMS >= 0 && model.ResidualRMS <= 0.10 &&
		model.Condition >= 1 && model.Condition <= 4 &&
		model.Samples >= minSamples && model.CoverageBins >= minCoverageBins
	if !valid {
		return fmt.Errorf("%w: invalid persisted model", ErrInvalidFit)
	}
	return nil
}
