package poller

import (
	"math"
	"sort"
)

const (
	headingQualityWindow       = 8
	headingQualityWarmup       = 5
	minMagneticFieldUT         = 10.0
	maxMagneticFieldUT         = 100.0
	minHorizontalFieldUT       = 5.0
	maxFieldResidualFraction   = 0.30
	maxStationaryDispersionDeg = 12.0
	stationaryAngularRateDPS   = 2.0
)

type headingQualityResult struct {
	Valid             bool
	Reason            string
	FieldResidual     float64
	HorizontalFieldUT float64
	DispersionDeg     float64
}

type headingQuality struct {
	fields   []float64
	headings []float64
}

func (q *headingQuality) evaluate(headingDeg, fieldUT, horizontalFieldUT,
	excessG, angularRateDPS float64, tiltCompensated bool) headingQualityResult {
	if finitePositive(fieldUT) {
		q.fields = appendWindow(q.fields, fieldUT, headingQualityWindow)
	}
	if finite(headingDeg) {
		q.headings = appendWindow(q.headings, headingDeg, headingQualityWindow)
	}

	result := headingQualityResult{HorizontalFieldUT: horizontalFieldUT}
	if len(q.fields) >= headingQualityWarmup {
		reference := median(q.fields)
		if reference > 0 {
			result.FieldResidual = math.Abs(fieldUT-reference) / reference
		}
	}
	if len(q.headings) >= headingQualityWarmup {
		result.DispersionDeg = circularStdDevDeg(q.headings)
	}

	switch {
	case !finite(headingDeg) || !finitePositive(fieldUT):
		result.Reason = "non-finite-sample"
	case fieldUT < minMagneticFieldUT || fieldUT > maxMagneticFieldUT:
		result.Reason = "magnetic-field-range"
	case !finite(horizontalFieldUT) || horizontalFieldUT < minHorizontalFieldUT:
		result.Reason = "weak-horizontal-field"
	case !tiltCompensated:
		result.Reason = "dynamic-acceleration"
	case excessG > tiltCompMaxExcessG:
		result.Reason = "dynamic-acceleration"
	case len(q.fields) < headingQualityWarmup || len(q.headings) < headingQualityWarmup:
		result.Reason = "warming-up"
	case result.FieldResidual > maxFieldResidualFraction:
		result.Reason = "magnetic-field-disturbance"
	case angularRateDPS < stationaryAngularRateDPS &&
		result.DispersionDeg > maxStationaryDispersionDeg:
		result.Reason = "unstable-heading"
	default:
		result.Valid = true
	}
	return result
}

func appendWindow(values []float64, value float64, limit int) []float64 {
	if len(values) == limit {
		copy(values, values[1:])
		values[len(values)-1] = value
		return values
	}
	return append(values, value)
}

func median(values []float64) float64 {
	copyOfValues := append([]float64(nil), values...)
	sort.Float64s(copyOfValues)
	middle := len(copyOfValues) / 2
	if len(copyOfValues)%2 == 0 {
		return (copyOfValues[middle-1] + copyOfValues[middle]) / 2
	}
	return copyOfValues[middle]
}

func circularStdDevDeg(values []float64) float64 {
	var sumSin, sumCos float64
	for _, degrees := range values {
		radians := degrees * math.Pi / 180
		sumSin += math.Sin(radians)
		sumCos += math.Cos(radians)
	}
	r := math.Hypot(sumSin, sumCos) / float64(len(values))
	if r <= 0 {
		return 180
	}
	if r > 1 {
		r = 1
	}
	return math.Sqrt(-2*math.Log(r)) * 180 / math.Pi
}

func finite(value float64) bool {
	return !math.IsNaN(value) && !math.IsInf(value, 0)
}

func finitePositive(value float64) bool {
	return finite(value) && value > 0
}
