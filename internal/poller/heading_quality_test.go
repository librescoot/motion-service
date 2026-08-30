package poller

import "testing"

func TestHeadingQualityWarmsUpThenAcceptsStableField(t *testing.T) {
	var quality headingQuality
	var result headingQualityResult
	for i := 0; i < headingQualityWarmup; i++ {
		result = quality.evaluate(359+float64(i%3), 48+float64(i%2), 20, 0.01, 0.2, true)
		if i+1 < headingQualityWarmup && result.Reason != "warming-up" {
			t.Fatalf("sample %d reason = %q, want warming-up", i, result.Reason)
		}
	}
	if !result.Valid {
		t.Fatalf("stable heading invalid: %s", result.Reason)
	}
}

func TestHeadingQualityRejectsFieldDisturbance(t *testing.T) {
	var quality headingQuality
	for i := 0; i < headingQualityWarmup; i++ {
		quality.evaluate(10, 48, 20, 0.01, 0.1, true)
	}
	result := quality.evaluate(11, 80, 20, 0.01, 0.1, true)
	if result.Reason != "magnetic-field-disturbance" {
		t.Fatalf("reason = %q, want magnetic-field-disturbance", result.Reason)
	}
}

func TestHeadingQualityRejectsUnstableStationaryHeading(t *testing.T) {
	var quality headingQuality
	for _, heading := range []float64{0, 45, 315, 60, 300} {
		quality.evaluate(heading, 48, 20, 0.01, 0.1, true)
	}
	result := quality.evaluate(90, 48, 20, 0.01, 0.1, true)
	if result.Reason != "unstable-heading" {
		t.Fatalf("reason = %q, want unstable-heading", result.Reason)
	}
}

func TestHeadingQualityAllowsChangingHeadingWhileRotating(t *testing.T) {
	var quality headingQuality
	var result headingQualityResult
	for _, heading := range []float64{0, 30, 60, 90, 120} {
		result = quality.evaluate(heading, 48, 20, 0.01, 20, true)
	}
	if !result.Valid {
		t.Fatalf("rotating heading invalid: %s", result.Reason)
	}
}

func TestCircularDispersionHandlesNorthWrap(t *testing.T) {
	if got := circularStdDevDeg([]float64{359, 0, 1}); got > 2 {
		t.Fatalf("dispersion = %.2f, want near zero", got)
	}
}
