package poller

import (
	"math"
	"testing"
)

func TestTiltFromSpecificForceLevel(t *testing.T) {
	roll, pitch := tiltFromSpecificForce(0, 0, -1)
	assertAngleNear(t, roll, 0, 1e-12)
	assertAngleNear(t, pitch, 0, 1e-12)
}

func TestTiltFromSpecificForceLeanAndPitch(t *testing.T) {
	const rollWant = 18.0 * math.Pi / 180.0
	roll, pitch := tiltFromSpecificForce(0, -math.Sin(rollWant), -math.Cos(rollWant))
	assertAngleNear(t, roll, rollWant, 1e-12)
	assertAngleNear(t, pitch, 0, 1e-12)

	const pitchWant = -12.0 * math.Pi / 180.0
	roll, pitch = tiltFromSpecificForce(math.Sin(pitchWant), 0, -math.Cos(pitchWant))
	assertAngleNear(t, roll, 0, 1e-12)
	assertAngleNear(t, pitch, pitchWant, 1e-12)
}

func TestCalibrationPolicyFailsClosed(t *testing.T) {
	good := headingQualityResult{Valid: true}
	if got := applyCalibrationPolicy(good, "uncalibrated", 0); got.Valid || got.Reason != "uncalibrated" {
		t.Fatalf("uncalibrated heading accepted: %+v", got)
	}
	if got := applyCalibrationPolicy(good, "calibrated", 15); got.Valid || got.Reason != "excessive_tilt" {
		t.Fatalf("tilted planar heading accepted: %+v", got)
	}
	if got := applyCalibrationPolicy(good, "calibrated", 2); !got.Valid {
		t.Fatalf("calibrated upright heading rejected: %+v", got)
	}
}

func assertAngleNear(t *testing.T, got, want, tolerance float64) {
	t.Helper()
	if math.Abs(got-want) > tolerance {
		t.Fatalf("angle = %.12f, want %.12f", got, want)
	}
}
