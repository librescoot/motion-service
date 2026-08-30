package bmx

import (
	"math"
	"testing"
)

func TestHeadingCardinalDirections(t *testing.T) {
	mag := &Magnetometer{calibration: Calibration{}}
	tests := []struct {
		name       string
		x, y, want float64
	}{
		{"north", 1, 0, 0},
		{"east", 0, -1, 90},
		{"south", -1, 0, 180},
		{"west", 0, 1, 270},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			got := mag.HeadingFromVector(tc.x, tc.y, 0, 0, 0)
			assertHeadingNear(t, got, tc.want, 1e-9)
		})
	}
}

func TestTiltCompensationReducesToXYHeadingAtLevel(t *testing.T) {
	mag := &Magnetometer{calibration: Calibration{}}
	withoutTilt := mag.HeadingFromVector(12, -7, 31, math.NaN(), math.NaN())
	level := mag.HeadingFromVector(12, -7, 31, 0, 0)
	assertHeadingNear(t, level, withoutTilt, 1e-9)
}

func TestTiltCompensationPreservesHeadingThroughRoll(t *testing.T) {
	mag := &Magnetometer{calibration: Calibration{}}
	const roll = 22.0 * math.Pi / 180.0
	const horizontalX = 20.0
	const horizontalY = -15.0
	const vertical = 42.0

	// Invert the roll part of HeadingFromVector's leveling transform.
	bodyX := horizontalX
	bodyY := horizontalY*math.Cos(roll) + vertical*math.Sin(roll)
	bodyZ := -horizontalY*math.Sin(roll) + vertical*math.Cos(roll)

	got := mag.HeadingFromVector(bodyX, bodyY, bodyZ, roll, 0)
	want := mag.HeadingFromVector(horizontalX, horizontalY, vertical, 0, 0)
	assertHeadingNear(t, got, want, 1e-9)
}

func assertHeadingNear(t *testing.T, got, want, tolerance float64) {
	t.Helper()
	delta := math.Mod(got-want+540, 360) - 180
	if math.Abs(delta) > tolerance {
		t.Fatalf("heading = %.12f, want %.12f (delta %.12f)", got, want, delta)
	}
}
