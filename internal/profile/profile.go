package profile

import (
	"github.com/librescoot/motion-service/internal/bmx"
)

// Profile is the hardware configuration derived from alarm and power-manager state.
type Profile int

const (
	Idle Profile = iota

	ArmedAwake

	// Stricter than awake mode so ambient vibration cannot wake a powered-down MDB.
	ArmedHibernation

	Level1

	Waiting
)

func (p Profile) String() string {
	switch p {
	case Idle:
		return "idle"
	case ArmedAwake:
		return "armed-awake"
	case ArmedHibernation:
		return "armed-hibernation"
	case Level1:
		return "level1"
	case Waiting:
		return "waiting"
	default:
		return "unknown"
	}
}

type Spec struct {
	Sensor       bmx.SensorConfig
	InterruptPin bmx.InterruptPin

	// Idle configures the engine but must not assert a physical interrupt.
	EnableInterrupt bool
}

// Configs contains register values and their physical timing/threshold contract.
func Configs(p Profile) Spec {
	switch p {
	case Idle:

		return Spec{
			Sensor:          bmx.SensorConfig{Mode: bmx.InterruptModeSlowMotion, Bandwidth: bmx.ACCEL_BW_7_81HZ, Threshold: 0x14, Duration: 0x02},
			InterruptPin:    bmx.InterruptPinNone,
			EnableInterrupt: false,
		}
	case ArmedAwake:
		// Any-motion: 31.25 Hz, about 23 mg, four samples (~64 ms).
		return Spec{
			Sensor:          bmx.SensorConfig{Mode: bmx.InterruptModeAnyMotion, Bandwidth: bmx.ACCEL_BW_31_25HZ, Threshold: 0x06, Duration: 0x03},
			InterruptPin:    bmx.InterruptPinBoth,
			EnableInterrupt: true,
		}
	case ArmedHibernation:
		// Same cadence, but about 31 mg to reject urban vibration.
		return Spec{
			Sensor:          bmx.SensorConfig{Mode: bmx.InterruptModeAnyMotion, Bandwidth: bmx.ACCEL_BW_31_25HZ, Threshold: 0x08, Duration: 0x03},
			InterruptPin:    bmx.InterruptPinBoth,
			EnableInterrupt: true,
		}
	case Level1:

		return Spec{
			Sensor:          bmx.SensorConfig{Mode: bmx.InterruptModeSlowMotion, Bandwidth: bmx.ACCEL_BW_15_63HZ, Threshold: 0x08, Duration: 0x03},
			InterruptPin:    bmx.InterruptPinBoth,
			EnableInterrupt: true,
		}
	case Waiting:
		// INT1 latches slow motion for the poller during the L2 waiting window.
		return Spec{
			Sensor:          bmx.SensorConfig{Mode: bmx.InterruptModeSlowMotion, Bandwidth: bmx.ACCEL_BW_7_81HZ, Threshold: 0x06, Duration: 0x03},
			InterruptPin:    bmx.InterruptPinINT1,
			EnableInterrupt: true,
		}
	default:
		return Configs(Idle)
	}
}

// Derive switches armed mode before hibernation; the profile must reach the
// registers before power-manager completes the transition.
func Derive(alarmStatus, pmState string) Profile {
	switch alarmStatus {
	case "armed":
		if isHibernationImminent(pmState) {
			return ArmedHibernation
		}
		return ArmedAwake
	case "level-1-triggered":
		return Level1
	case "level-2-triggered":
		return Waiting
	default:

		return Idle
	}
}

func isHibernationImminent(pmState string) bool {
	switch pmState {
	case "hibernating-imminent",
		"hibernating-manual-imminent",
		"hibernating-timer-imminent",
		"hibernating",
		"hibernating-manual",
		"hibernating-timer":
		return true
	}
	return false
}
