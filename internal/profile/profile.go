package profile

import (
	"github.com/librescoot/motion-service/internal/bmx"
)

// Profile names a chip configuration tied to alarm-service's FSM state.
// The mapping from (alarm.status, power-manager.state) → Profile lives in
// Derive; the actual register configs live in Configs.
type Profile int

const (
	// Idle is the default — chip running, motion engines off, no INT routing.
	// Used while the alarm is disabled / disarmed / in seatbox-access.
	Idle Profile = iota

	// ArmedAwake is used while the vehicle is parked with the alarm armed
	// but the system is still running (not hibernating). Any-motion at
	// 31.25 Hz with a permissive threshold catches contact while still
	// rejecting urban environmental noise.
	ArmedAwake

	// ArmedHibernation is programmed just before pm-service suspends the
	// system. Same engine as ArmedAwake but stricter so urban vibration
	// doesn't wake the MDB. Critical: must be in registers before suspend.
	ArmedHibernation

	// Level1 is the L1-triggered state — slow-motion at 15.63 Hz. Confirms
	// deliberate push/tilt over ~256 ms.
	Level1

	// Waiting is the L2-waiting state — slow-motion at 7.81 Hz. Conservative
	// re-trigger threshold for ongoing manipulation.
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

// Spec is the full chip configuration for a profile.
type Spec struct {
	Sensor       bmx.SensorConfig
	InterruptPin bmx.InterruptPin
	// EnableInterrupt is false for profiles where the engine should be
	// configured but the INT pin must not assert (Idle).
	EnableInterrupt bool
}

// Configs returns the Spec for a profile. Register values are the same
// ones alarm-service has been running in production — see
// alarm-service/internal/fsm/state_machine.go for the original definitions.
func Configs(p Profile) Spec {
	switch p {
	case Idle:
		// sensorIdle: low-BW slow-motion engine, interrupt off.
		return Spec{
			Sensor:          bmx.SensorConfig{Mode: bmx.InterruptModeSlowMotion, Bandwidth: bmx.ACCEL_BW_7_81HZ, Threshold: 0x14, Duration: 0x02},
			InterruptPin:    bmx.InterruptPinNone,
			EnableInterrupt: false,
		}
	case ArmedAwake:
		// sensorArmed: any-motion at 31.25 Hz, threshold ~23 mg, 4 samples (~64 ms).
		return Spec{
			Sensor:          bmx.SensorConfig{Mode: bmx.InterruptModeAnyMotion, Bandwidth: bmx.ACCEL_BW_31_25HZ, Threshold: 0x06, Duration: 0x03},
			InterruptPin:    bmx.InterruptPinBoth,
			EnableInterrupt: true,
		}
	case ArmedHibernation:
		// sensorArmedHibernation: any-motion at 31.25 Hz, stricter ~31 mg threshold.
		return Spec{
			Sensor:          bmx.SensorConfig{Mode: bmx.InterruptModeAnyMotion, Bandwidth: bmx.ACCEL_BW_31_25HZ, Threshold: 0x08, Duration: 0x03},
			InterruptPin:    bmx.InterruptPinBoth,
			EnableInterrupt: true,
		}
	case Level1:
		// sensorLevel1: slow-motion at 15.63 Hz, ~31 mg, 4 samples (~256 ms).
		return Spec{
			Sensor:          bmx.SensorConfig{Mode: bmx.InterruptModeSlowMotion, Bandwidth: bmx.ACCEL_BW_15_63HZ, Threshold: 0x08, Duration: 0x03},
			InterruptPin:    bmx.InterruptPinBoth,
			EnableInterrupt: true,
		}
	case Waiting:
		// sensorWaiting: slow-motion at 7.81 Hz, ~23 mg, 4 samples (~512 ms).
		// Enabled with INT1 (not the nRF) so the slow-motion engine latches
		// into INT_STATUS_0 and the poller can re-trigger L2 on continued
		// motion during the waiting-movement window. Polling relies on latched
		// mode, which is only configured when a pin is set.
		return Spec{
			Sensor:          bmx.SensorConfig{Mode: bmx.InterruptModeSlowMotion, Bandwidth: bmx.ACCEL_BW_7_81HZ, Threshold: 0x06, Duration: 0x03},
			InterruptPin:    bmx.InterruptPinINT1,
			EnableInterrupt: true,
		}
	default:
		return Configs(Idle)
	}
}

// Derive returns the chip profile that should be active given the current
// alarm and pm-service states. Hibernation-imminent power states route the
// "armed" alarm status to the stricter ArmedHibernation profile.
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
		// disabled, disarmed, delay-armed, seatbox-access, any unknown — Idle.
		return Idle
	}
}

// isHibernationImminent reports whether pm-service has signalled it is
// about to suspend the system. The chip must be in armed-hibernation
// profile before any of these states transition into the actual suspend.
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
