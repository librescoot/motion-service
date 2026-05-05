package bmx

// InterruptPin represents which interrupt pin to use
type InterruptPin int

const (
	InterruptPinNone InterruptPin = iota
	InterruptPinINT1
	InterruptPinINT2
	InterruptPinBoth
)

func (p InterruptPin) String() string {
	switch p {
	case InterruptPinNone:
		return "none"
	case InterruptPinINT1:
		return "int1"
	case InterruptPinINT2:
		return "int2"
	case InterruptPinBoth:
		return "both"
	default:
		return "unknown"
	}
}

// ParseInterruptPin parses a string to InterruptPin
func ParseInterruptPin(s string) InterruptPin {
	switch s {
	case "int1":
		return InterruptPinINT1
	case "int2":
		return InterruptPinINT2
	case "both":
		return InterruptPinBoth
	case "none":
		return InterruptPinNone
	default:
		return InterruptPinNone
	}
}

// InterruptMode selects which BMX055 interrupt engine to use.
type InterruptMode int

const (
	// InterruptModeAnyMotion uses the slope/any-motion engine (register 0x16).
	// Fires when |accel[n] - accel[n-2]| exceeds threshold for N consecutive
	// samples. Responsive to brief impacts — suitable for awake-armed
	// alertness detection.
	InterruptModeAnyMotion InterruptMode = iota

	// InterruptModeSlowMotion uses the slow-motion engine (register 0x18).
	// Fires when the slope exceeds threshold for N consecutive samples.
	// Requires sustained movement — suitable for confirming deliberate
	// manipulation in L1 / waiting states.
	InterruptModeSlowMotion
)

func (m InterruptMode) String() string {
	switch m {
	case InterruptModeAnyMotion:
		return "any-motion"
	case InterruptModeSlowMotion:
		return "slow-motion"
	default:
		return "unknown"
	}
}

// SensorConfig is the full hardware configuration for a detection profile.
type SensorConfig struct {
	Mode      InterruptMode
	Bandwidth byte // PMU_BW register value
	Threshold byte // 1 LSB = 3.91 mg in 2g range
	Duration  byte // N = dur+1 consecutive samples must exceed threshold
}

// Sensitivity represents motion detection sensitivity levels
type Sensitivity int

const (
	SensitivityLow Sensitivity = iota
	SensitivityMedium
	SensitivityHigh
)

func (s Sensitivity) String() string {
	switch s {
	case SensitivityLow:
		return "low"
	case SensitivityMedium:
		return "medium"
	case SensitivityHigh:
		return "high"
	default:
		return "unknown"
	}
}

// ParseSensitivity parses a string to Sensitivity
func ParseSensitivity(s string) Sensitivity {
	switch s {
	case "low":
		return SensitivityLow
	case "medium":
		return SensitivityMedium
	case "high":
		return SensitivityHigh
	default:
		return SensitivityMedium
	}
}

// GetThreshold returns the threshold value for a given sensitivity
func (s Sensitivity) GetThreshold() byte {
	switch s {
	case SensitivityLow:
		return 0x10
	case SensitivityMedium:
		return 0x09
	case SensitivityHigh:
		return 0x08
	default:
		return 0x09
	}
}

// GetDuration returns the duration value for a given sensitivity
func (s Sensitivity) GetDuration() byte {
	return 0x01
}