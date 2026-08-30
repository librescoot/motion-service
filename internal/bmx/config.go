package bmx

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

type InterruptMode int

const (
	InterruptModeAnyMotion InterruptMode = iota

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

type SensorConfig struct {
	Mode      InterruptMode
	Bandwidth byte
	Threshold byte
	Duration  byte
}

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

func (s Sensitivity) GetDuration() byte {
	return 0x01
}
