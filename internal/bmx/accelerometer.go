package bmx

import (
	"fmt"
	"math"
)

// Accelerometer represents the BMX055 accelerometer
type Accelerometer struct {
	*i2cDevice
}

// NewAccelerometer creates and initializes the accelerometer
func NewAccelerometer(bus string) (*Accelerometer, error) {
	dev, err := openI2C(bus, BMX055_ACCEL_ADDR)
	if err != nil {
		return nil, err
	}
	dev.name = "Accelerometer"

	accel := &Accelerometer{i2cDevice: dev}

	chipID, err := accel.ReadByteData(ACCEL_CHIP_ID_REG)
	if err != nil {
		accel.Close()
		return nil, fmt.Errorf("failed to read accelerometer chip ID: %w", err)
	}

	if chipID != 0xFA && chipID != 0xFB {
		accel.Close()
		return nil, fmt.Errorf("invalid accelerometer chip ID: 0x%02X (expected 0xFA or 0xFB)", chipID)
	}

	if err := accel.WriteByteData(ACCEL_PMU_LPW, 0x00); err != nil {
		accel.Close()
		return nil, fmt.Errorf("failed to set accelerometer power mode: %w", err)
	}

	return accel, nil
}

// ReadData reads raw acceleration data (12-bit) in one block transaction.
// The accel data registers (0x02..0x07) auto-increment, so we pull all
// six bytes with a single SMBus I2C_BLOCK ioctl instead of six byte reads.
func (a *Accelerometer) ReadData() (x, y, z int16, err error) {
	buf, err := a.ReadBlockData(ACCEL_ACCD_X_LSB_REG, 6)
	if err != nil {
		return 0, 0, 0, err
	}
	if len(buf) != 6 {
		return 0, 0, 0, fmt.Errorf("accel ReadData: short read (%d bytes)", len(buf))
	}

	// 12-bit signed; the bottom 4 bits of each LSB byte are status/sticky
	// flags, and the arithmetic shift-right by 4 sign-extends correctly.
	x = (int16(buf[1])<<8 | int16(buf[0])) >> 4
	y = (int16(buf[3])<<8 | int16(buf[2])) >> 4
	z = (int16(buf[5])<<8 | int16(buf[4])) >> 4
	return x, y, z, nil
}

// ReadDataInG reads acceleration data converted to g-force in sensor frame.
func (a *Accelerometer) ReadDataInG() (x, y, z, magnitude float64, err error) {
	rawX, rawY, rawZ, err := a.ReadData()
	if err != nil {
		return 0, 0, 0, 0, err
	}

	const scale = 1024.0
	x = float64(rawX) / scale
	y = float64(rawY) / scale
	z = float64(rawZ) / scale
	magnitude = math.Sqrt(x*x + y*y + z*z)

	return x, y, z, magnitude, nil
}

// ReadDataInGVehicleFrame reads acceleration in g, transformed into the
// vehicle frame described by the supplied Orientation. Magnitude is
// frame-invariant; computed from the vehicle-frame components.
func (a *Accelerometer) ReadDataInGVehicleFrame(o Orientation) (vx, vy, vz, magnitude float64, err error) {
	sx, sy, sz, _, err := a.ReadDataInG()
	if err != nil {
		return 0, 0, 0, 0, err
	}
	vx, vy, vz = o.Apply(sx, sy, sz)
	magnitude = math.Sqrt(vx*vx + vy*vy + vz*vz)
	return vx, vy, vz, magnitude, nil
}

// ConfigureSlowNoMotion configures slow/no-motion detection.
// Register 0x27 is shared: slo_no_mot_dur occupies bits[7:2], slope_dur
// occupies bits[1:0]. The duration parameter is the logical slo_no_mot_dur
// value (0-63) and is shifted into bits[7:2]; bits[1:0] (slope_dur) are
// preserved via read-modify-write so the any-motion engine isn't disturbed.
func (a *Accelerometer) ConfigureSlowNoMotion(threshold, duration byte) error {
	if err := a.WriteByteData(ACCEL_SLO_NO_MOT_THRESHOLD, threshold); err != nil {
		return fmt.Errorf("failed to set slow/no-motion threshold: %w", err)
	}

	existing, err := a.ReadByteData(ACCEL_SLO_NO_MOT_DURATION)
	if err != nil {
		return fmt.Errorf("failed to read duration register: %w", err)
	}
	val := (existing & 0x03) | (duration << 2)
	if err := a.WriteByteData(ACCEL_SLO_NO_MOT_DURATION, val); err != nil {
		return fmt.Errorf("failed to set slow/no-motion duration: %w", err)
	}

	return nil
}

// ConfigureInterruptPin configures the interrupt pin behavior
func (a *Accelerometer) ConfigureInterruptPin(useInt2 bool, latched bool) error {
	outCtrl, err := a.ReadByteData(ACCEL_INT_OUT_CTRL)
	if err != nil {
		return fmt.Errorf("failed to read interrupt output control: %w", err)
	}

	if useInt2 {
		outCtrl |= ACCEL_INT2_ACTIVE_HIGH
		outCtrl &^= ACCEL_INT2_OPEN_DRAIN
	} else {
		outCtrl |= ACCEL_INT1_ACTIVE_HIGH
		outCtrl &^= ACCEL_INT1_OPEN_DRAIN
	}

	if err := a.WriteByteData(ACCEL_INT_OUT_CTRL, outCtrl); err != nil {
		return fmt.Errorf("failed to write interrupt output control: %w", err)
	}

	latchMode := byte(ACCEL_INT_NON_LATCHED)
	if latched {
		latchMode = byte(ACCEL_INT_LATCHED)
	}

	if err := a.WriteByteData(ACCEL_INT_LATCH, latchMode); err != nil {
		return fmt.Errorf("failed to write interrupt latch: %w", err)
	}

	return nil
}

// MapInterruptToPin maps slow/no-motion interrupt to INT1 or INT2
func (a *Accelerometer) MapInterruptToPin(useInt2 bool) error {
	if useInt2 {
		if err := a.WriteByteData(ACCEL_INT_MAP_2, ACCEL_INT2_MAP_SLOW_NO_MOTION); err != nil {
			return fmt.Errorf("failed to map interrupt to INT2: %w", err)
		}
	} else {
		if err := a.WriteByteData(ACCEL_INT_MAP_0, ACCEL_INT1_MAP_SLOW_NO_MOTION); err != nil {
			return fmt.Errorf("failed to map interrupt to INT1: %w", err)
		}
	}
	return nil
}

// DisableInterruptMapping disables interrupt mapping to any pin
func (a *Accelerometer) DisableInterruptMapping() error {
	if err := a.WriteByteData(ACCEL_INT_MAP_0, 0x00); err != nil {
		return fmt.Errorf("failed to clear INT1 mapping: %w", err)
	}
	if err := a.WriteByteData(ACCEL_INT_MAP_2, 0x00); err != nil {
		return fmt.Errorf("failed to clear INT2 mapping: %w", err)
	}
	return nil
}

// EnableSlowNoMotionInterrupt enables slow-motion interrupts on X/Y/Z axes
func (a *Accelerometer) EnableSlowNoMotionInterrupt(slowMotion bool) error {
	intEn := byte(ACCEL_INT_EN_SLOW_NO_MOTION_X | ACCEL_INT_EN_SLOW_NO_MOTION_Y | ACCEL_INT_EN_SLOW_NO_MOTION_Z)

	if !slowMotion {
		intEn |= ACCEL_INT_EN_SLOW_NO_MOTION_SEL
	}

	if err := a.WriteByteData(ACCEL_INT_EN_2, intEn); err != nil {
		return fmt.Errorf("failed to enable slow/no-motion interrupt: %w", err)
	}

	return nil
}

// DisableSlowNoMotionInterrupt disables slow/no-motion interrupts
func (a *Accelerometer) DisableSlowNoMotionInterrupt() error {
	if err := a.WriteByteData(ACCEL_INT_EN_2, 0x00); err != nil {
		return fmt.Errorf("failed to disable slow/no-motion interrupt: %w", err)
	}
	return nil
}

// GetInterruptStatus reads and checks if slow/no-motion interrupt occurred
func (a *Accelerometer) GetInterruptStatus() (bool, error) {
	status, err := a.ReadByteData(ACCEL_INT_STATUS_0)
	if err != nil {
		return false, fmt.Errorf("failed to read interrupt status: %w", err)
	}

	return (status & ACCEL_INT_STATUS_SLOW_NO_MOT) != 0, nil
}

// ClearLatchedInterrupt clears a latched interrupt while preserving latch
// mode. Register 0x21 holds reset_int (bit 7, write-only) and latch_int
// (bits 3:0). Writing 0x80 alone would zero latch_int, dropping the chip
// into non-latched mode — subsequent edges would auto-clear before the
// poller could see them.
func (a *Accelerometer) ClearLatchedInterrupt() error {
	if err := a.WriteByteData(ACCEL_INT_RST_LATCH, 0x80|ACCEL_INT_LATCHED); err != nil {
		return fmt.Errorf("failed to clear latched interrupt: %w", err)
	}
	return nil
}

// ConfigureInterruptPins configures one or both interrupt pins (active-high,
// push-pull) and the chip-wide latch mode.
func (a *Accelerometer) ConfigureInterruptPins(pin InterruptPin, latched bool) error {
	outCtrl, err := a.ReadByteData(ACCEL_INT_OUT_CTRL)
	if err != nil {
		return fmt.Errorf("failed to read interrupt output control: %w", err)
	}

	if pin == InterruptPinINT1 || pin == InterruptPinBoth {
		outCtrl |= ACCEL_INT1_ACTIVE_HIGH
		outCtrl &^= ACCEL_INT1_OPEN_DRAIN
	}
	if pin == InterruptPinINT2 || pin == InterruptPinBoth {
		outCtrl |= ACCEL_INT2_ACTIVE_HIGH
		outCtrl &^= ACCEL_INT2_OPEN_DRAIN
	}

	if err := a.WriteByteData(ACCEL_INT_OUT_CTRL, outCtrl); err != nil {
		return fmt.Errorf("failed to write interrupt output control: %w", err)
	}

	latchMode := byte(ACCEL_INT_NON_LATCHED)
	if latched {
		latchMode = byte(ACCEL_INT_LATCHED)
	}
	if err := a.WriteByteData(ACCEL_INT_LATCH, latchMode); err != nil {
		return fmt.Errorf("failed to write interrupt latch: %w", err)
	}
	return nil
}

// MapInterruptToPins maps the slow/no-motion interrupt to one or both pins.
func (a *Accelerometer) MapInterruptToPins(pin InterruptPin) error {
	if pin == InterruptPinINT1 || pin == InterruptPinBoth {
		if err := a.WriteByteData(ACCEL_INT_MAP_0, ACCEL_INT1_MAP_SLOW_NO_MOTION); err != nil {
			return fmt.Errorf("failed to map slow-motion to INT1: %w", err)
		}
	}
	if pin == InterruptPinINT2 || pin == InterruptPinBoth {
		if err := a.WriteByteData(ACCEL_INT_MAP_2, ACCEL_INT2_MAP_SLOW_NO_MOTION); err != nil {
			return fmt.Errorf("failed to map slow-motion to INT2: %w", err)
		}
	}
	return nil
}

// EnableAnyMotionInterrupt enables the slope/any-motion engine on all axes.
// duration writes bits[1:0] of register 0x27 (shared with slo_no_mot_dur in
// bits[7:2]); existing slo_no_mot_dur bits are preserved.
func (a *Accelerometer) EnableAnyMotionInterrupt(threshold, duration byte) error {
	existing, err := a.ReadByteData(ACCEL_SLOPE_DURATION)
	if err != nil {
		return fmt.Errorf("failed to read slope duration register: %w", err)
	}
	val := (existing &^ 0x03) | (duration & 0x03)
	if err := a.WriteByteData(ACCEL_SLOPE_DURATION, val); err != nil {
		return fmt.Errorf("failed to set slope duration: %w", err)
	}
	if err := a.WriteByteData(ACCEL_SLOPE_THRESHOLD, threshold); err != nil {
		return fmt.Errorf("failed to set slope threshold: %w", err)
	}
	intEn := byte(ACCEL_INT_EN_SLOPE_X | ACCEL_INT_EN_SLOPE_Y | ACCEL_INT_EN_SLOPE_Z)
	if err := a.WriteByteData(ACCEL_INT_EN_0, intEn); err != nil {
		return fmt.Errorf("failed to enable any-motion interrupt: %w", err)
	}
	return nil
}

// DisableAnyMotionInterrupt disables the slope/any-motion engine.
func (a *Accelerometer) DisableAnyMotionInterrupt() error {
	if err := a.WriteByteData(ACCEL_INT_EN_0, 0x00); err != nil {
		return fmt.Errorf("failed to disable any-motion interrupt: %w", err)
	}
	return nil
}

// MapAnyMotionToPins maps the slope/any-motion interrupt to the specified
// pins. OR'd into the existing INT_MAP_0/INT_MAP_2 contents so it doesn't
// clobber a slow-motion mapping configured separately.
func (a *Accelerometer) MapAnyMotionToPins(pin InterruptPin) error {
	if pin == InterruptPinINT1 || pin == InterruptPinBoth {
		existing, err := a.ReadByteData(ACCEL_INT_MAP_0)
		if err != nil {
			return fmt.Errorf("failed to read INT_MAP_0: %w", err)
		}
		if err := a.WriteByteData(ACCEL_INT_MAP_0, existing|ACCEL_INT1_MAP_SLOPE); err != nil {
			return fmt.Errorf("failed to map any-motion to INT1: %w", err)
		}
	}
	if pin == InterruptPinINT2 || pin == InterruptPinBoth {
		existing, err := a.ReadByteData(ACCEL_INT_MAP_2)
		if err != nil {
			return fmt.Errorf("failed to read INT_MAP_2: %w", err)
		}
		if err := a.WriteByteData(ACCEL_INT_MAP_2, existing|ACCEL_INT2_MAP_SLOPE); err != nil {
			return fmt.Errorf("failed to map any-motion to INT2: %w", err)
		}
	}
	return nil
}

// GetAnyMotionInterruptStatus reads INT_STATUS_0 and returns true if the
// slope/any-motion bit is set.
func (a *Accelerometer) GetAnyMotionInterruptStatus() (bool, error) {
	status, err := a.ReadByteData(ACCEL_INT_STATUS_0)
	if err != nil {
		return false, fmt.Errorf("failed to read interrupt status: %w", err)
	}
	return (status & ACCEL_INT_STATUS_SLOPE) != 0, nil
}

// GetMotionInterruptStatus returns true if either the slope/any-motion or
// the slow/no-motion bit is set in INT_STATUS_0. Use this in the poller so
// it works regardless of which engine is currently active.
func (a *Accelerometer) GetMotionInterruptStatus() (bool, error) {
	status, err := a.ReadByteData(ACCEL_INT_STATUS_0)
	if err != nil {
		return false, fmt.Errorf("failed to read interrupt status: %w", err)
	}
	return (status & (ACCEL_INT_STATUS_SLOPE | ACCEL_INT_STATUS_SLOW_NO_MOT)) != 0, nil
}

// SetBandwidth sets the accelerometer low-pass filter bandwidth (PMU_BW
// 0x10). ODR = 2 * BW. Power-on default is 1000 Hz; must be set explicitly
// after every soft reset.
func (a *Accelerometer) SetBandwidth(bw byte) error {
	if err := a.WriteByteData(ACCEL_PMU_BW, bw); err != nil {
		return fmt.Errorf("failed to set bandwidth: %w", err)
	}
	return nil
}

// SoftReset performs a soft reset of the accelerometer
func (a *Accelerometer) SoftReset() error {
	if err := a.WriteByteData(ACCEL_BGW_SOFTRESET, 0xB6); err != nil {
		return fmt.Errorf("failed to soft reset accelerometer: %w", err)
	}
	return nil
}

// SetupMotionDetection configures the accelerometer for motion detection
func (a *Accelerometer) SetupMotionDetection(threshold, duration byte, useInt2, latched bool) error {
	if err := a.ConfigureSlowNoMotion(threshold, duration); err != nil {
		return err
	}

	if err := a.WriteByteData(ACCEL_INT_SRC, 0x00); err != nil {
		return fmt.Errorf("failed to set interrupt source: %w", err)
	}

	if err := a.ConfigureInterruptPin(useInt2, latched); err != nil {
		return err
	}

	if err := a.MapInterruptToPin(useInt2); err != nil {
		return err
	}

	if err := a.EnableSlowNoMotionInterrupt(true); err != nil {
		return err
	}

	return nil
}