package bmx

import (
	"fmt"
	"math"
	"time"
)

// TrimData holds the BMM150 temperature compensation trim values
type TrimData struct {
	digX1   int8
	digY1   int8
	digX2   int8
	digY2   int8
	digZ1   uint16
	digZ2   int16
	digZ3   int16
	digZ4   int16
	digXY1  uint8
	digXY2  int8
	digXYZ1 uint16
}

// Calibration holds runtime-tunable magnetometer calibration.
//
// HardIronOffset is subtracted from the raw 16-bit (post-compensation) value
// before scaling to µT — i.e. the same units the chip reports. Capture by
// rotating the vehicle 360° in place and taking (max+min)/2 per axis.
//
// AxisSign and YawOffsetDeg map sensor-frame µT to vehicle-frame µT used
// for the heading calculation. AxisSign is ±1 per axis; YawOffsetDeg is
// added to the final compass heading. These cover the common cases where
// the BMX055 is mounted upside-down or rotated relative to vehicle forward.
type Calibration struct {
	HardIronOffset [3]int16
	AxisSign       [3]float64
	YawOffsetDeg   float64
}

// DefaultCalibration is the empirical calibration captured on Deep Blue.
// Other vehicles will need their own values — see docs/calibration.md.
var DefaultCalibration = Calibration{
	HardIronOffset: [3]int16{-441, -259, -1164},
	AxisSign:       [3]float64{-1, 1, -1},
	YawOffsetDeg:   180.0,
}

// Magnetometer represents the BMX055 magnetometer
type Magnetometer struct {
	*i2cDevice
	trimData    TrimData
	calibration Calibration
}

// NewMagnetometer creates and initializes the magnetometer
func NewMagnetometer(bus string) (*Magnetometer, error) {
	dev, err := openI2C(bus, BMX055_MAG_ADDR)
	if err != nil {
		return nil, err
	}
	dev.name = "Magnetometer"

	mag := &Magnetometer{i2cDevice: dev, calibration: DefaultCalibration}

	// Enable power control bit (equivalent to bmm050_init power enable)
	if err := mag.WriteByteData(MAG_POWER_CTRL, 0x01); err != nil {
		mag.Close()
		return nil, fmt.Errorf("failed to enable magnetometer power: %w", err)
	}

	time.Sleep(5 * time.Millisecond)

	// Verify chip ID
	chipID, err := mag.ReadByteData(MAG_CHIP_ID_REG)
	if err != nil {
		mag.Close()
		return nil, fmt.Errorf("failed to read magnetometer chip ID: %w", err)
	}

	if chipID != 0x32 {
		mag.Close()
		return nil, fmt.Errorf("invalid magnetometer chip ID: 0x%02X (expected 0x32)", chipID)
	}

	// Apply Regular preset (9 XY reps / 15 Z reps) — datasheet ±2.5° heading
	// accuracy is specified for this preset, not for the power-on default of
	// 1 rep which is loud enough to be unusable as a compass.
	if err := mag.WriteByteData(MAG_REPXY, MAG_REPXY_REGULAR); err != nil {
		mag.Close()
		return nil, fmt.Errorf("failed to set magnetometer REPXY: %w", err)
	}
	if err := mag.WriteByteData(MAG_REPZ, MAG_REPZ_REGULAR); err != nil {
		mag.Close()
		return nil, fmt.Errorf("failed to set magnetometer REPZ: %w", err)
	}

	// Normal mode @ 10 Hz ODR. Datasheet pairs the Regular preset with 10 Hz;
	// running at 30 Hz with only 1 rep (the prior config) gave more samples
	// of noisier data than is useful.
	opmodeOdr := byte(MAG_OPMODE_NORMAL | MAG_ODR_10HZ)
	if err := mag.WriteByteData(MAG_OPMODE_ODR, opmodeOdr); err != nil {
		mag.Close()
		return nil, fmt.Errorf("failed to set magnetometer operation mode: %w", err)
	}

	// Read trim data for temperature compensation
	if err := mag.readTrimData(); err != nil {
		mag.Close()
		return nil, fmt.Errorf("failed to read magnetometer trim data: %w", err)
	}

	return mag, nil
}

// readTrimData reads the trim registers for temperature compensation
func (m *Magnetometer) readTrimData() error {
	var err error

	// Read dig_x1
	val, err := m.ReadByteData(MAG_DIG_X1)
	if err != nil {
		return err
	}
	m.trimData.digX1 = int8(val)

	// Read dig_y1
	val, err = m.ReadByteData(MAG_DIG_Y1)
	if err != nil {
		return err
	}
	m.trimData.digY1 = int8(val)

	// Read dig_x2
	val, err = m.ReadByteData(MAG_DIG_X2)
	if err != nil {
		return err
	}
	m.trimData.digX2 = int8(val)

	// Read dig_y2
	val, err = m.ReadByteData(MAG_DIG_Y2)
	if err != nil {
		return err
	}
	m.trimData.digY2 = int8(val)

	// Read dig_z1 (16-bit)
	lsb, err := m.ReadByteData(MAG_DIG_Z1_LSB)
	if err != nil {
		return err
	}
	msb, err := m.ReadByteData(MAG_DIG_Z1_MSB)
	if err != nil {
		return err
	}
	m.trimData.digZ1 = uint16(msb)<<8 | uint16(lsb)

	// Read dig_z2 (16-bit signed)
	lsb, err = m.ReadByteData(MAG_DIG_Z2_LSB)
	if err != nil {
		return err
	}
	msb, err = m.ReadByteData(MAG_DIG_Z2_MSB)
	if err != nil {
		return err
	}
	m.trimData.digZ2 = int16(uint16(msb)<<8 | uint16(lsb))

	// Read dig_z3 (16-bit signed)
	lsb, err = m.ReadByteData(MAG_DIG_Z3_LSB)
	if err != nil {
		return err
	}
	msb, err = m.ReadByteData(MAG_DIG_Z3_MSB)
	if err != nil {
		return err
	}
	m.trimData.digZ3 = int16(uint16(msb)<<8 | uint16(lsb))

	// Read dig_z4 (16-bit signed)
	lsb, err = m.ReadByteData(MAG_DIG_Z4_LSB)
	if err != nil {
		return err
	}
	msb, err = m.ReadByteData(MAG_DIG_Z4_MSB)
	if err != nil {
		return err
	}
	m.trimData.digZ4 = int16(uint16(msb)<<8 | uint16(lsb))

	// Read dig_xy1
	val, err = m.ReadByteData(MAG_DIG_XY1)
	if err != nil {
		return err
	}
	m.trimData.digXY1 = uint8(val)

	// Read dig_xy2
	val, err = m.ReadByteData(MAG_DIG_XY2)
	if err != nil {
		return err
	}
	m.trimData.digXY2 = int8(val)

	// Read dig_xyz1 (16-bit)
	lsb, err = m.ReadByteData(MAG_DIG_XYZ1_LSB)
	if err != nil {
		return err
	}
	msb, err = m.ReadByteData(MAG_DIG_XYZ1_MSB)
	if err != nil {
		return err
	}
	m.trimData.digXYZ1 = uint16(msb)<<8 | uint16(lsb)

	return nil
}

// readRhall reads the Hall resistance value needed for compensation
func (m *Magnetometer) readRhall() (uint16, error) {
	lsb, err := m.ReadByteData(MAG_RHALL_LSB)
	if err != nil {
		return 0, err
	}
	msb, err := m.ReadByteData(MAG_RHALL_MSB)
	if err != nil {
		return 0, err
	}

	// Combine LSB and MSB, shift right by 2 (14-bit value)
	rhall := (uint16(msb) << 6) | (uint16(lsb) >> 2)
	return rhall, nil
}

// compensateX applies temperature compensation to X-axis magnetometer data.
// Mirrors the Bosch BMM150 reference compensation flow; intermediates that
// shift dig_xyz1 left by 14 must be 32-bit — uint16<<14 truncates the value
// to ~zero and silently breaks compensation.
func (m *Magnetometer) compensateX(magDataX int16, dataRhall uint16) int16 {
	if magDataX == BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL {
		return BMM150_OVERFLOW_OUTPUT
	}

	var processCompX2 int32
	if dataRhall != 0 {
		processCompX1 := uint32(m.trimData.digXYZ1) << 14
		processCompX2 = int32(processCompX1 / uint32(dataRhall))
	}

	retval := int16(processCompX2)
	processCompX3 := uint16(retval) - 16384
	retval = int16(m.trimData.digX2) +
		int16((int32(m.trimData.digX1)*int32(processCompX3))>>15)
	processCompX4 := int32(magDataX) * int32(retval)
	retval = int16(processCompX4 / 4096)
	processCompX5 := int32(retval) * int32(m.trimData.digXY2)
	retval = int16((processCompX5 + 16384) / 32768)
	retval = int16(int32(m.trimData.digXY1)*int32(retval)) + retval
	retval = int16((int32(retval) + 16384) / 32768)

	return retval
}

// compensateY applies temperature compensation to Y-axis magnetometer data.
// Same uint16<<14 overflow fix as compensateX.
func (m *Magnetometer) compensateY(magDataY int16, dataRhall uint16) int16 {
	if magDataY == BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL {
		return BMM150_OVERFLOW_OUTPUT
	}

	var processCompY2 int32
	if dataRhall != 0 {
		processCompY1 := uint32(m.trimData.digXYZ1) << 14
		processCompY2 = int32(processCompY1 / uint32(dataRhall))
	}

	retval := int16(processCompY2)
	processCompY3 := uint16(retval) - 16384
	retval = int16(m.trimData.digY2) +
		int16((int32(m.trimData.digY1)*int32(processCompY3))>>15)
	processCompY4 := int32(magDataY) * int32(retval)
	retval = int16(processCompY4 / 4096)

	return retval
}

// compensateZ applies temperature compensation to Z-axis magnetometer data
func (m *Magnetometer) compensateZ(magDataZ int16, dataRhall uint16) int16 {
	// Check for overflow
	if magDataZ == BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL {
		return BMM150_OVERFLOW_OUTPUT
	}

	// Check for division by zero conditions
	if m.trimData.digZ2 == 0 || m.trimData.digZ1 == 0 ||
		m.trimData.digXYZ1 == 0 || dataRhall == 0 {
		return BMM150_OVERFLOW_OUTPUT
	}

	processCompZ1 := int16(dataRhall) - int16(m.trimData.digXYZ1)
	processCompZ3 := int32(m.trimData.digZ1) * (int32(magDataZ) << 14)
	processCompZ4 := int16((processCompZ3 - (16384 * 16384)) /
		(int32(m.trimData.digZ2) +
			(int32(m.trimData.digZ4)*int32(processCompZ1))/32768))
	retval := int32(processCompZ4) + (int32(m.trimData.digZ2) * 8192)
	retval = retval / 16384

	return int16(retval)
}

// ReadData reads raw magnetometer data and applies temperature compensation
func (m *Magnetometer) ReadData() (x, y, z int16, err error) {
	xLSB, err := m.ReadByteData(MAG_DATAX_LSB)
	if err != nil {
		return 0, 0, 0, err
	}
	xMSB, err := m.ReadByteData(MAG_DATAX_LSB + 1)
	if err != nil {
		return 0, 0, 0, err
	}

	yLSB, err := m.ReadByteData(MAG_DATAY_LSB)
	if err != nil {
		return 0, 0, 0, err
	}
	yMSB, err := m.ReadByteData(MAG_DATAY_LSB + 1)
	if err != nil {
		return 0, 0, 0, err
	}

	zLSB, err := m.ReadByteData(MAG_DATAZ_LSB)
	if err != nil {
		return 0, 0, 0, err
	}
	zMSB, err := m.ReadByteData(MAG_DATAZ_LSB + 1)
	if err != nil {
		return 0, 0, 0, err
	}

	// Read Hall resistance for temperature compensation
	rhall, err := m.readRhall()
	if err != nil {
		return 0, 0, 0, err
	}

	// Extract raw magnetometer values
	xRaw := (uint16(xMSB) << 5) | (uint16(xLSB) >> 3)
	yRaw := (uint16(yMSB) << 5) | (uint16(yLSB) >> 3)

	// Sign extend 13-bit values to 16-bit
	if xRaw&0x1000 != 0 {
		xRaw |= 0xE000
	}
	if yRaw&0x1000 != 0 {
		yRaw |= 0xE000
	}
	xRawSigned := int16(xRaw)
	yRawSigned := int16(yRaw)

	zRaw := (uint16(zMSB) << 7) | (uint16(zLSB) >> 1)

	// Sign extend 15-bit value to 16-bit
	if zRaw&0x4000 != 0 {
		zRaw |= 0x8000
	}
	zRawSigned := int16(zRaw)

	// Apply temperature compensation
	x = m.compensateX(xRawSigned, rhall)
	y = m.compensateY(yRawSigned, rhall)
	z = m.compensateZ(zRawSigned, rhall)

	return x, y, z, nil
}

// SetCalibration replaces the runtime calibration used by ReadDataInMicroTesla
// and the heading calculations. Safe to call at any time.
func (m *Magnetometer) SetCalibration(cal Calibration) {
	m.calibration = cal
}

// Calibration returns the current runtime calibration.
func (m *Magnetometer) Calibration() Calibration {
	return m.calibration
}

// raw LSB → µT scale factor (per Bosch BMM150 reference).
const magScaleUT = 16.0

// ReadDataInMicroTesla reads the magnetometer in vehicle-frame µT with
// hard-iron and axis-sign calibration applied. The Z axis is included so
// that consumers can do their own tilt compensation if needed.
func (m *Magnetometer) ReadDataInMicroTesla() (x, y, z, magnitude float64, err error) {
	rawX, rawY, rawZ, err := m.ReadData()
	if err != nil {
		return 0, 0, 0, 0, err
	}

	cal := m.calibration
	x = m.calibration.AxisSign[0] * float64(rawX-cal.HardIronOffset[0]) / magScaleUT
	y = m.calibration.AxisSign[1] * float64(rawY-cal.HardIronOffset[1]) / magScaleUT
	z = m.calibration.AxisSign[2] * float64(rawZ-cal.HardIronOffset[2]) / magScaleUT
	magnitude = math.Sqrt(x*x + y*y + z*z)

	return x, y, z, magnitude, nil
}

// HeadingFromVector computes a compass heading (0-360°, 0=North, 90=East)
// from a vehicle-frame mag vector with optional roll/pitch tilt compensation.
//
// Pass roll/pitch in radians. Pass NaN for either to skip tilt compensation
// (X/Y-only heading, valid only when the sensor is near horizontal).
func (m *Magnetometer) HeadingFromVector(magX, magY, magZ, rollRad, pitchRad float64) float64 {
	var bx, by float64
	if math.IsNaN(rollRad) || math.IsNaN(pitchRad) {
		bx = magX
		by = magY
	} else {
		// Project the magnetic vector onto the horizontal plane.
		// Standard NED-frame tilt compensation; signs follow vehicle frame
		// (X forward, Y left, Z up) which the AxisSign step produces.
		sr, cr := math.Sincos(rollRad)
		sp, cp := math.Sincos(pitchRad)
		bx = magX*cp + magY*sp*sr + magZ*sp*cr
		by = magY*cr - magZ*sr
	}

	angleDeg := math.Atan2(-by, bx) * 180.0 / math.Pi
	angleDeg += m.calibration.YawOffsetDeg
	angleDeg = math.Mod(angleDeg+360.0, 360.0)
	return angleDeg
}

// ReadHeading reads magnetometer data and returns a non-tilt-compensated
// compass heading. Retained for callers that don't have an accelerometer
// available; prefer ComputeTiltCompensatedHeading where possible.
func (m *Magnetometer) ReadHeading() (heading float64, err error) {
	x, y, z, _, err := m.ReadDataInMicroTesla()
	if err != nil {
		return 0, err
	}
	return m.HeadingFromVector(x, y, z, math.NaN(), math.NaN()), nil
}