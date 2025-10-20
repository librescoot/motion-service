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

// Magnetometer represents the BMX055 magnetometer
type Magnetometer struct {
	*i2cDevice
	trimData TrimData
}

// NewMagnetometer creates and initializes the magnetometer
func NewMagnetometer(bus string) (*Magnetometer, error) {
	dev, err := openI2C(bus, BMX055_MAG_ADDR)
	if err != nil {
		return nil, err
	}
	dev.name = "Magnetometer"

	mag := &Magnetometer{i2cDevice: dev}

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

	// Set operation mode to Normal (bits 2-1 = 00) and data rate to 30Hz (bits 5-3 = 111)
	// 0x38 = 0b00111000 = 30Hz Normal mode
	if err := mag.WriteByteData(MAG_OPMODE_ODR, 0x38); err != nil {
		mag.Close()
		return nil, fmt.Errorf("failed to set magnetometer operation mode: %w", err)
	}

	// Verify configuration was written
	opmode, err := mag.ReadByteData(MAG_OPMODE_ODR)
	if err != nil {
		fmt.Printf("Warning: failed to read back OPMODE_ODR: %v\n", err)
	} else {
		fmt.Printf("Magnetometer OPMODE_ODR register: 0x%02X\n", opmode)
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

// compensateX applies temperature compensation to X-axis magnetometer data
func (m *Magnetometer) compensateX(magDataX int16, dataRhall uint16) int16 {
	// Check for overflow
	if magDataX == BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL {
		return BMM150_OVERFLOW_OUTPUT
	}

	var processCompX2 int32
	if dataRhall != 0 {
		// Availability of valid data
		processCompX1 := uint16(m.trimData.digXYZ1) << 14
		processCompX2 = int32(processCompX1) / int32(dataRhall)
	} else {
		processCompX2 = 0
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

// compensateY applies temperature compensation to Y-axis magnetometer data
func (m *Magnetometer) compensateY(magDataY int16, dataRhall uint16) int16 {
	// Check for overflow
	if magDataY == BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL {
		return BMM150_OVERFLOW_OUTPUT
	}

	var processCompY2 int32
	if dataRhall != 0 {
		// Availability of valid data
		processCompY1 := uint16(m.trimData.digXYZ1) << 14
		processCompY2 = int32(processCompY1) / int32(dataRhall)
	} else {
		processCompY2 = 0
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

// Hard-iron compensation offsets measured with board installed in scooter
// with batteries, rotating 360 degrees in one spot
const (
	hardIronOffsetX int16 = -441
	hardIronOffsetY int16 = -259
	hardIronOffsetZ int16 = -1164

	// Heading calibration offset for Deep Blue
	// BMX055 mounted on underside of board → 180° offset
	headingCalibrationOffset float64 = 180.0
)

// ReadDataInMicroTesla reads magnetometer data converted to µT with hard-iron compensation
func (m *Magnetometer) ReadDataInMicroTesla() (x, y, z, magnitude float64, err error) {
	rawX, rawY, rawZ, err := m.ReadData()
	if err != nil {
		return 0, 0, 0, 0, err
	}

	// Apply hard-iron compensation
	compensatedX := rawX - hardIronOffsetX
	compensatedY := rawY - hardIronOffsetY
	compensatedZ := rawZ - hardIronOffsetZ

	const scale = 16.0
	x = float64(compensatedX) / scale
	y = float64(compensatedY) / scale
	z = float64(compensatedZ) / scale
	magnitude = math.Sqrt(x*x + y*y + z*z)

	return x, y, z, magnitude, nil
}

// ReadHeading reads magnetometer data and calculates compass heading in degrees (0-360)
func (m *Magnetometer) ReadHeading() (heading float64, err error) {
	rawX, rawY, rawZ, err := m.ReadData()
	if err != nil {
		return 0, err
	}

	// Apply hard-iron compensation (same as 9axis service)
	dx := float64(rawX - hardIronOffsetX)
	dy := float64(rawY - hardIronOffsetY)
	_ = rawZ - hardIronOffsetZ

	// Sensor is at bottom of board, flip around x-axis
	dx = dx * -1.0

	// Calculate heading using atan2
	angleRad := math.Atan2(dy, dx)
	angleDeg := angleRad * 180.0 / math.Pi

	// Normalize to 0-360 range
	if angleDeg < 0.0 {
		angleDeg = 360.0 + angleDeg
	}

	// Apply Deep Blue calibration offset
	angleDeg += headingCalibrationOffset
	if angleDeg >= 360.0 {
		angleDeg -= 360.0
	}

	return angleDeg, nil
}