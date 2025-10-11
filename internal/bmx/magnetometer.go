package bmx

import (
	"fmt"
	"math"
	"time"
)

// Magnetometer represents the BMX055 magnetometer
type Magnetometer struct {
	*i2cDevice
}

// NewMagnetometer creates and initializes the magnetometer
func NewMagnetometer(bus string) (*Magnetometer, error) {
	dev, err := openI2C(bus, BMX055_MAG_ADDR)
	if err != nil {
		return nil, err
	}
	dev.name = "Magnetometer"

	mag := &Magnetometer{i2cDevice: dev}

	if err := mag.WriteByteData(MAG_POWER_CTRL, 0x01); err != nil {
		mag.Close()
		return nil, fmt.Errorf("failed to enable magnetometer power: %w", err)
	}

	time.Sleep(5 * time.Millisecond)

	chipID, err := mag.ReadByteData(MAG_CHIP_ID_REG)
	if err != nil {
		mag.Close()
		return nil, fmt.Errorf("failed to read magnetometer chip ID: %w", err)
	}

	if chipID != 0x32 {
		mag.Close()
		return nil, fmt.Errorf("invalid magnetometer chip ID: 0x%02X (expected 0x32)", chipID)
	}

	if err := mag.WriteByteData(MAG_OPMODE_ODR, 0x00); err != nil {
		mag.Close()
		return nil, fmt.Errorf("failed to set magnetometer operation mode: %w", err)
	}

	return mag, nil
}

// ReadData reads raw magnetometer data
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

	xRaw := (uint16(xMSB) << 5) | (uint16(xLSB) >> 3)
	yRaw := (uint16(yMSB) << 5) | (uint16(yLSB) >> 3)

	if xRaw&0x1000 != 0 {
		xRaw |= 0xE000
	}
	if yRaw&0x1000 != 0 {
		yRaw |= 0xE000
	}
	x = int16(xRaw)
	y = int16(yRaw)

	zRaw := (uint16(zMSB) << 7) | (uint16(zLSB) >> 1)

	if zRaw&0x4000 != 0 {
		zRaw |= 0x8000
	}
	z = int16(zRaw)

	return x, y, z, nil
}

// ReadDataInMicroTesla reads magnetometer data converted to µT
func (m *Magnetometer) ReadDataInMicroTesla() (x, y, z, magnitude float64, err error) {
	rawX, rawY, rawZ, err := m.ReadData()
	if err != nil {
		return 0, 0, 0, 0, err
	}

	const scale = 16.0
	x = float64(rawX) / scale
	y = float64(rawY) / scale
	z = float64(rawZ) / scale
	magnitude = math.Sqrt(x*x + y*y + z*z)

	return x, y, z, magnitude, nil
}