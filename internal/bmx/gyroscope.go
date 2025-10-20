package bmx

import (
	"fmt"
	"math"
)

// Gyroscope represents the BMX055 gyroscope
type Gyroscope struct {
	*i2cDevice
	biasX float64
	biasY float64
	biasZ float64
}

// NewGyroscope creates and initializes the gyroscope
func NewGyroscope(bus string) (*Gyroscope, error) {
	dev, err := openI2C(bus, BMX055_GYRO_ADDR)
	if err != nil {
		return nil, err
	}
	dev.name = "Gyroscope"

	gyro := &Gyroscope{i2cDevice: dev}

	chipID, err := gyro.ReadByteData(GYRO_CHIP_ID_REG)
	if err != nil {
		gyro.Close()
		return nil, fmt.Errorf("failed to read gyroscope chip ID: %w", err)
	}

	if chipID != 0x0F {
		gyro.Close()
		return nil, fmt.Errorf("invalid gyroscope chip ID: 0x%02X (expected 0x0F)", chipID)
	}

	// Set to normal power mode
	if err := gyro.WriteByteData(GYRO_LPM1, 0x00); err != nil {
		gyro.Close()
		return nil, fmt.Errorf("failed to set gyroscope power mode: %w", err)
	}

	// Set range to ±500°/s (0x02)
	if err := gyro.WriteByteData(GYRO_RANGE, 0x02); err != nil {
		gyro.Close()
		return nil, fmt.Errorf("failed to set gyroscope range: %w", err)
	}

	// Set filter bandwidth to 47 Hz (0x03: 400 Hz ODR, 47 Hz filter)
	if err := gyro.WriteByteData(GYRO_BW, 0x03); err != nil {
		gyro.Close()
		return nil, fmt.Errorf("failed to set gyroscope filter bandwidth: %w", err)
	}

	return gyro, nil
}

// ReadData reads raw gyroscope data (16-bit)
func (g *Gyroscope) ReadData() (x, y, z int16, err error) {
	xLSB, err := g.ReadByteData(GYRO_RATE_X_LSB)
	if err != nil {
		return 0, 0, 0, err
	}
	xMSB, err := g.ReadByteData(GYRO_RATE_X_LSB + 1)
	if err != nil {
		return 0, 0, 0, err
	}

	yLSB, err := g.ReadByteData(GYRO_RATE_Y_LSB)
	if err != nil {
		return 0, 0, 0, err
	}
	yMSB, err := g.ReadByteData(GYRO_RATE_Y_LSB + 1)
	if err != nil {
		return 0, 0, 0, err
	}

	zLSB, err := g.ReadByteData(GYRO_RATE_Z_LSB)
	if err != nil {
		return 0, 0, 0, err
	}
	zMSB, err := g.ReadByteData(GYRO_RATE_Z_LSB + 1)
	if err != nil {
		return 0, 0, 0, err
	}

	x = int16(xMSB)<<8 | int16(xLSB)
	y = int16(yMSB)<<8 | int16(yLSB)
	z = int16(zMSB)<<8 | int16(zLSB)

	return x, y, z, nil
}

// ReadDataInDPS reads gyroscope data converted to degrees/second with bias compensation
func (g *Gyroscope) ReadDataInDPS() (x, y, z, magnitude float64, err error) {
	rawX, rawY, rawZ, err := g.ReadData()
	if err != nil {
		return 0, 0, 0, 0, err
	}

	// Scale for ±500°/s range: 65.6 LSB/°/s
	const scale = 65.6
	x = float64(rawX)/scale - g.biasX
	y = float64(rawY)/scale - g.biasY
	z = float64(rawZ)/scale - g.biasZ
	magnitude = math.Sqrt(x*x + y*y + z*z)

	return x, y, z, magnitude, nil
}

// Calibrate performs gyroscope bias calibration by sampling when stationary
func (g *Gyroscope) Calibrate(samples int) error {
	if samples < 10 {
		samples = 100 // Default to 100 samples
	}

	var sumX, sumY, sumZ float64

	for i := 0; i < samples; i++ {
		rawX, rawY, rawZ, err := g.ReadData()
		if err != nil {
			return fmt.Errorf("failed to read sample %d: %w", i, err)
		}

		const scale = 65.6
		sumX += float64(rawX) / scale
		sumY += float64(rawY) / scale
		sumZ += float64(rawZ) / scale
	}

	g.biasX = sumX / float64(samples)
	g.biasY = sumY / float64(samples)
	g.biasZ = sumZ / float64(samples)

	return nil
}

// SoftReset performs a soft reset of the gyroscope
func (g *Gyroscope) SoftReset() error {
	if err := g.WriteByteData(GYRO_BGW_SOFTRESET, 0xB6); err != nil {
		return fmt.Errorf("failed to soft reset gyroscope: %w", err)
	}
	return nil
}