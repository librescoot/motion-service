package bmx

import (
	"fmt"
	"math"
	"time"
)

type Gyroscope struct {
	*i2cDevice
	biasX float64
	biasY float64
	biasZ float64
}

func NewGyroscope(bus string) (*Gyroscope, error) {
	dev, err := openI2C(bus, BMX055_GYRO_ADDR)
	if err != nil {
		return nil, err
	}
	dev.name = "Gyroscope"

	gyro := &Gyroscope{i2cDevice: dev}

	chipID, err := gyro.ReadByteData(GYRO_CHIP_ID_REG)
	if err != nil {
		_ = gyro.Close()
		return nil, fmt.Errorf("failed to read gyroscope chip ID: %w", err)
	}

	if chipID != 0x0F {
		_ = gyro.Close()
		return nil, fmt.Errorf("invalid gyroscope chip ID: 0x%02X (expected 0x0F)", chipID)
	}

	if err := gyro.Configure(); err != nil {
		_ = gyro.Close()
		return nil, err
	}

	return gyro, nil
}

func (g *Gyroscope) Configure() error {

	if err := g.WriteByteData(GYRO_LPM1, 0x00); err != nil {
		return fmt.Errorf("failed to set gyroscope power mode: %w", err)
	}

	if err := g.WriteByteData(GYRO_RANGE, 0x02); err != nil {
		return fmt.Errorf("failed to set gyroscope range: %w", err)
	}

	if err := g.WriteByteData(GYRO_BW, 0x03); err != nil {
		return fmt.Errorf("failed to set gyroscope filter bandwidth: %w", err)
	}
	return nil
}

func (g *Gyroscope) ReadData() (x, y, z int16, err error) {
	buf, err := g.ReadBlockData(GYRO_RATE_X_LSB, 6)
	if err != nil {
		return 0, 0, 0, err
	}
	if len(buf) != 6 {
		return 0, 0, 0, fmt.Errorf("gyro ReadData: short read (%d bytes)", len(buf))
	}
	x = int16(buf[1])<<8 | int16(buf[0])
	y = int16(buf[3])<<8 | int16(buf[2])
	z = int16(buf[5])<<8 | int16(buf[4])
	return x, y, z, nil
}

func (g *Gyroscope) ReadDataInDPS() (x, y, z, magnitude float64, err error) {
	rawX, rawY, rawZ, err := g.ReadData()
	if err != nil {
		return 0, 0, 0, 0, err
	}

	const scale = 65.6
	x = float64(rawX)/scale - g.biasX
	y = float64(rawY)/scale - g.biasY
	z = float64(rawZ)/scale - g.biasZ
	magnitude = math.Sqrt(x*x + y*y + z*z)

	return x, y, z, magnitude, nil
}

func (g *Gyroscope) ReadDataInDPSVehicleFrame(o Orientation) (vx, vy, vz, magnitude float64, err error) {
	sx, sy, sz, _, err := g.ReadDataInDPS()
	if err != nil {
		return 0, 0, 0, 0, err
	}
	vx, vy, vz = o.Apply(sx, sy, sz)
	magnitude = math.Sqrt(vx*vx + vy*vy + vz*vz)
	return vx, vy, vz, magnitude, nil
}

func (g *Gyroscope) Calibrate(samples int) error {
	if samples < 10 {
		samples = 100
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

func (g *Gyroscope) SoftReset() error {
	writeErr := g.WriteByteData(GYRO_BGW_SOFTRESET, 0xB6)

	var idErr error
	for attempt := 0; attempt < 3; attempt++ {
		time.Sleep(10 * time.Millisecond)
		var chipID byte
		if chipID, idErr = g.ReadByteData(GYRO_CHIP_ID_REG); idErr == nil {
			if chipID == 0x0F {
				return nil
			}
			idErr = fmt.Errorf("invalid gyroscope chip ID: 0x%02X (expected 0x0F)", chipID)
		}
	}
	if writeErr != nil {
		return fmt.Errorf("failed to soft reset gyroscope: %w (chip unresponsive afterwards: %v)", writeErr, idErr)
	}
	return fmt.Errorf("gyroscope unresponsive after soft reset: %w", idErr)
}
