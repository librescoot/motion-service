package bmx

import (
	"fmt"
	"math"
	"sync"
	"time"
)

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

type Calibration struct {
	HardIronOffset [3]float64
	SoftIronXY     [2][2]float64
	Orientation    Orientation
	YawOffsetDeg   float64
	State          string
}

// DefaultCalibration is only the mechanical sensor-to-vehicle axis mapping.
// Magnetic correction is device-specific and loaded from /data.
var DefaultCalibration = Calibration{
	SoftIronXY: [2][2]float64{{1, 0}, {0, 1}},
	Orientation: Orientation{

		AxisOrder: [3]int{1, 0, 2},
		AxisSign:  [3]float64{-1, 1, 1},
	},
	State: "uncalibrated",
}

type Magnetometer struct {
	*i2cDevice
	trimData            TrimData
	calibrationMu       sync.RWMutex
	calibration         Calibration
	calibrationRevision uint64
}

// MagSample keeps the raw ADC values and the factory-trim-compensated values
// from one register burst together. Calibration fitting must use Comp*, since
// those are the units consumed by the runtime calibration pipeline.
type MagSample struct {
	RawX, RawY, RawZ    int16
	RHall               uint16
	DataReady           bool
	CompX, CompY, CompZ int16
}

func NewMagnetometer(bus string) (*Magnetometer, error) {
	dev, err := openI2C(bus, BMX055_MAG_ADDR)
	if err != nil {
		return nil, err
	}
	dev.name = "Magnetometer"

	mag := &Magnetometer{i2cDevice: dev, calibration: DefaultCalibration}

	if err := mag.WriteByteData(MAG_POWER_CTRL, 0x01); err != nil {
		_ = mag.Close()
		return nil, fmt.Errorf("failed to enable magnetometer power: %w", err)
	}

	// BMM150 needs a power-on delay before its chip ID is readable.
	time.Sleep(5 * time.Millisecond)

	chipID, err := mag.ReadByteData(MAG_CHIP_ID_REG)
	if err != nil {
		_ = mag.Close()
		return nil, fmt.Errorf("failed to read magnetometer chip ID: %w", err)
	}

	if chipID != 0x32 {
		_ = mag.Close()
		return nil, fmt.Errorf("invalid magnetometer chip ID: 0x%02X (expected 0x32)", chipID)
	}

	if err := mag.WriteByteData(MAG_REPXY, MAG_REPXY_REGULAR); err != nil {
		_ = mag.Close()
		return nil, fmt.Errorf("failed to set magnetometer REPXY: %w", err)
	}
	if err := mag.WriteByteData(MAG_REPZ, MAG_REPZ_REGULAR); err != nil {
		_ = mag.Close()
		return nil, fmt.Errorf("failed to set magnetometer REPZ: %w", err)
	}

	opmodeOdr := byte(MAG_OPMODE_NORMAL | MAG_ODR_10HZ)
	if err := mag.WriteByteData(MAG_OPMODE_ODR, opmodeOdr); err != nil {
		_ = mag.Close()
		return nil, fmt.Errorf("failed to set magnetometer operation mode: %w", err)
	}

	if err := mag.readTrimData(); err != nil {
		_ = mag.Close()
		return nil, fmt.Errorf("failed to read magnetometer trim data: %w", err)
	}

	return mag, nil
}

func (m *Magnetometer) readTrimData() error {
	var err error

	val, err := m.ReadByteData(MAG_DIG_X1)
	if err != nil {
		return err
	}
	m.trimData.digX1 = int8(val)

	val, err = m.ReadByteData(MAG_DIG_Y1)
	if err != nil {
		return err
	}
	m.trimData.digY1 = int8(val)

	val, err = m.ReadByteData(MAG_DIG_X2)
	if err != nil {
		return err
	}
	m.trimData.digX2 = int8(val)

	val, err = m.ReadByteData(MAG_DIG_Y2)
	if err != nil {
		return err
	}
	m.trimData.digY2 = int8(val)

	lsb, err := m.ReadByteData(MAG_DIG_Z1_LSB)
	if err != nil {
		return err
	}
	msb, err := m.ReadByteData(MAG_DIG_Z1_MSB)
	if err != nil {
		return err
	}
	m.trimData.digZ1 = uint16(msb)<<8 | uint16(lsb)

	lsb, err = m.ReadByteData(MAG_DIG_Z2_LSB)
	if err != nil {
		return err
	}
	msb, err = m.ReadByteData(MAG_DIG_Z2_MSB)
	if err != nil {
		return err
	}
	m.trimData.digZ2 = int16(uint16(msb)<<8 | uint16(lsb))

	lsb, err = m.ReadByteData(MAG_DIG_Z3_LSB)
	if err != nil {
		return err
	}
	msb, err = m.ReadByteData(MAG_DIG_Z3_MSB)
	if err != nil {
		return err
	}
	m.trimData.digZ3 = int16(uint16(msb)<<8 | uint16(lsb))

	lsb, err = m.ReadByteData(MAG_DIG_Z4_LSB)
	if err != nil {
		return err
	}
	msb, err = m.ReadByteData(MAG_DIG_Z4_MSB)
	if err != nil {
		return err
	}
	m.trimData.digZ4 = int16(uint16(msb)<<8 | uint16(lsb))

	val, err = m.ReadByteData(MAG_DIG_XY1)
	if err != nil {
		return err
	}
	m.trimData.digXY1 = uint8(val)

	val, err = m.ReadByteData(MAG_DIG_XY2)
	if err != nil {
		return err
	}
	m.trimData.digXY2 = int8(val)

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

// Compensation follows the BMM150 trim formula and returns 1/16 µT units.
func (m *Magnetometer) compensateX(magDataX int16, dataRhall uint16) int16 {
	if magDataX == BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL ||
		dataRhall == 0 || m.trimData.digXYZ1 == 0 {
		return BMM150_OVERFLOW_OUTPUT
	}

	val := int64(uint32(m.trimData.digXYZ1)<<14/uint32(dataRhall)) - 0x4000
	inner := ((int64(m.trimData.digXY2)*(val*val>>7) +
		val*(int64(m.trimData.digXY1)<<7)) >> 9) + 0x100000
	inner = inner * (int64(int16(m.trimData.digX2)) + 0xA0) >> 12
	return int16((int64(magDataX)*inner)>>13 + int64(m.trimData.digX1)<<3)
}

func (m *Magnetometer) compensateY(magDataY int16, dataRhall uint16) int16 {
	if magDataY == BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL ||
		dataRhall == 0 || m.trimData.digXYZ1 == 0 {
		return BMM150_OVERFLOW_OUTPUT
	}

	val := int64(uint32(m.trimData.digXYZ1)<<14/uint32(dataRhall)) - 0x4000
	inner := ((int64(m.trimData.digXY2)*(val*val>>7) +
		val*(int64(m.trimData.digXY1)<<7)) >> 9) + 0x100000
	inner = inner * (int64(int16(m.trimData.digY2)) + 0xA0) >> 12
	return int16((int64(magDataY)*inner)>>13 + int64(m.trimData.digY1)<<3)
}

// Z uses the distinct trim formula; int64 math avoids intermediate overflow.
func (m *Magnetometer) compensateZ(magDataZ int16, dataRhall uint16) int16 {
	if magDataZ == BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL ||
		m.trimData.digZ2 == 0 || m.trimData.digZ1 == 0 ||
		m.trimData.digXYZ1 == 0 || dataRhall == 0 {
		return BMM150_OVERFLOW_OUTPUT
	}

	a := (int64(magDataZ) - int64(m.trimData.digZ4)) << 15
	dr := int64(int16(dataRhall)) - int64(int16(m.trimData.digXYZ1))
	b := int64(m.trimData.digZ3) * dr >> 2
	num := a - b

	inner := (int64(m.trimData.digZ1)*int64(int16(dataRhall))<<1 + 1<<15) >> 16

	den := int64(m.trimData.digZ2) + int64(int16(inner))
	if den == 0 {
		return BMM150_OVERFLOW_OUTPUT
	}

	out := num / den
	switch {
	case out > math.MaxInt16:
		return math.MaxInt16
	case out < math.MinInt16:
		return math.MinInt16
	}
	return int16(out)
}

// ReadRaw returns uncompensated 13/15-bit ADC values, 14-bit Hall resistance,
// and DRDY. Calibration capture needs this pre-correction chip output.
func (m *Magnetometer) ReadRaw() (x, y, z int16, rhall uint16, drdy bool, err error) {

	buf, e := m.ReadBlockData(MAG_DATAX_LSB, 8)
	if e != nil {
		err = e
		return
	}
	if len(buf) != 8 {
		err = fmt.Errorf("mag ReadRaw: short read (%d bytes)", len(buf))
		return
	}
	xLSB, xMSB := buf[0], buf[1]
	yLSB, yMSB := buf[2], buf[3]
	zLSB, zMSB := buf[4], buf[5]
	rhallLSB, rhallMSB := buf[6], buf[7]

	xRaw := (uint16(xMSB) << 5) | (uint16(xLSB) >> 3)
	yRaw := (uint16(yMSB) << 5) | (uint16(yLSB) >> 3)
	if xRaw&0x1000 != 0 {
		xRaw |= 0xE000
	}
	if yRaw&0x1000 != 0 {
		yRaw |= 0xE000
	}

	zRaw := (uint16(zMSB) << 7) | (uint16(zLSB) >> 1)
	if zRaw&0x4000 != 0 {
		zRaw |= 0x8000
	}

	rhall = (uint16(rhallMSB) << 6) | (uint16(rhallLSB) >> 2)
	drdy = rhallLSB&0x01 != 0

	return int16(xRaw), int16(yRaw), int16(zRaw), rhall, drdy, nil
}

func (m *Magnetometer) ReadSample() (MagSample, error) {
	rawX, rawY, rawZ, rhall, dataReady, err := m.ReadRaw()
	if err != nil {
		return MagSample{}, err
	}
	return MagSample{
		RawX: rawX, RawY: rawY, RawZ: rawZ,
		RHall: rhall, DataReady: dataReady,
		CompX: m.compensateX(rawX, rhall),
		CompY: m.compensateY(rawY, rhall),
		CompZ: m.compensateZ(rawZ, rhall),
	}, nil
}

func (m *Magnetometer) ReadData() (x, y, z int16, err error) {
	sample, err := m.ReadSample()
	if err != nil {
		return 0, 0, 0, err
	}
	return sample.CompX, sample.CompY, sample.CompZ, nil
}

func (m *Magnetometer) SetCalibration(cal Calibration) {
	m.calibrationMu.Lock()
	defer m.calibrationMu.Unlock()
	m.calibration = cal
	m.calibrationRevision++
}

func (m *Magnetometer) Calibration() Calibration {
	m.calibrationMu.RLock()
	defer m.calibrationMu.RUnlock()
	return m.calibration
}

func (m *Magnetometer) CalibrationRevision() uint64 {
	m.calibrationMu.RLock()
	defer m.calibrationMu.RUnlock()
	return m.calibrationRevision
}

// Compensated BMM150 LSB values are 1/16 µT.
const magScaleUT = 16.0

func (m *Magnetometer) ReadDataInMicroTesla() (vx, vy, vz, magnitude float64, err error) {
	_, _, _, vx, vy, vz, magnitude, _, err = m.ReadAll()
	return
}

// ReadAll supplies both chip-frame compensated values and calibrated vehicle-frame
// µT from one I²C transfer; DRDY is false when polling faster than ODR.
func (m *Magnetometer) ReadAll() (compX, compY, compZ int16, vx, vy, vz, magnitude float64, drdy bool, err error) {
	sample, e := m.ReadSample()
	if e != nil {
		err = e
		return
	}
	compX, compY, compZ = sample.CompX, sample.CompY, sample.CompZ
	drdy = sample.DataReady

	cal := m.Calibration()
	sx := float64(compX) - cal.HardIronOffset[0]
	sy := float64(compY) - cal.HardIronOffset[1]
	sz := float64(compZ) - cal.HardIronOffset[2]
	matrix := cal.SoftIronXY
	if matrix == ([2][2]float64{}) {
		matrix = [2][2]float64{{1, 0}, {0, 1}}
	}
	sx, sy = matrix[0][0]*sx+matrix[0][1]*sy,
		matrix[1][0]*sx+matrix[1][1]*sy
	vx, vy, vz = cal.Orientation.Apply(sx, sy, sz)
	vx /= magScaleUT
	vy /= magScaleUT
	vz /= magScaleUT
	magnitude = math.Sqrt(vx*vx + vy*vy + vz*vz)
	return
}

func (m *Magnetometer) Orientation() Orientation {
	return m.Calibration().Orientation
}

func (m *Magnetometer) LeveledHorizontal(magX, magY, magZ, rollRad, pitchRad float64) (bx, by float64) {
	if math.IsNaN(rollRad) || math.IsNaN(pitchRad) {
		return magX, magY
	}
	// NED frame: X forward, Y right, Z down.
	sr, cr := math.Sincos(rollRad)
	sp, cp := math.Sincos(pitchRad)
	return magX*cp + magY*sp*sr + magZ*sp*cr,
		magY*cr - magZ*sr
}

func (m *Magnetometer) HeadingFromVector(magX, magY, magZ, rollRad, pitchRad float64) float64 {
	bx, by := m.LeveledHorizontal(magX, magY, magZ, rollRad, pitchRad)

	angleDeg := math.Atan2(-by, bx) * 180.0 / math.Pi
	angleDeg += m.Calibration().YawOffsetDeg
	angleDeg = math.Mod(angleDeg+360.0, 360.0)
	return angleDeg
}

func (m *Magnetometer) ReadHeading() (heading float64, err error) {
	x, y, z, _, err := m.ReadDataInMicroTesla()
	if err != nil {
		return 0, err
	}
	return m.HeadingFromVector(x, y, z, math.NaN(), math.NaN()), nil
}
