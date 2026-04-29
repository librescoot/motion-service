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
// Other vehicles will need their own values — run bmx-calibrate, drive a
// few circles, and use the JSON summary to set HardIronOffset.
//
// HardIronOffset captured 2026-04-29 with bmx-calibrate over ~5.6 full
// rotations on Deep Blue (CW + CCW circles + figure-8s). The Y offset of
// +320 LSB (~20 µT) is the battery/motor pack's lateral magnetization;
// the Z offset of +996 LSB (~62 µT) is the steel chassis sheet directly
// below the sensor — its residual magnetization actually exceeds Earth's
// vertical field at this distance.
//
// AxisSign and YawOffsetDeg need post-deploy verification: spin the
// scooter clockwise (looking from above) and confirm heading increases
// 0→90→180→270 (otherwise flip a sign); park pointing magnetic North and
// set YawOffsetDeg to whatever offset is needed to read 0.
var DefaultCalibration = Calibration{
	HardIronOffset: [3]int16{-9, 320, 996},
	AxisSign:       [3]float64{-1, 1, -1},
	YawOffsetDeg:   0,
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

// compensateX applies temperature compensation to X-axis magnetometer data.
// All intermediates are int64 — the equivalent C formula overflows int32
// for typical earth-field readings (mag * inner ≈ 6e10 with normal trim
// values, exceeding the int32 range).
//
// Output is in 1/16 µT per LSB, so a 30 µT field yields ≈ 480 LSB. This
// is the scale used downstream in ReadDataInMicroTesla (which divides by
// 16) and matches the convention of the Linux IIO magnetometer drivers.
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

// compensateY applies temperature compensation to Y-axis magnetometer data.
// Same structure as compensateX with digY1/digY2 substituted; digXY1/XY2
// are shared cross-axis trim values.
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

// compensateZ applies temperature compensation to Z-axis magnetometer data.
// The Z formula is structurally different from X/Y — it uses dig_z1..dig_z4
// and digXYZ1, with a denominator built from dig_z1 * rhall. int64 math
// throughout to avoid the int32 overflow in (mag - dig_z4) << 15 * 1000.
//
// Output is in 1/16 µT per LSB, matching X/Y.
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
	// int16 truncation mirrors the C reference. For typical trim/rhall
	// pairs the value fits, so this is a no-op; for pathological inputs
	// it wraps the way the C code does.
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

// ReadRaw reads the magnetometer's pre-compensation 13/15-bit ADC outputs
// plus the 14-bit Hall resistance value and the data-ready flag. Use this
// for calibration capture — it's the chip output before any temperature
// compensation, which is what offline ellipsoid fits operate on.
//
// drdy is true if the chip reports that this sample is fresh (DRDY bit in
// register 0x48). Polling faster than ODR will return drdy=false on the
// repeat reads.
func (m *Magnetometer) ReadRaw() (x, y, z int16, rhall uint16, drdy bool, err error) {
	xLSB, err := m.ReadByteData(MAG_DATAX_LSB)
	if err != nil {
		return
	}
	xMSB, err := m.ReadByteData(MAG_DATAX_LSB + 1)
	if err != nil {
		return
	}
	yLSB, err := m.ReadByteData(MAG_DATAY_LSB)
	if err != nil {
		return
	}
	yMSB, err := m.ReadByteData(MAG_DATAY_LSB + 1)
	if err != nil {
		return
	}
	zLSB, err := m.ReadByteData(MAG_DATAZ_LSB)
	if err != nil {
		return
	}
	zMSB, err := m.ReadByteData(MAG_DATAZ_LSB + 1)
	if err != nil {
		return
	}
	rhallLSB, err := m.ReadByteData(MAG_RHALL_LSB)
	if err != nil {
		return
	}
	rhallMSB, err := m.ReadByteData(MAG_RHALL_MSB)
	if err != nil {
		return
	}

	// X/Y are 13-bit signed: 5 bits in LSB[7:3], 8 bits in MSB[7:0].
	xRaw := (uint16(xMSB) << 5) | (uint16(xLSB) >> 3)
	yRaw := (uint16(yMSB) << 5) | (uint16(yLSB) >> 3)
	if xRaw&0x1000 != 0 {
		xRaw |= 0xE000
	}
	if yRaw&0x1000 != 0 {
		yRaw |= 0xE000
	}

	// Z is 15-bit signed: 7 bits in LSB[7:1], 8 bits in MSB[7:0].
	zRaw := (uint16(zMSB) << 7) | (uint16(zLSB) >> 1)
	if zRaw&0x4000 != 0 {
		zRaw |= 0x8000
	}

	// RHALL is 14-bit unsigned: 6 bits in LSB[7:2], 8 bits in MSB[7:0].
	// LSB bit 0 is the data-ready status.
	rhall = (uint16(rhallMSB) << 6) | (uint16(rhallLSB) >> 2)
	drdy = rhallLSB&0x01 != 0

	return int16(xRaw), int16(yRaw), int16(zRaw), rhall, drdy, nil
}

// ReadData reads the magnetometer's data registers and returns
// temperature-compensated values in chip "1/16 µT" units. For calibration
// capture use ReadRaw instead.
func (m *Magnetometer) ReadData() (x, y, z int16, err error) {
	rawX, rawY, rawZ, rhall, _, err := m.ReadRaw()
	if err != nil {
		return 0, 0, 0, err
	}
	return m.compensateX(rawX, rhall),
		m.compensateY(rawY, rhall),
		m.compensateZ(rawZ, rhall),
		nil
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