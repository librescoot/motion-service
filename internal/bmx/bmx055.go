package bmx

import (
	"fmt"
	"syscall"
	"unsafe"

	"golang.org/x/sys/unix"
)

// I2C addresses for BMX055 sensors
const (
	BMX055_ACCEL_ADDR = 0x18
	BMX055_GYRO_ADDR  = 0x68
	BMX055_MAG_ADDR   = 0x10
)

// Orientation maps the BMX055 chip's intrinsic XYZ axes onto a vehicle
// body frame. All three sensors in the package (BMA253 accel, BMG160 gyro,
// BMM150 mag) share the same axis system, so a single orientation applies
// uniformly to whichever ReadDataIn*VehicleFrame helper the caller uses.
//
// AxisOrder maps each vehicle axis to a sensor index (0=X, 1=Y, 2=Z).
// AxisSign is ±1 per VEHICLE axis, applied after permutation.
//
// Example: chip mounted face-down on the PCB underside and rotated 90° so
// chip +Y points to vehicle forward, chip +X to vehicle right, chip +Z to
// vehicle down (NED frame):
//
//	Orientation{AxisOrder: [3]int{1, 0, 2}, AxisSign: [3]float64{1, 1, 1}}
type Orientation struct {
	AxisOrder [3]int
	AxisSign  [3]float64
}

// Apply maps a sensor-frame triple (sx, sy, sz) into the vehicle frame
// described by this Orientation.
func (o Orientation) Apply(sx, sy, sz float64) (vx, vy, vz float64) {
	src := [3]float64{sx, sy, sz}
	return o.AxisSign[0] * src[o.AxisOrder[0]],
		o.AxisSign[1] * src[o.AxisOrder[1]],
		o.AxisSign[2] * src[o.AxisOrder[2]]
}

// Accelerometer registers
const (
	ACCEL_CHIP_ID_REG          = 0x00
	ACCEL_ACCD_X_LSB_REG       = 0x02
	ACCEL_ACCD_Y_LSB_REG       = 0x04
	ACCEL_ACCD_Z_LSB_REG       = 0x06
	ACCEL_INT_STATUS_0         = 0x09
	ACCEL_INT_STATUS_1         = 0x0A
	ACCEL_INT_STATUS_2         = 0x0B
	ACCEL_INT_STATUS_3         = 0x0C
	ACCEL_PMU_RANGE            = 0x0F
	ACCEL_PMU_BW               = 0x10
	ACCEL_PMU_LPW              = 0x11
	ACCEL_BGW_SOFTRESET        = 0x14
	ACCEL_INT_EN_0             = 0x16
	ACCEL_INT_EN_1             = 0x17
	ACCEL_INT_EN_2             = 0x18
	ACCEL_INT_MAP_0            = 0x19
	ACCEL_INT_MAP_1            = 0x1A
	ACCEL_INT_MAP_2            = 0x1B
	ACCEL_INT_SRC              = 0x1E
	ACCEL_INT_OUT_CTRL         = 0x20
	ACCEL_INT_LATCH            = 0x21
	ACCEL_INT_RST_LATCH        = 0x21
	ACCEL_SLO_NO_MOT_DURATION  = 0x27
	ACCEL_SLO_NO_MOT_THRESHOLD = 0x29
)

// Accelerometer interrupt bits
const (
	ACCEL_INT_EN_SLOW_NO_MOTION_X   = 0x01
	ACCEL_INT_EN_SLOW_NO_MOTION_Y   = 0x02
	ACCEL_INT_EN_SLOW_NO_MOTION_Z   = 0x04
	ACCEL_INT_EN_SLOW_NO_MOTION_SEL = 0x08
	ACCEL_INT_STATUS_SLOW_NO_MOT    = 0x08
)

// Accelerometer interrupt mapping
const (
	ACCEL_INT1_MAP_SLOW_NO_MOTION = 0x08
	ACCEL_INT2_MAP_SLOW_NO_MOTION = 0x08
)

// Interrupt latch modes
const (
	ACCEL_INT_NON_LATCHED = 0x00
	ACCEL_INT_LATCHED     = 0x0F
)

// Interrupt output control
const (
	ACCEL_INT1_ACTIVE_HIGH = 0x01
	ACCEL_INT1_OPEN_DRAIN  = 0x02
	ACCEL_INT2_ACTIVE_HIGH = 0x04
	ACCEL_INT2_OPEN_DRAIN  = 0x08
)

// Gyroscope registers
const (
	GYRO_CHIP_ID_REG   = 0x00
	GYRO_RATE_X_LSB    = 0x02
	GYRO_RATE_Y_LSB    = 0x04
	GYRO_RATE_Z_LSB    = 0x06
	GYRO_RANGE         = 0x0F
	GYRO_BW            = 0x10
	GYRO_LPM1          = 0x11
	GYRO_BGW_SOFTRESET = 0x14
)

// Magnetometer registers
const (
	MAG_DATAX_LSB   = 0x42
	MAG_DATAY_LSB   = 0x44
	MAG_DATAZ_LSB   = 0x46
	MAG_RHALL_LSB   = 0x48
	MAG_RHALL_MSB   = 0x49
	MAG_CHIP_ID_REG = 0x40
	MAG_POWER_CTRL  = 0x4B
	MAG_OPMODE_ODR  = 0x4C
	MAG_REPXY       = 0x51
	MAG_REPZ        = 0x52
)

// Magnetometer preset values (REPXY, REPZ) per Bosch BMM150 reference.
// Reps encode as nXY = 1 + 2*REPXY and nZ = 1 + REPZ.
// Default after power-on is REPXY=REPZ=0x00 (1 rep) — noisiest possible.
const (
	MAG_REPXY_LOWPOWER  = 0x01 // 3 reps
	MAG_REPZ_LOWPOWER   = 0x02 // 3 reps
	MAG_REPXY_REGULAR   = 0x04 // 9 reps
	MAG_REPZ_REGULAR    = 0x0E // 15 reps
	MAG_REPXY_ENHANCED  = 0x07 // 15 reps
	MAG_REPZ_ENHANCED   = 0x1A // 27 reps
	MAG_REPXY_HIGHACC   = 0x17 // 47 reps
	MAG_REPZ_HIGHACC    = 0x52 // 83 reps
)

// Magnetometer OPMODE_ODR (0x4C) field encodings.
// Bits 5-3: data rate. Bits 2-1: opmode. Bit 0: self-test.
const (
	MAG_ODR_10HZ = 0x00 << 3
	MAG_ODR_2HZ  = 0x01 << 3
	MAG_ODR_6HZ  = 0x02 << 3
	MAG_ODR_8HZ  = 0x03 << 3
	MAG_ODR_15HZ = 0x04 << 3
	MAG_ODR_20HZ = 0x05 << 3
	MAG_ODR_25HZ = 0x06 << 3
	MAG_ODR_30HZ = 0x07 << 3

	MAG_OPMODE_NORMAL = 0x00 << 1
	MAG_OPMODE_FORCED = 0x01 << 1
	MAG_OPMODE_SLEEP  = 0x03 << 1
)

// Magnetometer trim registers (compensation parameters)
const (
	MAG_DIG_X1       = 0x5D
	MAG_DIG_Y1       = 0x5E
	MAG_DIG_Z4_LSB   = 0x62
	MAG_DIG_Z4_MSB   = 0x63
	MAG_DIG_X2       = 0x64
	MAG_DIG_Y2       = 0x65
	MAG_DIG_Z2_LSB   = 0x68
	MAG_DIG_Z2_MSB   = 0x69
	MAG_DIG_Z1_LSB   = 0x6A
	MAG_DIG_Z1_MSB   = 0x6B
	MAG_DIG_XYZ1_LSB = 0x6C
	MAG_DIG_XYZ1_MSB = 0x6D
	MAG_DIG_Z3_LSB   = 0x6E
	MAG_DIG_Z3_MSB   = 0x6F
	MAG_DIG_XY2      = 0x70
	MAG_DIG_XY1      = 0x71
)

// Magnetometer overflow constants
const (
	BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL = -4096
	BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL  = -16384
	BMM150_OVERFLOW_OUTPUT             = -32768
)

// I2C/SMBus constants
const (
	I2C_SLAVE            = 0x0703
	I2C_SMBUS            = 0x0720
	I2C_SMBUS_READ       = 1
	I2C_SMBUS_WRITE      = 0
	I2C_SMBUS_BYTE_DATA  = 2
	I2C_SMBUS_WORD_DATA  = 3
	I2C_SMBUS_BLOCK_DATA = 5
)

// SMBus I/O control data structure
type smbusIoctlData struct {
	readWrite byte
	command   byte
	size      uint32
	data      *[34]byte
}

// i2cDevice represents a generic I2C device
type i2cDevice struct {
	fd      int
	bus     string
	addr    byte
	name    string
}

// openI2C opens the I2C bus and sets the slave address
func openI2C(bus string, addr byte) (*i2cDevice, error) {
	fd, err := unix.Open(bus, unix.O_RDWR, 0)
	if err != nil {
		return nil, fmt.Errorf("failed to open I2C bus %s: %w", bus, err)
	}

	_, _, errno := syscall.Syscall(
		syscall.SYS_IOCTL,
		uintptr(fd),
		I2C_SLAVE,
		uintptr(addr),
	)
	if errno != 0 {
		unix.Close(fd)
		return nil, fmt.Errorf("failed to set I2C slave address 0x%02X: %v", addr, errno)
	}

	return &i2cDevice{
		fd:   fd,
		bus:  bus,
		addr: addr,
	}, nil
}

// Close closes the I2C device
func (d *i2cDevice) Close() error {
	if d.fd >= 0 {
		return unix.Close(d.fd)
	}
	return nil
}

// ReadByteData reads a byte from a register using SMBus protocol
func (d *i2cDevice) ReadByteData(reg byte) (byte, error) {
	var dataBlock [34]byte
	data := &smbusIoctlData{
		readWrite: I2C_SMBUS_READ,
		command:   reg,
		size:      I2C_SMBUS_BYTE_DATA,
		data:      &dataBlock,
	}

	_, _, errno := syscall.Syscall(
		syscall.SYS_IOCTL,
		uintptr(d.fd),
		I2C_SMBUS,
		uintptr(unsafe.Pointer(data)),
	)

	if errno != 0 {
		return 0, fmt.Errorf("I2C_SMBUS read failed: %v", errno)
	}
	return dataBlock[0], nil
}

// WriteByteData writes a byte to a register using SMBus protocol
func (d *i2cDevice) WriteByteData(reg, value byte) error {
	var dataBlock [34]byte
	dataBlock[0] = value

	data := &smbusIoctlData{
		readWrite: I2C_SMBUS_WRITE,
		command:   reg,
		size:      I2C_SMBUS_BYTE_DATA,
		data:      &dataBlock,
	}

	_, _, errno := syscall.Syscall(
		syscall.SYS_IOCTL,
		uintptr(d.fd),
		I2C_SMBUS,
		uintptr(unsafe.Pointer(data)),
	)

	if errno != 0 {
		return fmt.Errorf("I2C_SMBUS write failed: %v", errno)
	}
	return nil
}