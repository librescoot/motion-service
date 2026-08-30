package bmx

import (
	"fmt"
	"syscall"
	"unsafe"

	"golang.org/x/sys/unix"
)

const (
	BMX055_ACCEL_ADDR = 0x18
	BMX055_GYRO_ADDR  = 0x68
	BMX055_MAG_ADDR   = 0x10
)

// Orientation maps the shared BMX055 sensor axes into the vehicle frame:
// AxisOrder permutes axes and AxisSign applies vehicle-axis polarity.
type Orientation struct {
	AxisOrder [3]int
	AxisSign  [3]float64
}

func (o Orientation) Apply(sx, sy, sz float64) (vx, vy, vz float64) {
	src := [3]float64{sx, sy, sz}
	return o.AxisSign[0] * src[o.AxisOrder[0]],
		o.AxisSign[1] * src[o.AxisOrder[1]],
		o.AxisSign[2] * src[o.AxisOrder[2]]
}

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

const (
	ACCEL_INT_EN_SLOW_NO_MOTION_X   = 0x01
	ACCEL_INT_EN_SLOW_NO_MOTION_Y   = 0x02
	ACCEL_INT_EN_SLOW_NO_MOTION_Z   = 0x04
	ACCEL_INT_EN_SLOW_NO_MOTION_SEL = 0x08
	ACCEL_INT_STATUS_SLOW_NO_MOT    = 0x08

	ACCEL_INT_EN_SLOPE_X = 0x01
	ACCEL_INT_EN_SLOPE_Y = 0x02
	ACCEL_INT_EN_SLOPE_Z = 0x04

	ACCEL_INT_STATUS_SLOPE = 0x04
)

const (
	ACCEL_INT1_MAP_SLOW_NO_MOTION = 0x08
	ACCEL_INT2_MAP_SLOW_NO_MOTION = 0x08
	ACCEL_INT1_MAP_SLOPE          = 0x04
	ACCEL_INT2_MAP_SLOPE          = 0x04
)

// Register 0x27 shares slope duration (bits 1:0) with slow-motion duration.
const (
	ACCEL_SLOPE_DURATION  = 0x27
	ACCEL_SLOPE_THRESHOLD = 0x28
)

// ODR is twice the selected accelerometer bandwidth; reset defaults to 1000 Hz.
const (
	ACCEL_BW_7_81HZ  = 0x08
	ACCEL_BW_15_63HZ = 0x09
	ACCEL_BW_31_25HZ = 0x0A
	ACCEL_BW_62_5HZ  = 0x0B
	ACCEL_BW_125HZ   = 0x0C
	ACCEL_BW_250HZ   = 0x0D
	ACCEL_BW_500HZ   = 0x0E
	ACCEL_BW_1000HZ  = 0x0F
)

const (
	ACCEL_INT_NON_LATCHED = 0x00
	ACCEL_INT_LATCHED     = 0x0F
)

const (
	ACCEL_INT1_ACTIVE_HIGH = 0x01
	ACCEL_INT1_OPEN_DRAIN  = 0x02
	ACCEL_INT2_ACTIVE_HIGH = 0x04
	ACCEL_INT2_OPEN_DRAIN  = 0x08
)

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

const (
	MAG_REPXY_LOWPOWER = 0x01
	MAG_REPZ_LOWPOWER  = 0x02
	MAG_REPXY_REGULAR  = 0x04
	MAG_REPZ_REGULAR   = 0x0E
	MAG_REPXY_ENHANCED = 0x07
	MAG_REPZ_ENHANCED  = 0x1A
	MAG_REPXY_HIGHACC  = 0x17
	MAG_REPZ_HIGHACC   = 0x52
)

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

const (
	BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL = -4096
	BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL  = -16384
	BMM150_OVERFLOW_OUTPUT             = -32768
)

const (
	I2C_SLAVE                = 0x0703
	I2C_SMBUS                = 0x0720
	I2C_SMBUS_READ           = 1
	I2C_SMBUS_WRITE          = 0
	I2C_SMBUS_BYTE_DATA      = 2
	I2C_SMBUS_WORD_DATA      = 3
	I2C_SMBUS_BLOCK_DATA     = 5
	I2C_SMBUS_I2C_BLOCK_DATA = 8

	I2C_SMBUS_BLOCK_MAX = 32
)

// Linux's SMBus ioctl ABI expects a 34-byte data union.
type smbusIoctlData struct {
	readWrite byte
	command   byte
	size      uint32
	data      *[34]byte
}

type i2cDevice struct {
	fd   int
	bus  string
	addr byte
	name string
}

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
		_ = unix.Close(fd)
		return nil, fmt.Errorf("failed to set I2C slave address 0x%02X: %v", addr, errno)
	}

	return &i2cDevice{
		fd:   fd,
		bus:  bus,
		addr: addr,
	}, nil
}

func (d *i2cDevice) Close() error {
	if d.fd >= 0 {
		return unix.Close(d.fd)
	}
	return nil
}

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

func (d *i2cDevice) ReadBlockData(reg byte, count int) ([]byte, error) {
	if count < 1 || count > I2C_SMBUS_BLOCK_MAX {
		return nil, fmt.Errorf("ReadBlockData: invalid count %d (must be 1..%d)", count, I2C_SMBUS_BLOCK_MAX)
	}
	var dataBlock [34]byte
	dataBlock[0] = byte(count)
	data := &smbusIoctlData{
		readWrite: I2C_SMBUS_READ,
		command:   reg,
		size:      I2C_SMBUS_I2C_BLOCK_DATA,
		data:      &dataBlock,
	}

	_, _, errno := syscall.Syscall(
		syscall.SYS_IOCTL,
		uintptr(d.fd),
		I2C_SMBUS,
		uintptr(unsafe.Pointer(data)),
	)
	if errno != 0 {
		return nil, fmt.Errorf("I2C_SMBUS_I2C_BLOCK read failed: %v", errno)
	}
	got := int(dataBlock[0])
	if got > count {
		got = count
	}
	out := make([]byte, got)
	copy(out, dataBlock[1:1+got])
	return out, nil
}

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
