package driver

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"syscall"
)

// DriverBinding represents a kernel driver that needs to be unbound
type DriverBinding struct {
	DriverName string
	DeviceID   string
}

// Unbind unbinds a kernel driver from a device
func Unbind(driverName, deviceID string) error {
	unbindPath := filepath.Join("/sys/bus/i2c/drivers", driverName, "unbind")

	file, err := os.OpenFile(unbindPath, os.O_WRONLY, 0)
	if err != nil {
		if os.IsNotExist(err) {
			return nil
		}
		return fmt.Errorf("failed to open unbind file %s: %w", unbindPath, err)
	}
	defer file.Close()

	_, err = file.WriteString(deviceID)
	if err != nil {
		// ENODEV = device isn't bound to this driver. That's exactly what
		// we wanted, so don't surface it as an error.
		if errors.Is(err, syscall.ENODEV) {
			return nil
		}
		return fmt.Errorf("failed to write device ID to unbind file: %w", err)
	}

	return nil
}

// UnbindBMX055 unbinds any kernel driver that may be holding one of the
// BMX055 sensors. We try multiple driver names per sensor because the IIO
// driver names have changed across kernel versions:
//   - Magnetometer is bmc150_magn_i2c on 5.4+ (matches DT compatible
//     "bosch,bmm050"); the older bmm150_i2c name is also tried.
//   - Accelerometer drivers are usually disabled in the LibreScoot kernel
//     config but we still try in case a future image leaves them enabled.
//   - Gyroscope likewise.
//
// Unbind is a no-op if the driver path or device binding doesn't exist;
// only an unexpected open/write failure surfaces as an error.
func UnbindBMX055() error {
	drivers := []DriverBinding{
		{"bmc150_accel_i2c", "3-0018"},
		{"bma2x2_i2c", "3-0018"},
		{"bmg160_i2c", "3-0068"},
		{"bmc150_magn_i2c", "3-0010"},
		{"bmm150_i2c", "3-0010"},
	}

	for _, d := range drivers {
		if err := Unbind(d.DriverName, d.DeviceID); err != nil {
			return fmt.Errorf("failed to unbind %s: %w", d.DriverName, err)
		}
	}

	return nil
}