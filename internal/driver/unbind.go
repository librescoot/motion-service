package driver

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"syscall"
	"time"
)

// unbindSettleTimeout caps how long we'll wait for the kernel to actually
// release the device after writing the unbind file. The write itself is
// non-blocking; the i2c device only becomes free for userspace access once
// the kernel's detach path completes, and racing it leads to ENXIO on the
// first SMBUS read (see bean librescoot-o7in).
const unbindSettleTimeout = 2 * time.Second
const unbindPollInterval = 10 * time.Millisecond

// DriverBinding represents a kernel driver that needs to be unbound
type DriverBinding struct {
	DriverName string
	DeviceID   string
}

// Unbind unbinds a kernel driver from a device and waits for the kernel to
// actually release it (or returns an error after unbindSettleTimeout). Safe
// to call on drivers or devices that aren't currently bound.
func Unbind(driverName, deviceID string) error {
	boundPath := filepath.Join("/sys/bus/i2c/drivers", driverName, deviceID)
	if _, err := os.Stat(boundPath); os.IsNotExist(err) {
		// Driver isn't bound to this device — nothing to do.
		return nil
	}

	unbindPath := filepath.Join("/sys/bus/i2c/drivers", driverName, "unbind")
	file, err := os.OpenFile(unbindPath, os.O_WRONLY, 0)
	if err != nil {
		if os.IsNotExist(err) {
			return nil
		}
		return fmt.Errorf("failed to open unbind file %s: %w", unbindPath, err)
	}

	_, writeErr := file.WriteString(deviceID)
	closeErr := file.Close()
	if writeErr != nil {
		// ENODEV = device isn't bound to this driver. That's exactly what
		// we wanted, so don't surface it as an error.
		if errors.Is(writeErr, syscall.ENODEV) {
			return nil
		}
		return fmt.Errorf("failed to write device ID to unbind file: %w", writeErr)
	}
	if closeErr != nil {
		return fmt.Errorf("failed to close unbind file: %w", closeErr)
	}

	return waitForDetach(boundPath)
}

// waitForDetach polls until the kernel symlink for the bound device is gone,
// or until unbindSettleTimeout elapses.
func waitForDetach(boundPath string) error {
	deadline := time.Now().Add(unbindSettleTimeout)
	for {
		if _, err := os.Stat(boundPath); os.IsNotExist(err) {
			return nil
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("kernel did not release %s within %v", boundPath, unbindSettleTimeout)
		}
		time.Sleep(unbindPollInterval)
	}
}

// UnbindBMX055 unbinds any kernel driver that may be holding one of the
// BMX055 sensors. We try multiple driver names per sensor because the IIO
// driver names have changed across kernel versions:
//   - Magnetometer is bmc150_magn_i2c on 5.4+ (matches DT compatible
//     "bosch,bmm050"); the older bmm150_i2c name is also tried.
//   - Accelerometer drivers are usually disabled in the Librescoot kernel
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