package driver

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"syscall"
	"time"
)

// sysfs unbind is asynchronous; reading I²C before detach completes can return ENXIO.
const unbindSettleTimeout = 2 * time.Second
const unbindPollInterval = 10 * time.Millisecond

type DriverBinding struct {
	DriverName string
	DeviceID   string
}

func Unbind(driverName, deviceID string) error {
	boundPath := filepath.Join("/sys/bus/i2c/drivers", driverName, deviceID)
	if _, err := os.Stat(boundPath); os.IsNotExist(err) {

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

// sysfs unbind returns before the kernel has released the I²C device.
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

// Kernel driver names vary by image; absent bindings are intentionally no-ops.
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
