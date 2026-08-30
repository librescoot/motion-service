# Librescoot Motion Service

Part of the [Librescoot](https://librescoot.org/) open-source platform.

## Overview

`motion-service` owns the BMX055 IMU on a Librescoot vehicle. It reads accelerometer, gyroscope, and—when available—magnetometer data; publishes telemetry and motion interrupts through Redis/Valkey; and programs the sensor profile required by [`alarm-service`](../alarm-service/README.md) and power-manager state.

## Capabilities

- Direct I²C/SMBus access to the BMX055 accelerometer, gyroscope, and magnetometer, including release of the kernel drivers before use.
- Continuous accelerometer/gyroscope telemetry and optional magnetometer/heading telemetry.
- Motion interrupt detection through an evdev GPIO edge source when available, with I²C status polling as a fallback/watchdog.
- Alarm- and hibernation-aware sensor profiles, with the applied register-derived state exposed in Redis.
- A Redis RPC interface for hibernation preparation, diagnostics, telemetry rate, and streaming control.
- `motion-calibrate`, a separate one-shot CSV capture utility for magnetometer calibration data.

## Operation and interfaces

### Profiles and alarm relationship

motion-service watches `alarm.status` and `power-manager.state` hashes. It owns the hardware configuration; alarm-service does not set sensor registers. The derived profiles are:

| Profile | Selected when | Interrupt mode |
|---|---|---|
| `idle` | any other alarm status | disabled |
| `armed-awake` | `alarm.status=armed` outside hibernation states | any-motion, both interrupt pins |
| `armed-hibernation` | `alarm.status=armed` while power-manager is hibernating or hibernation-imminent | any-motion, both interrupt pins |
| `level1` | `alarm.status=level-1-triggered` | slow-motion, both interrupt pins |
| `waiting` | `alarm.status=level-2-triggered` | slow-motion, INT1 |

Before hibernation, alarm-service calls `prepare-hibernation` on `motion:rpc`. That call applies and confirms `armed-hibernation`; it is the synchronization point between the two services. On startup, a latched motion interrupt is published as a `wake-hibernation` event and stored as `motion.wake-cause` for a consumer that starts later.

The service also watches `vehicle.state` to select the telemetry poll rate: 5 Hz for `parked` and `ready-to-drive`, and 1 Hz for other values. The process flag supplies the initial rate; the first vehicle state update can replace it.

### Redis/Valkey contract

| Interface | Direction | Contract |
|---|---|---|
| `motion:sensors` | publishes | JSON `SensorReading`: millisecond `timestamp`; `accel` and `gyro` axes; optional `mag` axis. Axis objects contain `x`, `y`, `z`, `magnitude`, and `unit`. |
| `motion:heading` | publishes | JSON heading with `heading_deg`, raw/fast/slow headings, `accuracy_deg`, `tilt_compensated`, `tilt_deg`, magnetic strength, excess acceleration, and yaw rate. |
| `motion:interrupt` | publishes | JSON `{ "type": ..., "timestamp": ..., "engine": ... }`; alarm-service consumes this channel. |
| `motion:ready` | publishes | Current millisecond timestamp after initialization. |
| `motion` hash | publishes | Initialization/error fields; telemetry state; `current-profile`, `mode`, `bandwidth`, `threshold`, `duration`, `pin`, and `interrupt`; heading fields; and the one-shot `wake-cause` when applicable. |
| `alarm` and `power-manager` hashes | watches | `status` and `state` determine the hardware profile. |
| `vehicle` hash | watches | `state` determines telemetry rate. |

Sensor telemetry streaming is enabled initially. `motion` hash status fields are state snapshots, not Pub/Sub notifications.

### RPC

The `motion:rpc` request channel provides these methods:

| Method | Request | Result |
|---|---|---|
| `prepare-hibernation` | `{ "profile": "armed-hibernation" }` (or empty profile) | Programs that profile and returns `programmed` and `profile`. Other profiles are rejected. |
| `get-calibration` | `{}` | Returns hard-iron offsets, axis order/sign, and yaw offset. |
| `clear-latch` | `{}` | Clears a latched accelerometer interrupt. |
| `soft-reset` | `{}` | Resets accelerometer and gyroscope, then reapplies the current profile. |
| `set-polling` | `{ "rate_hz": 1..100 }` | Sets both telemetry pollers and updates `motion.polling-rate-hz`. A later `vehicle.state` update re-derives the rate. |
| `set-streaming` | `{ "enabled": true|false }` | Enables or disables `motion:sensors` publication and updates `motion.streaming`. |

## Configuration

The service is configured by command-line flags:

```text
--i2c-bus PATH          default /dev/i2c-3
--redis ADDRESS         default localhost:6379
--log-level LEVEL       debug, info, warn, or error
--polling-rate HZ       initial rate; default 5
--evdev-device PATH     default /dev/input/by-path/platform-gpio-keys-event
--evdev-keycode CODE    default 0x2b
--version
```

Set `--evdev-device=` to disable evdev and use the I²C interrupt poller only. If the configured evdev device cannot be opened, the service logs a warning and continues with I²C polling.

## Build and test

Requires Go and the dependencies declared in `go.mod`.

```sh
make build          # static Linux ARMv7 binaries: bin/motion-service and bin/motion-calibrate
make build-amd64    # AMD64 variants
make test
make lint           # requires golangci-lint
```

`make run`, `make dev-build`, `make fmt`, and `make clean` are also available.

## Deployment and runtime dependencies

The Yocto package installs `/usr/bin/motion-service` and systemd unit `librescoot-motion.service`. The shipped unit starts after and wants `valkey.service`, runs as root, and uses `/dev/i2c-3`, `localhost:6379`, a 5 Hz initial poll rate, and `info` logging.

Runtime requirements are a supported BMX055 on the configured I²C bus, permission to unbind its kernel drivers, Redis/Valkey, and optionally the GPIO-key evdev device. alarm-service requires this service for normal motion alarm operation and hibernation preparation.

### Calibration utility

`motion-calibrate` takes exclusive ownership of the IMU and writes CSV samples. Its defaults are `/dev/i2c-3`, 20 Hz, `highacc` magnetometer preset, and output `/data/bmx-cal-<unix>.csv`. Use its `--output`, `--rate`, `--duration`, `--preset`, and `--progress-every` flags as needed.

The Yocto recipe installs only `motion-service`; build and deploy `motion-calibrate` and its source unit file manually when calibration is required. The provided `systemd/motion-calibrate.service` conflicts with `librescoot-alarm.service` and `librescoot-motion.service`, stopping them before capture. On stop it starts `librescoot-alarm.service` asynchronously. Invoke it manually with `systemctl start motion-calibrate.service`; it is not enabled for boot.

## Operational and security notes

- Do not run motion-service, alarm-service versions that own the IMU, or motion-calibrate concurrently outside the supplied systemd conflict handling. Concurrent I²C access can invalidate sensor configuration and capture data.
- The service resets and reprograms the sensor whenever its profile changes. Use the published `motion.current-profile` and associated fields to diagnose the actual applied configuration rather than attempting manual register writes.
- Redis/Valkey controls sensor state and exposes vehicle motion data without authentication or TLS in this service. Restrict it to trusted local clients.
- Calibration output and raw sensor data can reveal vehicle movement. Protect files under `/data` appropriately.

## License

This project is licensed under the [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](LICENSE).

Made with ❤️ by the Librescoot community
