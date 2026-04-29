# bmx-service

Hardware abstraction service for the BMX055 9-axis sensor package on LibreScoot.

## Features

- Direct I2C/SMBus communication with BMX055 sensors
- Continuous sensor polling (10Hz default, configurable 1-100Hz)
- Motion interrupt detection with three sensitivity presets
- Redis interface for sensor data and command/control
- Kernel driver management (automatic unbinding)

## Architecture

Three sensors in one package:
- **BMA253** Accelerometer (0x18): 12-bit, ±2g
- **BMG160** Gyroscope (0x68): 16-bit, ±2000°/s
- **BMM150** Magnetometer (0x10): 13/15-bit

## Build

```bash
make build          # ARM binary for target
make build-amd64    # AMD64 binary
```

## Usage

```bash
bmx-service --i2c-bus=/dev/i2c-3 --redis=localhost:6379 --polling-rate=10
```

## Redis Interface

### Published Data

**Sensor readings** (`bmx:sensors` channel, 10Hz):
```json
{
  "timestamp": 1696089234567,
  "accel": {"x": 0.278, "y": 0.021, "z": -0.944, "magnitude": 0.985, "unit": "g"},
  "gyro": {"x": -0.4, "y": -3.4, "z": -3.8, "magnitude": 5.1, "unit": "deg/s"},
  "mag": {"x": 10.5, "y": -15.2, "z": 8.6, "magnitude": 20.3, "unit": "uT"}
}
```

**Magnetic heading** (`bmx:heading` channel, 5Hz):
```json
{
  "timestamp": 1696089234567,
  "heading_deg": 127.4,
  "heading_raw_deg": 128.1,
  "accuracy_deg": 3.8,
  "tilt_compensated": true,
  "tilt_deg": 4.2,
  "mag_strength_ut": 48.6,
  "excess_g": 0.03,
  "yaw_rate_dps": 1.7
}
```
`accuracy_deg` is a heuristic that grows with tilt, non-gravity
acceleration (`excess_g`), and yaw rate. `tilt_compensated` is `false`
when the accelerometer is moving enough that it can't be trusted as a
pure gravity vector (hard braking, big bumps); the heading then falls
back to X/Y-only and accuracy is reported accordingly.

**Interrupt events** (`bmx:interrupt` PUBSUB + `bmx:events` Stream)
**Status** (`bmx` hash). Heading-related fields:
- `heading` (int, 0-359, legacy)
- `heading-deg` (float, 0-360)
- `heading-accuracy` (float, deg)
- `heading-tilt` (float, deg)
- `heading-tilt-comp` (`true`/`false`)

### Accepted Commands

Via `scooter:bmx` queue:
```bash
LPUSH scooter:bmx sensitivity:low|medium|high
LPUSH scooter:bmx pin:int1|int2|none
LPUSH scooter:bmx interrupt:enable|disable
LPUSH scooter:bmx reset
LPUSH scooter:bmx polling:10
```

## Sensitivity Presets

- **LOW**: threshold=0x10 (least sensitive)
- **MEDIUM**: threshold=0x09 (default)
- **HIGH**: threshold=0x08 (most sensitive)

## Deployment

```bash
make build
scp bin/bmx-service root@10.7.0.4:/usr/bin/
ssh root@10.7.0.4 "systemctl daemon-reload && systemctl restart librescoot-bmx"
```

## Magnetometer calibration capture

`bmx-calibrate` is a one-shot diagnostic binary that captures raw
magnetometer + accelerometer + gyroscope data to a CSV in `/data/`. Use
it to derive hard-iron and (with enough orientation coverage) soft-iron
calibration for a particular vehicle.

```bash
make build
scp bin/bmx-calibrate systemd/bmx-calibrate.service root@10.7.0.4:/data/
ssh root@10.7.0.4 'cp /data/bmx-calibrate /usr/bin/ \
  && cp /data/bmx-calibrate.service /etc/systemd/system/ \
  && systemctl daemon-reload'
```

The unit `Conflicts=` with `librescoot-alarm` and `librescoot-bmx`, so
starting it stops whichever is currently using the BMX055. On stop it
brings `librescoot-alarm` back up via `systemctl --no-block start`.

```bash
# Start a capture (alarm-service stops automatically)
ssh deep-blue systemctl start bmx-calibrate

# ... rotate the scooter (driving a circle works for X/Y hard-iron) ...

# Stop the capture (alarm-service comes back up)
ssh deep-blue systemctl stop bmx-calibrate

# Output CSV is at /data/bmx-cal-<unix-ts>.csv
ssh deep-blue ls -lh /data/bmx-cal-\*.csv
```

Live progress is logged to journald every 2 seconds:
```
samples=80 rate_hz=19.9 x="[-27,-20] span=7" y="[53,70] span=17"
  z="[101,107] span=6" hard_iron_xyz=[-23,61,104]
```

A high-quality hard-iron capture should produce X and Y spans of
roughly 2 × Earth's horizontal field (≈ ±480 LSB peak-to-peak in
Berlin, so a span around 950 LSB after a full 360° rotation).

CSV columns:
```
timestamp_ms, mag_raw_x, mag_raw_y, mag_raw_z, mag_rhall, mag_drdy,
mag_comp_x, mag_comp_y, mag_comp_z, ax_g, ay_g, az_g,
gx_dps, gy_dps, gz_dps
```

Raw values are int16 ADC outputs (13-bit X/Y, 15-bit Z) — these are
what an offline ellipsoid fit operates on. Compensated values are in
1/16 µT/LSB (480 LSB ≈ 30 µT) and shown for sanity-checking the
chip's response.

## Testing

```bash
# Monitor sensor data
redis-cli -h 10.7.0.4 SUBSCRIBE bmx:sensors

# Test motion detection
redis-cli -h 10.7.0.4 LPUSH scooter:bmx sensitivity:medium
redis-cli -h 10.7.0.4 LPUSH scooter:bmx interrupt:enable
redis-cli -h 10.7.0.4 SUBSCRIBE bmx:interrupt
# Shake the scooter - should see interrupt events
```