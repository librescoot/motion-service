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

**Interrupt events** (`bmx:interrupt` PUBSUB + `bmx:events` Stream)
**Status** (`bmx` hash)

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