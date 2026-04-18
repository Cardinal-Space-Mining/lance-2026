# Phoenix Drivers

Phoenix 5 and Phoenix 6 motor controller drivers for ROS 2.

This package contains drivers for both LANCE 1 and LANCE 2. The default launch
configuration lives in [`config/phoenix.json`](config/phoenix.json).

## Dependencies

1. Install `patchelf`.

   Without it, some build commands may complete but the Phoenix 5 driver can
   fail later with a linking error.

   ```bash
   sudo apt update
   sudo apt install patchelf
   ```

2. Install CTRE Phoenix 6.

   Follow the official CTRE Linux installation notes if the target machine is
   ARM. For a standard install, configure `YEAR` for the current CTRE repository:

   ```bash
   YEAR=2026
   sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
   sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr${YEAR}.list "https://deb.ctr-electronics.com/ctr${YEAR}.list"
   sudo apt update
   sudo apt install phoenix6
   ```

## Build Targets

The package can build LANCE 1, LANCE 2, or both.

```bash
colcon build --packages-select phoenix_ros_driver --cmake-args -DROBOT_TARGET=LANCE2
```

Valid values are:

- `LANCE1`: builds the LANCE 1 Phoenix 5 and Phoenix 6 drivers.
- `LANCE2`: builds the LANCE 2 Phoenix 6 driver.
- `ALL`: builds all supported drivers.

## Launching

The main launch file reads `config/phoenix.json` by default:

```bash
ros2 launch phoenix_ros_driver phoenix.launch.py
```

CAN helper scripts are available for bringing interfaces up and down:

```bash
./scripts/can_bringup.sh
./scripts/can_shutdown.sh
```

## `phoenix.json` Overview

`config/phoenix.json` is a launch-utils preprocessed JSON file. It contains
named driver actions, and each action can be enabled or disabled through a
preset.

At the top level:

```json
{
    "pragma:enable_preproc": true,
    "phoenix5_driver": { ... },
    "phoenix6_driver_old": { ... },
    "phoenix6_driver": { ... }
}
```

### Preprocessor Fields

`pragma:enable_preproc`

Enables launch-utils preprocessing. Keep this set to `true` for this file.

`pragma:default`

Selects the default preset for that action. A value of `null` means the action
is disabled unless a launch override selects a preset. In the current file, each
driver action has an `enabled` preset but defaults to disabled:

```json
"phoenix6_driver": {
    "pragma:default": null,
    "enabled": {
        ...
    }
}
```

To make a driver run by default, set that action's default to `"enabled"`.

## Driver Actions

### `phoenix5_driver`

Configures the LANCE 1 Phoenix 5 driver.

```json
"phoenix5_driver": {
    "pragma:default": null,
    "enabled": {
        "diagnostics_port": 1250,
        "canbus": "canable_B"
    }
}
```

Fields:

- `diagnostics_port`: TCP port for the CTRE Phoenix diagnostics server. Use a
  value greater than `0` to enable it.
- `canbus`: CAN interface name used by the Phoenix 5 driver.

### `phoenix6_driver_old`

Configures the older LANCE 1 Phoenix 6 driver.

```json
"phoenix6_driver_old": {
    "pragma:default": null,
    "enabled": {
        "diagnostics_port": 0,
        "canbus": "canable_A",
        "arduino_device": "",
        "power_cycle_fault_thresh_s": 1.5,
        "common": {
            "kP": 0.4,
            "kI": 0.05,
            "kD": 0.0001,
            "kV": 0.12,
            "neutral_deadband": 0.05,
            "neutral_brake": false,
            "stator_current_limit": 10.0,
            "supply_current_limit": 2.0,
            "voltage_limit": 12.0
        }
    }
}
```

Fields:

- `diagnostics_port`: CTRE diagnostics server port. `0` disables the server.
- `canbus`: CAN interface name.
- `arduino_device`: serial relay device path. An empty string disables the
  relay.
- `power_cycle_fault_thresh_s`: time threshold used by the old driver when
  handling power-cycle/reset fault behavior.
- `common`: shared motor configuration values used by the old hard-coded LANCE
  1 motor list.

### `phoenix6_driver`

Configures the current LANCE 2 Phoenix 6 driver.

```json
"phoenix6_driver": {
    "pragma:default": null,
    "enabled": {
        "info_pub_rate_ms": 50,
        "fault_pub_rate_ms": 250,
        "diagnostics_port": 0,
        "canbus": "canable_A",
        "motors": [
            {
                "name": "track_right",
                "can_id": 0,
                "controller": "FX",
                "follows": "",
                "kP": 0.4,
                "kI": 0.05,
                "kD": 0.0001,
                "kV": 0.12,
                "neutral_deadband": 0.05,
                "neutral_brake": false,
                "stator_current_limit": 10.0,
                "supply_current_limit": 2.0,
                "voltage_limit": 12.0
            }
        ]
    }
}
```

Top-level fields:

- `info_pub_rate_ms`: interval, in milliseconds, between `TalonInfo` publishes.
- `fault_pub_rate_ms`: interval, in milliseconds, between `TalonFaults`
  publishes.
- `diagnostics_port`: CTRE diagnostics server port. `0` disables the server.
- `canbus`: CAN interface name.
- `motors`: list of motor configuration objects.

## LANCE 2 Motor Fields

Each object in the `phoenix6_driver.enabled.motors` list defines one motor.
The launch helper flattens this list into ROS parameters:

```json
{
    "name": "hopper_act_left",
    "can_id": 4,
    "controller": "FXS",
    "follows": "",
    "sensor": "analog_pot",
    "pot_max_v": 3.3,
    "pot_inverted": true,
    "kP": 0.4,
    "kI": 0.05,
    "kD": 0.0001,
    "kV": 0.12,
    "neutral_deadband": 0.05,
    "neutral_brake": false,
    "stator_current_limit": 10.0,
    "supply_current_limit": 2.0,
    "voltage_limit": 12.0
}
```

## Required LANCE 2 Motor Fields

### `name`

Unique motor name. This becomes part of the ROS topic and parameter names. For
example, `hopper_act_left` publishes info on `lance/hopper_act_left/info` and
accepts controls on `lance/hopper_act_left/ctrl`.

### `can_id`

CAN device ID for the Talon.

### `controller`

Motor controller type.

Supported values:

- `FX`: TalonFX.
- `FXS`: TalonFXS.

## Optional LANCE 2 Motor Fields

### PID and Motor Output Fields

#### `kP`

Slot 0 proportional gain.

#### `kI`

Slot 0 integral gain.

#### `kD`

Slot 0 derivative gain.

#### `kV`

Slot 0 velocity feedforward gain.

#### `neutral_deadband`

Duty-cycle neutral deadband.

#### `neutral_brake`

When `true`, the motor uses brake mode. When `false`, it uses coast mode.

#### `stator_current_limit`

Stator current limit in amps. Values greater than `0` enable the limit.

#### `supply_current_limit`

Supply current limit in amps. Values greater than `0` enable the limit.

#### `voltage_limit`

Peak forward and reverse voltage limit. Values greater than `0` enable voltage
limiting. The reverse limit is applied as the negative of this value.

### Follower Fields

#### `follows`

Name of another configured motor to follow. Use an empty string for no follower.

Example:

```json
{
    "name": "hopper_act_right",
    "follows": "hopper_act_left"
}
```

#### `follower_type`

Optional follower mode. Defaults to `normal`.

Supported values:

- `normal`
- `DifferentialFollower`
- `CustomMechanism`

`CustomMechanism` is only valid when `sensor` is `analog_pot` or
`AnalogPotentiometer`.

On LANCE 2, `CustomMechanism` is specifically for the mechanically linked
hopper linear actuators. Configure both actuators with `follower_type` set to
`CustomMechanism`, and set each actuator's `follows` field to the other
actuator. The driver detects the reciprocal config, creates one shared pair, and
routes commands from either motor topic through one software controller. Both
motors must be `FXS` motors with analog potentiometer feedback.

For `POSITION` commands, `msg.value` is the target normalized potentiometer
position. The driver reads both potentiometers and sends separate voltage
outputs so each actuator moves toward the same target.

For `VELOCITY` commands, `msg.value` is treated as the shared velocity request.
The driver applies a shared feedforward voltage and adds a balancing correction
from the difference between the two potentiometer positions.

#### `alignment`

Optional motor alignment for follower control. Defaults to `Aligned`.

Supported values:

- `Aligned`
- `Opposed`

### Sensor Fields

#### `sensor`

Optional feedback sensor source for `FXS` motors only. The Lance 2 driver does
not configure an external feedback sensor on `FX` motors, so omit this field for
`FX` controllers.

Supported values in the LANCE 2 driver:

- `Commutation`
- `QuadratureEncoder`
- `PulseWidthEncoder`
- `RemoteCANcoder`
- `FusedCANcoder`
- `SyncCANcoder`
- `RemotePigeon2Yaw`
- `RemotePigeon2Pitch`
- `RemotePigeon2Roll`
- `AnalogPotentiometer`
- `analog_pot`

Remote, fused, and sync sensor sources are `FXS` feedback options and require
`remote_sensor_id`. This includes:

- `RemoteCANcoder`
- `FusedCANcoder`
- `SyncCANcoder`
- `RemotePigeon2Yaw`
- `RemotePigeon2Pitch`
- `RemotePigeon2Roll`

### Remote, Fused, and Sync Sensor Fields

Use this field on `FXS` motors when `sensor` is any `Remote*`, `Fused*`, or
`Sync*` sensor.

#### `remote_sensor_id`

CAN device ID of the remote sensor device used for feedback.

Example:

```json
{
    "name": "arm_joint",
    "can_id": 7,
    "controller": "FXS",
    "sensor": "RemoteCANcoder",
    "remote_sensor_id": 12
}
```

If `remote_sensor_id` is missing for a remote, fused, or sync sensor, the driver
defaults it to `-1`, logs a ROS error, and skips that motor.

### Analog Potentiometer Fields

Use these only on `FXS` motors when `sensor` is `analog_pot` or
`AnalogPotentiometer`.

#### `pot_max_v`

Maximum potentiometer voltage. The driver converts analog voltage to position as:

```text
position = analog_voltage / pot_max_v
```

This must be set to a positive value. If it is missing, the driver defaults it
to `-1.0`, logs a ROS error, and skips that motor.

#### `pot_inverted`

Whether to invert the normalized potentiometer position. Defaults to `false`.

When `true`, the driver publishes:

```text
position = 1.0 - (analog_voltage / pot_max_v)
```

## Current LANCE 2 Motors

The current `phoenix.json` defines these LANCE 2 motors:

| Name | CAN ID | Controller | Sensor | Follows | Follower Type |
| --- | ---: | --- | --- | --- | --- |
| `track_right` | 0 | `FX` | none | none | `normal` |
| `track_left` | 1 | `FX` | none | none | `normal` |
| `trencher` | 2 | `FX` | none | none | `normal` |
| `hopper_belt` | 3 | `FX` | none | none | `normal` |
| `hopper_act_left` | 4 | `FXS` | `analog_pot` | `hopper_act_right` | `CustomMechanism` |
| `hopper_act_right` | 5 | `FXS` | `analog_pot` | `hopper_act_left` | `CustomMechanism` |

Both hopper actuator potentiometers currently use:

- `pot_max_v`: `3.3`
- `pot_inverted`: `true`

## Runtime Topics

For each LANCE 2 motor named `<name>`, the driver creates:

- `lance/<name>/ctrl`: subscribes to `phoenix_ros_driver/msg/TalonCtrl`.
- `lance/<name>/info`: publishes `phoenix_ros_driver/msg/TalonInfo`.
- `lance/<name>/faults`: publishes `phoenix_ros_driver/msg/TalonFaults`.

The driver also subscribes to:

- `lance/watchdog_status`: receives watchdog state from `std_msgs/msg/Int32`.

## Phoenix 6 Status Bits

`TalonInfo.status` is a `uint8_t` bitfield:

1. Bit 0: device is enabled in hardware.
2. Bit 1: device is connected.
3. Bit 2: device has reset.
4. Bit 3: Phoenix API is enabled.
5. Bit 4: ROS watchdog is enabled.
6. Bit 5: no configuration blocks are in place.
7. Bit 6: device has been successfully configured.
8. Bit 7: device does not need its sensor position set.

## Configuration Tips

- Keep motor names unique.
- Set `follows` to another motor's `name`, not its CAN ID.
- Use `FXS` for motors that need the TalonFXS external feedback configuration.
- Use `analog_pot` only with a valid positive `pot_max_v`.
- Use current limits and voltage limits greater than `0` when you want the
  Phoenix configuration to enable those limits.
