# LANCE 2 Phoenix Driver Motor Test Plan

Target date: April 19, 2026

This plan is for the LANCE 2 Phoenix 6 driver and the current
`config/phoenix.json` setup. The goal is to verify that the driver launches,
configures the motors, publishes status, accepts control messages, and that the
LANCE 2 `CustomMechanism` hopper actuator control behaves safely.

## Safety Setup

- Put the robot in a mechanically safe state before enabling motor output.
- Keep the e-stop/watchdog disable path ready.
- Start with tracks off the ground or otherwise unable to drive the robot.
- Keep the hopper actuators unloaded if possible.
- Use small voltage commands first. Do not start with position commands.
- Have one person watching the mechanism and one person at the terminal if
  possible.

Stop immediately if:

- A motor moves the wrong direction.
- A hopper actuator moves while the other does not.
- The hopper actuators fight each other mechanically.
- The analog potentiometer position jumps, wraps, or moves opposite of what is
  expected.
- The driver repeatedly logs CTRE config or control failures while hardware is
  connected.

## Terminal Setup

Open a terminal at the workspace root:

```bash
cd ~/code/lance-ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select phoenix_ros_driver --cmake-args -DBUILD_LANCE2=ON
source install/setup.bash
```

For each new terminal, run:

```bash
cd ~/code/lance-ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 1. Confirm Config

Check `src/phoenix-driver/config/phoenix.json`.

Confirm:

- `phoenix6_driver` has the `enabled` preset.
- `canbus` matches the connected CAN adapter.
- The current expected CAN bus is:

```json
"canbus": "canable_A"
```

- `hopper_act_left` and `hopper_act_right` are both `FXS`.
- Both hopper actuators use:

```json
"follower_type": "CustomMechanism"
"sensor": "analog_pot"
"pot_max_v": 3.3
"pot_inverted": true
```

- Each hopper actuator follows the other:

```json
"hopper_act_left" follows "hopper_act_right"
"hopper_act_right" follows "hopper_act_left"
```

## 2. Launch Driver

Run:

```bash
ros2 launch phoenix_ros_driver phoenix.launch.py phoenix6_driver:=enabled
```

Expected:

- The driver starts.
- The launch output is not filled with full config dumps.
- You see:

```text
Configured Lance 2 CustomMechanism pair ...
Driver node (Phoenix6) has started
```

Not expected when hardware is connected correctly:

```text
Failed to apply config for motor ...
Could not transmit CAN Frame.
CAN frame not received/too-stale.
```

If those appear, check CAN power, CAN termination, device IDs, and `canbus`.

## 3. Confirm Topics

In a second terminal:

```bash
ros2 topic list | grep lance
```

Expected topics include:

```text
/lance/track_right/ctrl
/lance/track_right/info
/lance/track_right/faults
/lance/track_left/ctrl
/lance/trencher/ctrl
/lance/hopper_belt/ctrl
/lance/hopper_act_left/ctrl
/lance/hopper_act_left/info
/lance/hopper_act_right/ctrl
/lance/hopper_act_right/info
/lance/watchdog_status
```

## 4. Watchdog Disable/Enable

First publish disabled:

```bash
ros2 topic pub --once /lance/watchdog_status std_msgs/msg/Int32 "{data: 0}"
```

Expected:

- Motors are neutral.
- No motor should move.
- Any active `CustomMechanism` position or velocity command is cleared.

Then publish enabled:

```bash
ros2 topic pub --once /lance/watchdog_status std_msgs/msg/Int32 "{data: 100000}"
```

Expected:

- The driver is enabled by the Phoenix unmanaged feed.
- No motor should move until a fresh control command is sent.

## 5. Verify Info Topics

Check a normal motor:

```bash
ros2 topic echo /lance/track_right/info
```

Expected:

- Messages publish continuously.
- `status` changes reasonably when watchdog state changes.
- Bus voltage and current values look believable.

Check hopper actuator analog potentiometers:

```bash
ros2 topic echo /lance/hopper_act_left/info
ros2 topic echo /lance/hopper_act_right/info
```

Expected:

- `position` is roughly normalized from `0.0` to `1.0`.
- Both pots move in the expected direction.
- Both pots are stable when the mechanism is not moving.

Do not run position control until both pot readings look correct.

## 6. Test Unsupported Mode Logging

This is a quick software sanity check.

Normal motor:

```bash
ros2 topic pub --once /lance/track_right/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 3, value: 0.0}"
```

Expected log:

```text
Motor track_right does not support control mode 3
```

Custom mechanism:

```bash
ros2 topic pub --once /lance/hopper_act_left/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 3, value: 0.0}"
```

Expected log:

```text
CustomMechanism does not support control mode 3
```

## 7. Test Individual Normal Motors

Use voltage mode first with small values.

Hopper belt example:

```bash
ros2 topic pub --once /lance/hopper_belt/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 4, value: 1.0}"
```

Stop:

```bash
ros2 topic pub --once /lance/hopper_belt/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 15, value: 0.0}"
```

Expected:

- The motor moves gently for the voltage command.
- The motor stops on `DISABLED`.
- Direction matches the robot expectation.

Repeat carefully for:

- `track_right`
- `track_left`
- `trencher`
- `hopper_belt`

Keep the voltage low until directions are confirmed.

## 8. Test CustomMechanism Manual Mirrored Output

Start with the hopper actuators mechanically safe and watched closely.

Send a very small voltage command to either hopper actuator topic:

```bash
ros2 topic pub --once /lance/hopper_act_left/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 4, value: 0.5}"
```

Stop:

```bash
ros2 topic pub --once /lance/hopper_act_left/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 15, value: 0.0}"
```

Expected:

- Both hopper actuators receive the mirrored command.
- Both move in the same intended mechanism direction.
- Neither actuator fights the other.

Repeat from the other side:

```bash
ros2 topic pub --once /lance/hopper_act_right/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 4, value: 0.5}"
ros2 topic pub --once /lance/hopper_act_right/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 15, value: 0.0}"
```

Expected:

- Commanding either topic controls the shared mechanism pair.

## 9. Test CustomMechanism Position Control

Only do this after analog pot readings and manual mirrored output are correct.

Watch both info topics while testing.

Pick a target close to the current position. Example if both are near `0.45`,
try `0.47`:

```bash
ros2 topic pub --once /lance/hopper_act_left/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 1, value: 0.47}"
```

Expected:

- Both actuators move toward the same normalized position.
- The actuator that is farther from the target gets a different output than the
  other.
- The two positions stay close together.

Stop:

```bash
ros2 topic pub --once /lance/hopper_act_left/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 15, value: 0.0}"
```

Then test watchdog clearing:

```bash
ros2 topic pub --once /lance/hopper_act_left/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 1, value: 0.50}"
ros2 topic pub --once /lance/watchdog_status std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /lance/watchdog_status std_msgs/msg/Int32 "{data: 100000}"
```

Expected:

- The watchdog disable stops the actuators.
- After re-enable, the old position command does not resume.
- A fresh position command is required to move again.

## 10. Test CustomMechanism Velocity Control

Only do this after position control behaves safely.

Use a very small velocity request:

```bash
ros2 topic pub --once /lance/hopper_act_left/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 2, value: 0.1}"
```

Stop:

```bash
ros2 topic pub --once /lance/hopper_act_left/ctrl phoenix_ros_driver/msg/TalonCtrl "{mode: 15, value: 0.0}"
```

Expected:

- Both actuators move together.
- The driver applies a shared feedforward and a correction based on position
  difference.
- Positions remain close.

## 11. Pass Criteria

The test is successful if:

- Driver launches with `phoenix6_driver:=enabled`.
- No config apply failures appear with hardware connected.
- All expected topics exist.
- Unsupported modes log clear ROS errors.
- Watchdog disable stops motors and clears `CustomMechanism` active commands.
- Normal motors respond to small voltage commands and stop on disabled.
- Hopper actuator analog pot positions are sane.
- `CustomMechanism` mirrored output moves both actuators together.
- `CustomMechanism` position control moves both actuators toward the same target.

## Notes To Capture

Record these during the test:

- Actual CAN adapter name used.
- Any CTRE startup errors.
- Pot position range for both hopper actuators.
- Whether `pot_inverted` is correct for both actuators.
- Safe initial position targets tested.
- Any gain changes needed for `kP` or `kV`.
