# Lesson 6: Custom Messages and Services

**Time:** ~3–4 hours &nbsp;·&nbsp; **Prerequisite:** [Lesson 4](../04-hopper-sensor/README.md) ([Lesson 5](../05-launch-integration/README.md) is helpful for the stretch goals)

## What you'll build
Two packages:
- **`mining_stats_interfaces`** defines your own message type, `MiningStats`, and your own service type,
  `ResetStats`.
- **`mining_stats`** is a node that watches the robot mine and keeps score for the session:

| Statistic | How it's measured |
|-----------|-------------------|
| Current **state**: idle / digging / dumping | From the hopper position and trencher speed |
| Time spent digging | Adds up `dt` while digging |
| Number of dumps | Counts each time the hopper is raised to dump |
| Trencher energy used (Wh) | Power (volts × amps) × time |
| Peak trencher current (A) | Highest value seen |
| Hopper fullness | Latest value from your Lesson 4 sensor |

It publishes all of that in **one** `MiningStats` message on `/mining_stats/stats`. A service, `/mining_stats/reset`,
starts a new session and *answers* with the final stats from the old one:

```bash
$ ros2 service call /mining_stats/reset mining_stats_interfaces/srv/ResetStats "{reason: 'end of practice run'}"
mining_stats_interfaces.srv.ResetStats_Response(final_stats=mining_stats_interfaces.msg.MiningStats(header=...,
state=2, session_seconds=54.79999542236328, digging_seconds=24.199995040893555, dump_count=2,
trencher_energy_wh=28.35194969177246, trencher_peak_current_a=108.14517974853516, hopper_fullness=0.011812010779976845))
```

## Why it matters
Standard messages like `Float32` carry a single number with no meaning attached. As soon as data has *structure*
(several related values, units, named constants), you define your own type. The robot does this a lot:
- [`phoenix_ros_driver`](../../../phoenix-driver/msg/) defines `TalonInfo`, `TalonCtrl`, and `TalonFaults`, which you've
  used since Lesson 1.
- [`cardinal_perception`](../../../cardinal-perception/srv/) defines services like `UpdatePathPlanningMode`.
- `TalonCtrl.msg` has **constants** (`VELOCITY = 2` and so on), exactly as your `MiningStats` will.

**Services** are for request → response interactions: "do this and tell me the result". Mission control's Enable
button is a service call, as you saw in Lesson 0.

## Background
Read these (about 40 minutes):
- [Creating custom msg and srv files](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Writing a simple service and client (C++)](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html). Focus on the service side.

### C++ you'll use
| Concept | Where you'll use it |
|---------|---------------------|
| **Generated code** | You write a 10-line `.msg` file, and the build generates a C++ `struct` for it, with a header to `#include` |
| Nested types | `ResetStats::Request` and `ResetStats::Response` are types *inside* `ResetStats` |
| `static constexpr` class constants | `.msg` constants become `MiningStatsMsg::STATE_DIGGING` |
| `switch` on integer constants | Turning a state number into a name for logging |
| Fixed-width integers | `uint8_t`, `uint32_t`: why message fields use them |
| Edge detection | Counting a dump once when it *starts*, not on every step during it |
| `std::optional::value_or` | A default when no value has arrived yet |

---

## Setup
```bash
cd ~/code/lance-ws
cp -r src/training/lessons/06-mining-stats/starter/* src/training/sandbox/
```
This copies **two** packages. Make sure your Lesson 4 `hopper_sensor_sim` is also in `sandbox/`. The stats node reads
its output. (If you skipped Lesson 4, copy its `solution/hopper_sensor_sim` instead.)

---

## Part 1: Define the interfaces
Interfaces live in their own package. That way, any other package, including Python ones or tools on the driver's
laptop, can use your message types without depending on your node's code.

### 1a. The message
Create `mining_stats_interfaces/msg/MiningStats.msg`. Field syntax is `type name`, and constants are `type NAME = value`:
```
# Statistics for the current mining session (since the last reset).

# What the robot is doing right now
uint8 STATE_IDLE = 0      # hopper not lowered or raised, or trencher not spinning
uint8 STATE_DIGGING = 1   # hopper lowered and trencher spinning
uint8 STATE_DUMPING = 2   # hopper raised for offload

std_msgs/Header header

uint8 state

float32 session_seconds          # time since the last reset
float32 digging_seconds          # total time spent in STATE_DIGGING
uint32 dump_count                # times the hopper has been raised to dump

float32 trencher_energy_wh       # energy drawn by the trencher, in watt-hours
float32 trencher_peak_current_a  # highest trencher supply current seen, in amps

float32 hopper_fullness          # latest reading from lance/hopper_fullness (0 to 1)
```
Notice the **units in the field names** (`_seconds`, `_wh`, `_a`). A message is a contract with everyone who reads it,
so make it hard to misunderstand.

### 1b. The service
Create `mining_stats_interfaces/srv/ResetStats.srv`. A `---` line separates the **request** (above) from the
**response** (below). One of your own messages can be a field:
```
# Clear the mining statistics and start a new session.

string reason             # why the stats were reset (written to the log)
---
MiningStats final_stats   # the statistics just before they were cleared
```

### 1c. Package setup
Fill in the TODOs in `mining_stats_interfaces/package.xml`:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>std_msgs</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
and in `CMakeLists.txt`:
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MiningStats.msg"
    "srv/ResetStats.srv"
    DEPENDENCIES std_msgs)   # MiningStats uses std_msgs/Header

ament_export_dependencies(rosidl_default_runtime)
```
Delete the placeholder `README.md` files in `msg/` and `srv/`.

✅ **Checkpoint:**
```bash
colcon build --symlink-install --packages-select mining_stats_interfaces
source install/setup.bash
ros2 interface show mining_stats_interfaces/srv/ResetStats
```
This prints your service, with `MiningStats` expanded inside it. Now look at what was generated:
`install/mining_stats_interfaces/include/mining_stats_interfaces/mining_stats_interfaces/msg/detail/mining_stats__struct.hpp`.
Find your fields and your `STATE_` constants in the C++ `struct`. You never edit these files. They're rebuilt from the
`.msg` file every time.

---

## Part 2: Use the interfaces from the node package
Do the "three places" from Lesson 1 for `mining_stats_interfaces`: `package.xml`, `CMakeLists.txt` (`find_package` plus
`${mining_stats_interfaces_TARGETS}`), and the includes in the `.cpp`:
```cpp
#include <mining_stats_interfaces/msg/mining_stats.hpp>   // MiningStats.msg → mining_stats.hpp
#include <mining_stats_interfaces/srv/reset_stats.hpp>

using MiningStatsMsg = mining_stats_interfaces::msg::MiningStats;
using ResetStatsSrv = mining_stats_interfaces::srv::ResetStats;
```

✅ **Checkpoint:** Build **both** packages:
```bash
colcon build --symlink-install --packages-select mining_stats_interfaces mining_stats
```
colcon builds the interface package first, because `mining_stats` depends on it. That's the `<depend>` doing its job.

---

## Part 3: Compute and publish the stats
The starter already stores the latest trencher, actuator, and fullness messages, and works out `dt` every 100 ms.
That's the same pattern as Lesson 4.

### 3a. Classify the state
Write `uint8_t classifyState(double actuator_pos, double trencher_rps) const`:
- Hopper above `dump_above_pos` → `MiningStatsMsg::STATE_DUMPING`
- Hopper below `dig_below_pos` **and** `|trencher_rps| > trencher_min_rps` → `STATE_DIGGING`
- Otherwise → `STATE_IDLE`

The constants from your `.msg` file are now `static constexpr` members of the generated struct.

### 3b. Track the state and count dumps
Keep a `uint8_t state` member. In `step()`, classify the new state. **If it changed**:
- log it: `State: IDLE -> DIGGING`. Write a small `stateName(uint8_t)` helper with a `switch` that returns
  `"IDLE"`, `"DIGGING"`, `"DUMPING"`, or `"UNKNOWN"` in its `default:` case.
- if the new state is `STATE_DUMPING`, add one to `dump_count`.

Why only on a *change*? `step()` runs 10 times per second. If you counted "is dumping" on every step, one dump would
count as about 30. Reacting to the moment something *becomes* true is called **edge detection**, and you'll use it all the
time. Button presses work the same way. Look for `wasPressed` in
[`mining_controller.cpp`](../../../lance/src/robot/control/shared/mining_controller.cpp) and
[`advanced_controls.cpp`](../../../lance/src/mission_control/advanced_controls.cpp).

### 3c. Accumulate
- `session_seconds += dt`, and `digging_seconds += dt` while digging.
- Energy: power (W) = `bus_voltage` × `|supply_current|`. Energy (Wh) = power × hours, so add `power_w * (dt / 3600.0)`.
- Peak current: `std::max` of the current peak and `|supply_current|`.

### 3d. Publish
Create a publisher on `"~/stats"`. The `~/` puts it under the node's name, giving `/mining_stats/stats`. Write
`MiningStatsMsg buildStatsMsg() const`, which fills in every field (including `header.stamp = this->now()`), and publish it
at the end of `step()`. If no fullness reading has arrived yet, use `hopper_fullness.value_or(NAN)`.

> [!NOTE]
> Message fields are `float` (`float32`), while your members are `double`. Converting needs an explicit
> `static_cast<float>(...)`, so the narrowing is visible in the code.

✅ **Checkpoint:** Run the simulator (or the motor sim setup from Lesson 1), your Lesson 4 sensor with `source:=sim`, and
your stats node (`--ros-args -p use_sim_time:=true` in Gazebo). Then:
```bash
ros2 topic echo /mining_stats/stats
```
Dig (hopper down, trencher on), then dump (hopper all the way up). Watch `state` change, `digging_seconds` grow, and
`dump_count` go up by exactly **one** per dump.

---

## Part 4: The reset service
### 4a. Create the service
```cpp
this->reset_srv = this->create_service<ResetStatsSrv>(
    "~/reset",
    [this](const std::shared_ptr<ResetStatsSrv::Request> request,
           std::shared_ptr<ResetStatsSrv::Response> response)
    {
        this->onReset(*request, *response);
    });
```
Store it in a member (`rclcpp::Service<ResetStatsSrv>::SharedPtr`), for the same reason as subscriptions.

ROS gives your callback the request and an empty response to fill in. When the callback returns, ROS sends the response
back to whoever called. You don't call `publish` or `send` yourself.

### 4b. Write `onReset`
1. Fill `response.final_stats` using `buildStatsMsg()` **before** clearing anything.
2. Log the reason, and a short summary of the final stats.
3. Reset every statistic to zero. Should you reset `state` too? (The robot is still doing whatever it was doing.)

✅ **Checkpoint:**
```bash
ros2 service call /mining_stats/reset mining_stats_interfaces/srv/ResetStats "{reason: 'end of practice run'}"
```
The response contains the stats you'd built up. The next message on `/mining_stats/stats` starts from zero.

---

## Part 5: Foxglove
- Plot `/mining_stats/stats.digging_seconds` and `/mining_stats/stats.trencher_energy_wh` while you mine. (If
  `foxglove_bridge` was already running before you built your interface package, restart the simulation so the bridge
  can load your new message type.)
- Try adding a **Call Service** panel for `/mining_stats/reset`, like the Enable button in the team's layout,
  with the request `{"reason": "foxglove button"}`. Then you have a one-click "new session" button.

✅ **Done!** Run two dig-and-dump cycles, reset from Foxglove or the terminal, and share a screenshot of the final stats.

---

## Check your understanding
1. Why put the `.msg`/`.srv` files in a *separate* package? Now look at how
   [`cardinal-perception`](../../../cardinal-perception/CMakeLists.txt) does it (search for `rosidl_generate_interfaces`):
   it defines interfaces in the *same* package as its code. What's the trade-off? (The ROS docs page
   [Implementing custom interfaces](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/Working-with-interfaces/Single-Package-Define-And-Use-Interface.html)
   shows that approach.)
2. When should something be a **topic**, and when a **service**? Why is "reset the stats" a service, while the stats themselves are a topic?
3. Your `digging_seconds` will probably come out a bit *higher* than the time you held the trencher trigger. Think about
   what happens while the trencher spins down with the hopper still lowered. Is that a bug, or a reasonable definition of
   "digging"? How would you change it if you disagreed?
4. You add a field to `MiningStats.msg` and rebuild your node, but a teammate's Foxglove panel or a bag recorded
   yesterday still has the old version. What could go wrong? Why do teams try hard not to change messages that are
   already in use?
5. The service callback and `step()` both change the statistics. Could they run at the same time and corrupt them?
   (Hint: the executor, from Lesson 1.)

## Stretch goals
1. **Launch it.** Add a `mining_stats` action with a `sim` preset (with `use_sim_time`) to the launch config, as in
   Lesson 5.
2. **A C++ client.** Write a tiny node that calls `/mining_stats/reset` every 5 minutes. Use `create_client`,
   `wait_for_service`, and `async_send_request`, and log the response when it arrives. Why must the response be handled
   *asynchronously* inside a node?
3. **Dump efficiency.** Add `float32 fullness_at_last_dump` to the message: the hopper fullness when the last dump
   *started*. (You'll need to rebuild **both** packages. Why?)
4. **Average power.** Add the average trencher power while digging, in watts, to the message. Work out what extra
   member you need to store.
5. **Session summary service.** Add a second service, `GetStats.srv`, with an empty request and the stats as the
   response. What's the difference between that and just echoing the topic once?

## Reference solution
[`solution/mining_stats_interfaces/`](solution/mining_stats_interfaces/) and [`solution/mining_stats/`](solution/mining_stats/).

**Next:** [Lesson 7: Click to drive (capstone)](../07-click-to-drive/README.md)
