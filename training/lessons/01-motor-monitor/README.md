# Lesson 1: Motor Health Monitor

**Time:** ~3–4 hours &nbsp;·&nbsp; **Prerequisite:** [Lesson 0](../00-drive-the-robot/README.md)

## What you'll build
You'll write a ROS 2 node in C++ called `motor_monitor`. It:
- listens to the telemetry from every LANCE motor
- keeps track of each motor's current draw, including the highest it has seen
- prints a **warning** when a motor draws more current than a limit you choose
- prints a status table every 2 seconds
- publishes the **total current** of all motors so you can plot it live in Foxglove

When you're done, spinning the trencher in the simulator will look like this:

```
[WARN] [motor_monitor]: trencher is drawing 105.1 A (limit 80.0 A)
[INFO] [motor_monitor]:
  motor          msgs   enabled   vel(rps)   I(A)   peak I(A)   bus(V)
  track_left      790   yes        123.13   67.27     145.79    22.28
  track_right     790   yes          0.00    0.00       0.00    22.28
  trencher        790   yes        153.91  105.11     136.21    22.28
  hopper_belt     790   yes          0.00    0.00       0.00    22.28
  hopper_act      790   yes          0.00    0.00       0.00    22.28
```

## Why it matters
Motors that draw too much current overheat, trip breakers, and brown out the rest of the robot. The real
robot code watches for this. [`StallAnalyzer`](../../../lance/src/robot/core/stall_analyzer.hpp) uses the same
`TalonInfo` messages you'll use here to detect stalled tracks and a jammed trencher. The node you write
follows the same pattern as the robot's own nodes: subscriptions, publishers, parameters, and timers inside
a class.

## Background
Read these before starting (about 30 minutes total):
- [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). Skim it. You don't need to do it, because this lesson covers the same ideas.
- [Using parameters in a class (C++)](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
- [Quality of Service settings](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/topics/About-Quality-of-Service-Settings.html). Just read the "QoS policies" and "QoS compatibilities" sections.

### C++ you'll use
If any of these are new, look them up on [learncpp.com](https://www.learncpp.com/) as you go:

| Concept | Where you'll use it |
|---------|---------------------|
| Classes and inheritance (`class A : public B`) | Your node *is a* `rclcpp::Node` |
| `struct` | Grouping the stats for one motor |
| `std::vector`, range-based `for` loops | Storing one entry per motor |
| `std::shared_ptr` | ROS gives you publishers, subscriptions, and timers as shared pointers |
| Lambdas (`[this](...) { ... }`) | Callbacks that run when a message arrives or a timer fires |
| References (`const T&`) | Reading messages without copying them |

### The message you'll read
Every motor publishes a [`phoenix_ros_driver/msg/TalonInfo`](../../../phoenix-driver/msg/TalonInfo.msg) on
`/lance/<motor_name>/info`, 50 times per second in the simulator. The fields you'll use are:

| Field | Type | Meaning |
|-------|------|---------|
| `velocity` | `float64` | Motor speed in rotations per second |
| `supply_current` | `float32` | Current drawn from the battery, in amps |
| `bus_voltage` | `float32` | Battery voltage as seen by the motor controller |
| `status` | `uint8` | Bit flags. `status & STATUS_PHX_ENABLED` is non-zero when the motor is enabled |

The motor names are `track_left`, `track_right`, `trencher`, `hopper_belt`, and `hopper_act` (the hopper's
linear actuator).

> [!NOTE]
> The simulator reports `device_temp` as `0`, which is why this lesson watches current instead of temperature.

---

## Setup

### 1. Copy the starter package into your sandbox
```bash
cd ~/code/lance-ws
cp -r src/training/lessons/01-motor-monitor/starter/motor_monitor src/training/sandbox/
```

Open `src/training/sandbox/motor_monitor/` in VS Code. A ROS 2 C++ package is three things:

```
motor_monitor/
├── package.xml                  ← what this package is and what it depends on (for ROS tools)
├── CMakeLists.txt               ← how to compile it (for CMake)
└── src/motor_monitor_node.cpp   ← your code
```

### 2. Pick something to listen to
Your node needs motors to monitor. Pick **one** option and start it in its own terminal.

**Option A: full simulation (recommended).** You get Gazebo, Foxglove, and controller driving:
```bash
./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled
```
Remember to click **Enable** in Foxglove before driving.

**Option B: motor simulator only.** Use this if Gazebo is slow on your laptop. Run each command in its own terminal, and source the workspace in each one:
```bash
# Terminal 1: the motor simulator by itself
source install/setup.bash
ros2 run lance lance2_motor_sim

# Terminal 2: pretend mission control is enabling the robot
source install/setup.bash
ros2 topic pub -r 10 /lance/watchdog_status std_msgs/msg/Int32 "{data: 250}"

# Terminal 3 (when you want current to flow): run the trencher at 100%
source install/setup.bash
ros2 topic pub -r 20 --qos-reliability best_effort /lance/trencher/ctrl \
    phoenix_ros_driver/msg/TalonCtrl "{mode: 0, value: 1.0}"
```
Press `Ctrl` + `C` in terminal 3 to stop the trencher. Change `trencher` to `track_left` (and `1.0` to `0.8`) to
spin a track instead. `mode: 0` means "percent output". Look at
[`TalonCtrl.msg`](../../../phoenix-driver/msg/TalonCtrl.msg) for the other modes.

---

## Step 1: Build and run the starter
In a **new terminal**:
```bash
cd ~/code/lance-ws
colcon build --symlink-install --packages-select motor_monitor
source install/setup.bash
ros2 run motor_monitor motor_monitor_node
```

You'll repeat this **build → source → run** cycle after every step. `--packages-select` builds only your
package, which takes seconds instead of the many minutes a full build takes.

✅ **Checkpoint:** You see `[INFO] [motor_monitor]: Motor monitor started!`. Stop it with `Ctrl` + `C`.

**Read the starter code now.** Find `main()`. It starts ROS (`rclcpp::init`), creates your node, and then
**spins**: it waits for messages and timer events and runs your callbacks, until you press `Ctrl` + `C`.
Your node never calls its own callbacks. `spin()` calls them for you.

---

## Step 2: Subscribe to one motor
Goal: print the left track's velocity every time a message arrives.

### 2a. Add the dependency, in three places
Using a message from another package always takes the same three edits. If you miss one, you get a
compile error or a link error. This is the most common "why won't it build" problem in ROS.

1. **`package.xml`**: tell ROS tools you depend on the package:
   ```xml
   <depend>phoenix_ros_driver</depend>
   ```
2. **`CMakeLists.txt`**: find the package, then link it to your executable:
   ```cmake
   find_package(phoenix_ros_driver REQUIRED)
   ...
   target_link_libraries(motor_monitor_node
       rclcpp::rclcpp
       ${phoenix_ros_driver_TARGETS})
   ```
3. **Your `.cpp`**: include the header. Message `TalonInfo` in package `phoenix_ros_driver` becomes:
   ```cpp
   #include <phoenix_ros_driver/msg/talon_info.hpp>

   using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;  // a shorter name to type
   ```

### 2b. Create the subscription
In the constructor:
```cpp
this->track_left_sub = this->create_subscription<TalonInfoMsg>(
    "lance/track_left/info",    // topic name
    10,                         // QoS (we'll come back to this...)
    [this](const TalonInfoMsg& msg)
    {
        RCLCPP_INFO(this->get_logger(), "velocity: %.2f", msg.velocity);
    });
```

And as a member variable, in the `private:` section at the bottom of the class:
```cpp
rclcpp::Subscription<TalonInfoMsg>::SharedPtr track_left_sub;
```

The third argument is a **lambda**, an unnamed function written inline. `[this]` lets it use your node's
members (like `get_logger()`). ROS calls it once for every message that arrives.

> [!IMPORTANT]
> You **must** store the subscription in a member variable. `create_subscription` returns a shared
> pointer. If nothing holds on to it, the subscription is destroyed at the end of the constructor and you
> silently stop receiving messages. The same is true for publishers and timers.

Build, source, and run, with your simulator from Setup running.

### 2c. Debug: why is nothing printing? 🔍
If you did everything above, **no velocities print.** That's on purpose. This is one of the most common ROS
bugs, so it's worth finding yourself. Look carefully at your node's output and try:

```bash
ros2 topic info -v /lance/track_left/info
```

<details>
<summary>Hint 1</summary>

Look for a `[WARN]` near the top of your node's output. It mentions `incompatible QoS`.
</details>

<details>
<summary>Hint 2</summary>

Compare the `Reliability` of the publisher (`motor_sim` or the phoenix driver) with your subscription.
Passing `10` as the QoS means "keep the last 10 messages, **reliable** delivery".
</details>

<details>
<summary>Explanation</summary>

The motor publishers use **best effort** reliability: send the data and don't retry if it's lost.
That's the right choice for sensor data arriving 50 times per second, because a lost message is
replaced 20 ms later. Your subscription asked for **reliable** delivery, which a best-effort publisher
can't promise, so ROS refuses to connect them:

```
New publisher discovered on topic '/lance/track_left/info', offering incompatible QoS.
No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

The fix is to replace `10` with `rclcpp::SensorDataQoS()`, a preset meant for sensor streams.
You'll see `rclcpp::SensorDataQoS{}` used for subscriptions throughout the LANCE code.
</details>

✅ **Checkpoint:** Velocities scroll past. In Option A, drive forward and watch them change. In Option B, publish to
`/lance/track_left/ctrl`.

---

## Step 3: Monitor every motor
One hard-coded subscription doesn't scale. Make the list of motors a **parameter**, and create one
subscription per motor.

### 3a. A struct for each motor's stats
Above the class, define a `struct MotorStats` with these fields:

| Field | Type | Starts at |
|-------|------|-----------|
| `name` | `std::string` | |
| `supply_current` | `float` | `0` |
| `peak_supply_current` | `float` | `0` |
| `bus_voltage` | `float` | `0` |
| `velocity` | `double` | `0` |
| `enabled` | `bool` | `false` |
| `msg_count` | `std::size_t` | `0` |

Give each field a default value with braces, e.g. `float supply_current{0.f};`.

### 3b. Declare the parameter
```cpp
const auto motor_names = this->declare_parameter<std::vector<std::string>>(
    "motors",
    {"track_left", "track_right", "trencher", "hopper_belt", "hopper_act"});
```
`declare_parameter` returns the value, which is the default unless someone overrides it at launch.

### 3c. One subscription per motor
Add two members: a `std::vector<MotorStats> motors;` and a
`std::vector<rclcpp::Subscription<TalonInfoMsg>::SharedPtr> info_subs;`.
Then, in the constructor:

1. Resize `motors` to match `motor_names`, and set each `name`.
2. Loop over the motors. For each one, build the topic string (`"lance/" + name + "/info"`), create the
   subscription, and `push_back` it into `info_subs`.
3. Have every lambda call **one** member function, `onMotorInfo(i, msg)`, where `i` is the motor's index.
   Capture the index: `[this, i](const TalonInfoMsg& msg) { this->onMotorInfo(i, msg); }`.
4. Write `void onMotorInfo(std::size_t i, const TalonInfoMsg& msg)`. Look up `this->motors[i]` and update
   every field, including `peak_supply_current` and `msg_count`.

> [!TIP]
> Why capture an **index** instead of a reference or pointer to the `MotorStats`? If a
> `std::vector` grows, it can move its elements to new memory, and old references then point to garbage.
> An index is always valid. (We resize first, so ours won't grow, but capturing an index is the safer habit.)

Temporarily print one line per message in `onMotorInfo` to check it works, then remove it. Five motors at
50 Hz is too much to read.

✅ **Checkpoint:** Run with an override to watch only two motors:
```bash
ros2 run motor_monitor motor_monitor_node --ros-args -p "motors:=[trencher, track_left]"
```

---

## Step 4: Warn on high current
1. Declare a `double` parameter `current_warn_amps` with a default of `80.0`, and store it in a member variable.
2. In `onMotorInfo`, if the motor's `supply_current` is over the limit, log a warning.

First, try a plain `RCLCPP_WARN(...)` and spin the trencher. What happens to your terminal?

Then replace it with a throttled version, which prints at most once every 1000 ms:
```cpp
RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    1000,
    "%s is drawing %.1f A (limit %.1f A)",
    m.name.c_str(), m.supply_current, this->current_warn_amps);
```
`%s` needs a C string, so `std::string`s need `.c_str()`.

✅ **Checkpoint:** Spinning the trencher at full output prints a warning about once per second. (At 100% output it draws
about 105 A in the motor simulator.) Gentle driving shouldn't trigger it. If you never see a warning, use the status
table from Step 5, or `ros2 topic echo /lance/trencher/info --field supply_current`, to see real values, then
try a lower limit: `--ros-args -p current_warn_amps:=50.0`.

---

## Step 5: Publish the total current and print a report
### 5a. Publisher and timer
1. Add `#include <std_msgs/msg/float32.hpp>`. Remember the three places from Step 2a: `std_msgs` needs
   adding to `package.xml` and `CMakeLists.txt` (`${std_msgs_TARGETS}`) too.
2. Create a publisher of `std_msgs::msg::Float32` on `motor_monitor/total_current`.
3. Create a timer that runs every 100 ms:
   ```cpp
   this->publish_timer = this->create_wall_timer(100ms, [this]() { this->publishTotal(); });
   ```
   (`100ms` works because of the `using namespace std::chrono_literals;` line at the top.)
4. Write `publishTotal()`: add up `supply_current` over all motors, put the sum in a `Float32` message's
   `data` field, and `publish` it.

### 5b. Status report
Add a second timer, every 2 s, that calls `printReport()`. It should log one table with a line per motor,
like the output at the top of this lesson. `std::snprintf` with a fixed-width format like
`"%-12s %6zu %9.2f"` lines up columns nicely. Build the whole table into one `std::string` and log it
once, so lines from other log messages don't get mixed into it.

✅ **Checkpoint:** In another terminal, run
`ros2 topic echo /motor_monitor/total_current` while you drive. The number goes up when motors are working.

---

## Step 6: See it in Foxglove 📈
If you used Option B, start the Foxglove bridge in another terminal first:
`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`.

1. In Foxglove, add a new **Plot** panel.
2. Add these series:
   - `/motor_monitor/total_current.data`
   - `/lance/trencher/info.supply_current`
   - `/lance/track_left/info.supply_current`
3. Drive around and run the trencher. Watch how the total relates to the individual motors, and notice the
   spike every time a motor **starts**.

✅ **Done!** Take a screenshot of your plot and your terminal showing a warning, and share it with your mentor.

---

## Check your understanding
Talk these through with your mentor:
1. What would happen if you forgot to store a subscription in a member variable? Why?
2. Why do the motors publish with *best effort* instead of *reliable*? When *would* you want reliable?
3. Your callbacks and timers all touch `motors`. Can two of them run at the same time? (Hint: what kind of
   executor does `rclcpp::spin` use?) What would change if they could?
4. Look at [`control_node.cpp`](../../../lance/src/control_node.cpp) around line 105. How does the real
   robot controller subscribe to motor info? How is it like your code, and how is it different?
5. What is the peak current right when a motor starts, compared with when it runs steadily? Why might that
   matter for the real robot's battery and breakers?

## Stretch goals
1. **Live tuning.** Run `ros2 param set /motor_monitor current_warn_amps 10.0` while your node runs. It
   says "successful", but nothing changes. Why? Fix it with `add_on_set_parameters_callback`.
2. **Per-motor throttling.** `RCLCPP_WARN_THROTTLE` throttles *per line of code*, not per motor. If the
   trencher and a track are both over the limit, you'll only get about one warning per second between them. Track
   the last warning time per motor in `MotorStats` instead.
3. **Stale data detection.** On the real robot, a loose CAN cable means a motor stops reporting. Warn if
   any motor hasn't sent a message in 0.5 s. (Use `this->now()` and store an `rclcpp::Time` per motor.)
4. **Rolling average.** A short spike when starting shouldn't trigger a warning. Warn on the average over
   the last 0.5 s instead. (`std::deque` is handy for this.)
5. **Python comparison.** Write the Step 2 version in Python with `rclpy`. Look at
   [`hopper_fullness.py`](../../../lance/scripts/hopper_fullness.py) for an example. Which do you prefer, and why does
   the robot's main code use C++?

## Reference solution
[`solution/motor_monitor/`](solution/motor_monitor/). Try to finish on your own first, and use the
solution to compare approaches afterward. There's more than one right answer.

**Next:** [Lesson 2: Unit testing the stall analyzer](../02-unit-tests/README.md)
