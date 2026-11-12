# Lesson 4: Port a Python Node to C++

**Time:** ~4 hours &nbsp;·&nbsp; **Prerequisites:** [Lesson 1](../01-motor-monitor/README.md) and [Lesson 2](../02-unit-tests/README.md) ([Lesson 3](../03-breadcrumbs/README.md) helps, for `std::optional` and exceptions)

## What you'll build
The real robot measures how full its hopper is with a sensor that talks to the computer over a serial (USB)
cable. The sensor is a set of light-sensor bars run by an Arduino
([code on GitHub](https://github.com/Cardinal-Space-Mining/Arduino-Hopper-Fullness-Sensor)), which sends lines like
`Fs: 0.47`, meaning 47% full. The driver that reads them is a small **Python** node,
[`lance/scripts/hopper_fullness.py`](../../../../lance/scripts/hopper_fullness.py), which publishes the fraction
(0.0 = empty, 1.0 = full) on `lance/hopper_fullness`. In the simulator, nothing publishes that topic, so its line in Foxglove's hopper plot is
empty.

You'll write `hopper_sensor_sim`, a **C++ port** of that driver that can get its data from two sources:
- **`source:=file`** replays a saved log of the sensor's serial output, one line at a time, like the Python node
  reading the serial port.
- **`source:=sim`** simulates the hopper: it fills up while the trencher digs and empties while the belt dumps.

When you're done, you'll mine and dump in the simulator and watch the hopper fullness rise and fall in
Foxglove, next to the robot's own *estimate* of how full the hopper is.

## Why it matters
- The team writes nodes in both languages. Reading one and writing the other is an everyday skill.
- **Parsing** messy data from hardware (serial lines, network packets, files) is a classic source of crashes.
  You'll write a parser that rejects garbage safely, and prove it with tests.
- Simulating a sensor is how you test code that *uses* the sensor without the robot.
- **Integrating over time** ("rate × time = change") is how the robot estimates things it can't measure directly.
  [`collection_state.cpp`](../../../../lance/src/robot/core/collection_state.cpp) estimates hopper volume this way.

> [!WARNING]
> Your node publishes on the same topic as the real sensor. **Never run it on a network with the real robot.**
> If you might be at the lab, run `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in every terminal first.

## Background
### Python → C++ cheat sheet
| Python | C++ | Notes |
|--------|-----|-------|
| `self` | `this` | |
| `None` | `std::nullopt` (with `std::optional<T>`) | |
| `try:` / `except:` | `try { } catch (const std::exception& e) { }` | Catch specific types. Avoid Python's bare `except:` |
| `raise ValueError("...")` | `throw std::invalid_argument("...")` | |
| `float(text)` | `std::stof(text)` (throws on error) or `std::from_chars` (returns an error code) | |
| `"Fs: " in line` | `line.find("Fs: ") != std::string::npos` | |
| `line.startswith("Fs: ")` | `line.starts_with("Fs: ")` | C++20 |
| `line.strip()` | *(write it yourself)* | |
| `f"{x:.2f}"` | `snprintf(buf, size, "%.2f", x)` | |
| `rclpy.qos.qos_profile_sensor_data` | `rclcpp::SensorDataQoS()` | |
| `self.create_timer(1, cb)` | `create_wall_timer(1s, [this]() { ... })` | |

### C++ you'll use
| Concept | Where you'll use it |
|---------|---------------------|
| `std::string_view` | A cheap, read-only "window" onto text. Trimming it doesn't copy anything |
| `std::from_chars` and **structured bindings** (`auto [ptr, ec] = ...`) | Converting text to a number without exceptions |
| `std::ifstream`, `std::getline` | Reading a file line by line |
| `std::optional<T>` as "latest value, if any" | Remembering the last message from each motor |
| `rclcpp::Time` arithmetic | `(now - last).seconds()` |
| `<random>` (`std::mt19937`, `std::normal_distribution`) | Adding realistic sensor noise |
| `std::clamp` | Keeping the volume between empty and full |
| Throwing from a constructor, catching in `main()` | Failing cleanly on bad parameters |

---

## Setup
Start `training/<you>/lesson-4` from `training/<you>/main`, then:
```bash
cd ~/code/lance-ws
cp -r src/project/training/lessons/04-hopper-sensor/starter/hopper_sensor_sim src/project/training/sandbox/
colcon build --symlink-install --packages-select hopper_sensor_sim
```
Commit the starter before you change anything. The package is already set up with dependencies and a test target. Look at `CMakeLists.txt` and compare it with
Lesson 1. What's new? (Hint: `include/` and `if(BUILD_TESTING)`.)

---

## Part 1: Read the Python (no coding)
Open [`hopper_fullness.py`](../../../../lance/scripts/hopper_fullness.py) and answer:
1. What topic, message type, and QoS does it publish with? How often?
2. What does a line from the sensor look like? What does the node do with a line it can't understand?
3. What happens if the serial cable is unplugged while the node is running?
4. Python's bare `except:` catches *everything*, including mistakes in the code itself. Why is that risky?
5. Suppose the sensor sends `Fs: nan`. What does the Python node publish? Could that cause problems downstream?
6. Open the [Arduino sketch](https://github.com/Cardinal-Space-Mining/Arduino-Hopper-Fullness-Sensor/blob/main/Version%203/MainV3/MainV3.ino)
   and find the line that sends `Fs: `. What range of values can it send? (Look at what `CalcFinalVal` returns.)

<details>
<summary>Check your answers</summary>

1. `lance/hopper_fullness`, `std_msgs/Float32`, sensor-data QoS (best effort), once per second.
2. `Fs: <number>`. It prints "Failed to read line from serial!" and skips it.
3. `readline()` fails, the bare `except:` catches it, and it keeps printing that message. It never tries to reconnect,
   because `self.serial` isn't reset to `None`.
4. A typo, such as a misspelled variable name, is silently "handled" as if it were bad data, which hides bugs.
5. `float("nan")` succeeds, so it publishes **NaN**. Arduino's `Serial.print` prints `nan` (and `inf` or `ovf`) for
   invalid or huge floats. The current sketch shouldn't produce them, but a wiring fault or a future code change could.
   Any code that does math with NaN gets NaN back, and comparisons like `x > limit` are always false.
6. `Serial.println(FinalFull/100)`: a percentage from 0–100, divided by 100, so **0.00 to 1.00**, printed with 2 decimals.
</details>

---

## Part 2: Port the parser, test first
The parser lives in its own header, [`include/hopper_sensor_sim/fullness_parser.hpp`](starter/hopper_sensor_sim/include/hopper_sensor_sim/fullness_parser.hpp),
so both the node and the tests can use it. Right now it's a stub that always returns `std::nullopt`.

### 2a. Watch the tests fail
```bash
colcon build --symlink-install --packages-select hopper_sensor_sim
colcon test --packages-select hopper_sensor_sim
colcon test-result --verbose
```
`ParsesSimpleReading` fails. But `RejectsLinesWithoutPrefix` **passes**, even though the parser does nothing.
A function that rejects *everything* passes every "rejects bad input" test. That's why you always need tests that
check success *and* tests that check rejection.

> [!TIP]
> To run just this package's tests quickly, run the test executable directly: `./build/hopper_sensor_sim/fullness_parser_test`

### 2b. Write the parser
Implement `parseFullnessLine(std::string_view line)`:

1. **Trim whitespace** from both ends. `std::string_view` has `remove_prefix(n)` / `remove_suffix(n)`, which just
   move the edges of the "window" without copying anything. Check characters with `std::isspace`:
   ```cpp
   while (!line.empty() && std::isspace(static_cast<unsigned char>(line.front())))
   {
       line.remove_prefix(1);
   }
   ```
   Why the `static_cast<unsigned char>`? `char` can be negative, and passing a negative value (other than `EOF`)
   to `std::isspace` is undefined behavior.
2. **Check and remove the prefix** with `line.starts_with("Fs: ")` and `line.remove_prefix(...)`.
3. **Convert to a number** with `std::from_chars`:
   ```cpp
   float value{};
   const char* const end = line.data() + line.size();
   const auto [ptr, ec] = std::from_chars(line.data(), end, value);
   ```
   `ec` is the error code (`std::errc{}` means success), and `ptr` is where parsing stopped. These are **structured
   bindings**: one function returning two values. Reject the line if `ec` isn't success, *or* if `ptr != end`,
   which means there was junk after the number, like `"4.0 kg"`.
4. **Reject `nan` and `inf`** with `std::isfinite`. (`ovf` already fails to parse.)

> [!NOTE]
> **Why `from_chars` and not `std::stof`?** `stof` *throws* on bad input, and garbage from a sensor isn't
> exceptional. It's expected, so a return value is the better fit. `stof` also ignores trailing junk (`"4.0 kg"` → `4.0`)
> unless you check how much it read.

### 2c. Finish the tests
Add the tests listed at the bottom of `test/fullness_parser_test.cpp`. For each one, first work out what the
**Python** does, for example in a Python shell:
`line = "[42] Fs: 1.0".strip(); float(line.replace("Fs: ", ""))`.

✅ **Checkpoint:** All tests pass. Then discuss with your mentor: your parser and the Python don't agree on *everything*.
`"Fs: +3"` and `"Fs: 1_000"` both work in Python but not with `from_chars`, and `nan` is deliberately different.
Which differences matter? (The Arduino never prints `+` or `_`, so a port doesn't have to copy every quirk. It
just has to be *deliberate* about the differences.)

💾 **Commit, then fix the commit.** Commit your parser and tests, but leave out one file on purpose: stage only
`fullness_parser.hpp`, and commit it with the message `Add parser`. Now pretend you just noticed your mistake. Stage
`test/fullness_parser_test.cpp` too, change the message to `Add fullness parser and tests`, and use the arrow next to **Commit** →
**Commit (Amend)**. The Graph shows one commit, not two. (Only amend commits you haven't pushed yet. See
[GIT.md](../../GIT.md#pull-requests).)

---

## Part 3: The node, replaying a log file
A sample sensor log is in [`sample_serial_log.txt`](sample_serial_log.txt). The values are made up, but
the format matches what the Arduino sends: `Fs: <value>` lines with `\r\n` endings, plus some junk.

### 3a. Publisher and source selection
In the constructor:
1. Create the publisher: same topic and QoS as the Python node (Part 1, question 1).
2. Declare a string parameter `source` (default `"sim"`). If it's `"file"`, call `setupFileSource()`. If it's
   `"sim"`, call `setupSimSource()`, which can be empty for now. Otherwise:
   ```cpp
   throw std::invalid_argument("Parameter 'source' must be \"file\" or \"sim\", got \"" + source + "\"");
   ```
   A constructor can't return an error code, so throwing is how it says "I can't be built". `main()` already
   catches it and prints a clean `FATAL` message. Try `--ros-args -p source:=serial` once it's written.

### 3b. Read one line per tick
`setupFileSource()`:
- Declare `replay_file` (string) and `replay_period_s` (double, default `1.0`).
- Open the file into a `std::ifstream` member: `this->replay_file.open(path);`. If `!is_open()`, throw a
  `std::runtime_error`.
- Create a timer: `create_wall_timer(std::chrono::duration<double>(period_s), ...)`.

`replayNextLine()`:
```cpp
std::string line;
if (!std::getline(this->replay_file, line))
{
    // end of file: clear the error flag and rewind to the start
    this->replay_file.clear();
    this->replay_file.seekg(0);
    return;
}
```
Then parse the line. If it's valid, publish it and log `"<line> --> <value>"`, as the Python does. If not,
`RCLCPP_WARN` and skip it.

Run it from the workspace root:
```bash
ros2 run hopper_sensor_sim hopper_sensor_sim_node --ros-args \
    -p source:=file \
    -p replay_file:=src/project/training/lessons/04-hopper-sensor/sample_serial_log.txt
```

**Look closely at your log output.** Does anything look garbled? `getline` removes the `\n` but leaves the `\r`.
When the terminal prints a `\r`, it moves the cursor back to the start of the line, so whatever comes after it
overwrites the beginning. Remove a trailing `'\r'` from `line` with `pop_back()` before logging it.

✅ **Checkpoint:** Valid lines are published (check with `ros2 topic echo /lance/hopper_fullness`), every junk line
gets a warning (including `Fs: nan`), and the replay loops at the end of the file. Try `-p replay_period_s:=0.1`.

---

## Part 4: Simulate the hopper
Now the interesting part: instead of replaying recorded numbers, *compute* them from what the robot is doing.

### The model
| What the robot is doing | Condition | Effect |
|-------------------------|-----------|--------|
| Digging | hopper actuator position `< dig_below_pos` (hopper lowered, trencher in the ground) | volume grows by `fill_l_per_trencher_rev` × trencher speed (rev/s) |
| Dumping | hopper actuator position `> dump_above_pos` (hopper raised) | volume shrinks by `drain_l_per_belt_rev` × belt speed (rev/s) |
| Anything else | | no change |

The model tracks the volume in liters, which always stays between `0` and `capacity_l`. What it **publishes** is the
fullness fraction, `volume_l / capacity_l` (0–1), plus noise, the same as the real sensor. Make all of these
**parameters** with these defaults:

| Parameter | Default | |
|-----------|---------|---|
| `capacity_l` | `50.0` | Matches `capacity_volume_liters` in `robot_control.json` |
| `fill_l_per_trencher_rev` | `0.005` | About 0.8 L/s at full trencher speed in the sim |
| `drain_l_per_belt_rev` | `0.03` | Empties a full hopper in about 10 s |
| `dig_below_pos` | `0.35` | Actuator position is 0 (down) to 1 (up) |
| `dump_above_pos` | `0.8` | |
| `noise_stddev` | `0.01` | Noise on the published fraction (1% of full) |

### 4a. Remember the latest telemetry
Subscribe to `lance/trencher/info`, `lance/hopper_belt/info`, and `lance/hopper_act/info`, the same as Lesson 1. This
time, though, the callbacks do no work. They just store the message:
```cpp
std::optional<TalonInfoMsg> trencher;   // member
...
[this](const TalonInfoMsg& msg) { this->trencher = msg; }
```
The simulation runs on its own **timer** (every 100 ms), using whatever the latest values are. Why separate them? The
three topics arrive at their own times, and the model needs all three together.

### 4b. Step the simulation
In `stepSimulation()`:
1. If any of the three `optional`s is still empty, return (log a throttled "waiting" message).
2. **Measure `dt`** using the node's clock:
   ```cpp
   const rclcpp::Time now = this->now();
   // keep a std::optional<rclcpp::Time> last_step_time member.
   // First call: store it and return. Afterwards:
   const double dt = (now - *this->last_step_time).seconds();
   ```
   Skip the step if `dt <= 0`. That happens if sim time is paused or jumps backwards. Compare this with
   `prev_trencher_rotations` and friends in [`collection_state.cpp`](../../../../lance/src/robot/core/collection_state.cpp):
   it's the same `std::optional` "first call" pattern.
3. Work out `fill_rate` and `drain_rate` (liters per second) from the table, using `std::abs` on the velocities.
4. **Integrate:** `volume_l += (fill_rate - drain_rate) * dt;`, then `std::clamp` it.
5. **Add noise and publish** the fullness fraction:
   ```cpp
   std::mt19937 rng{std::random_device{}()};                            // member: random number generator
   std::normal_distribution<double> noise{0.0, this->noise_stddev};    // in stepSimulation()
   const double fullness = this->volume_l / this->capacity_l;
   publish(static_cast<float>(fullness + noise(this->rng)));
   ```
   Add the noise to the **published** value only, never to `volume_l` itself. What would happen if you did?

### 4c. Run it in the simulator
```bash
# terminal 1
./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled

# terminal 2
ros2 run hopper_sensor_sim hopper_sensor_sim_node --ros-args -p use_sim_time:=true
```
Click **Enable** in Foxglove, then:
1. **Dig:** lower the hopper (right stick down) and hold the right trigger to spin the trencher.
2. **Dump:** raise the hopper all the way (right stick up) and hold the left trigger to run the belt.

> [!TIP]
> **No Gazebo?** Use the motor-sim-only setup from [Lesson 1](../01-motor-monitor/README.md#2-pick-something-to-listen-to)
> and move things with `ros2 topic pub` (percent output, `mode: 0`): `hopper_act` at `-1.0` for a few seconds to lower
> it, then `trencher` at `1.0`. **Don't** use `use_sim_time:=true` in this setup. There's no simulator publishing
> `/clock`, so your node's time would never move and `dt` would always be 0.

✅ **Checkpoint:** `ros2 topic echo /lance/hopper_fullness` climbs while you dig and drops to about 0 when you dump. (With the
trencher at 100% output in the motor sim, it reaches about 0.2, or 10 L of 50, in 12 s. In teleop, the rate depends on how fast the trencher spins.)

---

## Part 5: See it in Foxglove 📈
The project's Foxglove layout already plots `/lance/hopper_fullness.data` (the "Hopper Height" plot), next to
`/lance/collection_state/volume.data`, the **robot's own estimate** of hopper volume from its collection model.

The two use different units: fullness is a fraction from 0 to 1, and the robot's estimate is in **liters** (0–50). On the
same axes, your line hugs the bottom. Add a **new Plot panel** with just `/lance/hopper_fullness.data`, and compare its
*shape* with the volume line. (Stretch goal 1 publishes liters too, for a direct comparison.)

Mine for a while and compare. They won't match: they're different models. Talk with your mentor
about *why*. (Hint: the robot's model also accounts for driving distance and mining depth. Look at
`CollectionState::update`.) This is exactly why the real robot has a sensor: estimates drift.

✅ **Done!** Screenshot the plot while mining and dumping, and share it.

---

## Part 6: Pull request and review
Push, and open a PR from `training/<you>/lesson-4` into `training/<you>/main`, as in Lesson 3. Put your plot screenshot
and your answers to the questions below in the description.

This time, your mentor will probably ask for changes. **Responding to review** is a normal part of every change on
the team:
1. Read every comment. If you're not sure what one means, ask by replying to it.
2. Fix the code, commit, and push. **Don't amend**: these commits are already on GitHub. The PR updates by itself.
3. Reply to each comment to say what you changed (or why you didn't), and click **Resolve conversation**.
4. Click **Re-request review** (the 🔄 icon next to your mentor's name) to tell them it's ready again.

> [!TIP]
> With the GitHub Pull Requests extension, review comments also appear in VS Code, right next to the line they're about.
> Click the GitHub icon in the sidebar to see your PRs.

✅ **Checkpoint:** Your PR is approved and merged. Sync `training/<you>/main` before starting Lesson 5.

---

## Check your understanding
1. Why does the parser take a `std::string_view` instead of a `std::string`? What would go wrong if it *returned* a
   `string_view` into a `std::string` that was about to be destroyed?
2. When is it better to report an error with a return value (`std::optional`, error codes), and when with an exception?
   Find one of each in your node.
3. Your published values can be slightly **negative** when the hopper is empty, because of the noise. Should you clamp
   them? What does a real sensor do? What would code that *uses* this topic need to handle?
4. Why compute `dt` from the clock instead of assuming it's exactly 0.1 s, since that's the timer period?
5. What's the difference between the trencher's `velocity` and `position` fields? Which one would you use to
   integrate *more accurately*, and why? (Look at how `collection_state.cpp` does it.)

## Stretch goals
1. **Debug topic.** Also publish the *true* (noise-free) volume **in liters** on `hopper_sensor_sim/true_volume_l`. Plot it
   on the same axes as `/lance/collection_state/volume.data` for a direct comparison with the robot's estimate.
2. **Realistic glitches.** Real serial links occasionally deliver corrupted lines. In sim mode, randomly (say 2% of
   the time) skip publishing and log a fake "corrupted line" instead. Use `std::bernoulli_distribution`.
3. **More accurate integration.** Integrate using the change in motor `position` (rotations) between steps instead
   of `velocity × dt`, as `collection_state.cpp` does. Why is this better when steps are uneven?
4. **Time to full.** Log an estimate of how many seconds until the hopper is full at the current fill rate.
5. **Quantize like the real sensor.** The real sensor can't report every value. Each light bar has 16 sensors, and
   `CalcFinalVal` maps the highest covered one to a fixed percentage. Read the sketch and make your simulated output
   jump between levels the same way. (A simple version: round to the nearest 0.04.)
6. **Better reconnect.** In the Python node, if the cable is unplugged, it never recovers (Part 1, question 3). Write
   down how you'd fix that, in either language.

## Reference solution
[`solution/hopper_sensor_sim/`](solution/hopper_sensor_sim/) covers Parts 2–4.

> [!NOTE]
> **For mentors:** The units come from the Arduino sketch (`Version 3/MainV3/MainV3.ino`):
> `Serial.print("Fs: "); Serial.println(FinalFull/100);`, where `FinalFull` is a percentage from 0–100.

**Next:** [Lesson 5: Launch your node with the robot](../05-launch-integration/README.md)
