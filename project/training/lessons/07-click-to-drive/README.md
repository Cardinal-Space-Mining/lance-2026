# Lesson 7 (Capstone): Click to Drive

**Time:** ~4–6 hours &nbsp;·&nbsp; **Prerequisites:** Lessons [1](../01-motor-monitor/README.md), [3](../03-breadcrumbs/README.md),
and [5](../05-launch-integration/README.md) ([2](../02-unit-tests/README.md) helps for the stretch goals)

## What you'll build
A node called `goto_point` that **drives the robot**. Click a spot on the ground in Foxglove's 3D view, and the
robot turns to face it, drives there, and stops:

```
New goal: (-1.00, 0.00) in 'odom'
IDLE -> TURNING
TURNING -> DRIVING
Arrived!
DRIVING -> IDLE
```

This brings together nearly everything from the earlier lessons:

| You need... | From... |
|-------------|---------|
| A node with parameters, timers, publishers, and subscriptions | Lesson 1 |
| The `TalonCtrl` message and motor topics | Lessons 0 and 1 |
| TF, coordinate frames, `try` / `catch`, `std::optional` | Lesson 3 |
| Markers in Foxglove | Lesson 3 |
| Sim time, `dt`, "latest value" members | Lesson 4 |
| Launch arguments like `robot_control:=null` | Lesson 5 |
| `switch` and state constants | Lesson 6 |

## Why it matters
Driving to a point is the core of autonomy. The real robot does a far more sophisticated version in
[`traversal_controller.cpp`](../../../../lance/src/robot/control/shared/traversal_controller.cpp): it follows planned
paths from perception, limits acceleration, and uses a Stanley steering controller. Mission control even has its own
"click a target" feature. After this lesson, you'll be able to read that code and recognize the pieces.

## ⚠️ Safety first
You are writing code that **moves the robot**. Even in simulation, build good habits now:

1. **Only one node commands the motors.** The robot's own controller (`robot_control`) normally sends track commands.
   For this lesson you turn it off with `robot_control:=null`, so the two don't fight.
2. **Motor controllers keep doing the last thing they were told.** If your node crashes while driving, the robot
   **keeps driving**. That's true of the simulator *and* the real Talon motor controllers. Your node must send a
   command every cycle, including "stop".
3. **The Disable button is your emergency stop.** Mission control's watchdog turns the motors off when you click
   **Disable** in Foxglove. Keep it on screen whenever your node is running.
4. **This lesson is simulator-only.** Never run it on the real robot without a mentor, and run
   `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` if you're at the lab.

## Background
### Differential drive
LANCE steers like a tank: it turns by running its tracks at different speeds. If you want the robot to move
forward at `v` m/s while turning at `ω` (omega) rad/s, with positive meaning counter-clockwise, to the left:
```
left  = v − ω × (track_separation / 2)
right = v + ω × (track_separation / 2)
```
`ω = 0` drives straight, `v = 0` spins in place, and anything else follows an arc. These are the same equations as
`bodyDynamicsToLeftTrackVelocityMps` / `...Right...` in
[`dynamics.hpp`](../../../../lance/src/robot/model/dynamics.hpp).

### Where is the goal, *from the robot's point of view*?
If you transform the goal into the robot's own frame, `base_link` (+x forward, +y left), the math becomes easy:
```
distance      = hypot(x, y)
heading_error = atan2(y, x)    // 0 = straight ahead, + = to the left, ±π = directly behind
```
`atan2` uses the signs of *both* arguments to give the full −π..π range. Plain `atan(y / x)` can't tell
"ahead-left" from "behind-right", and it divides by zero when `x = 0`.

### A state machine
Your node is always in exactly one **state**, and switches between them when conditions are met:
```
                 new goal                 facing the goal
   ┌──────┐  ─────────────▶  ┌─────────┐  ──────────────▶  ┌─────────┐
   │ IDLE │                  │ TURNING │                   │ DRIVING │
   └──────┘  ◀─────────────  └─────────┘  ◀──────────────  └─────────┘
       ▲         arrived                   facing far away      │
       └────────────────────────────────────────────────────────┘
                                 arrived
```

### Proportional control
"Turn faster the further you're pointing from the goal": `ω = turn_kp × heading_error`, capped at a maximum. The
same idea applies to speed: slow down as you get close.

### C++ you'll use
| Concept | Where |
|---------|-------|
| `enum class` + `switch` | The state machine. With `-Wall`, the compiler warns if a `switch` misses one of the enum's values |
| Nested types | `State` and `TrackCommand` defined *inside* the class |
| `static` member functions | `toString(State)` doesn't need `this` |
| `std::numbers::pi` (C++20) | Degrees ↔ radians |
| `std::atan2`, `std::hypot`, `std::clamp` | The geometry |
| `std::optional::reset()` | Clearing the goal when you arrive |

---

## Setup
Start `training/<you>/lesson-7` from `training/<you>/main`, then:
```bash
cd ~/code/lance-ws
cp -r src/project/training/lessons/07-click-to-drive/starter/goto_point src/project/training/sandbox/
colcon build --symlink-install --packages-select goto_point
```
Start the simulator **without the robot controller**:
```bash
./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled robot_control:=null
```
Use the preview tool from Lesson 5 to confirm `robot_control` isn't in the list of enabled actions.

---

## Part 1: Receive goals
### 1a. TF and parameters
Set up the TF buffer and listener as in Lesson 3. (Remember the declaration order!) Declare parameters for everything
you might want to tune: `fixed_frame` (`odom`), `robot_frame` (`base_link`), `max_speed_mps` (0.25),
`max_turn_radps` (0.6), `turn_kp` (1.5), `drive_kp` (0.8), `heading_tolerance_deg` (10), and `arrive_distance_m` (0.15).
Convert degrees to radians once, in the constructor: `deg * std::numbers::pi / 180.0`.

### 1b. Store the goal in a *fixed* frame
Subscribe to `/clicked_point` (`geometry_msgs/msg/PointStamped`). Every 3D panel in the team's Foxglove layout already
publishes there when you use its **publish point** tool and click on the scene. The point arrives in that panel's
display frame, usually `odom`.

In `onGoal`, transform the point into `fixed_frame` and store it in a `std::optional<PointStampedMsg> goal_fixed`:
```cpp
PointStampedMsg goal = msg;
goal.header.stamp = rclcpp::Time(0);   // 0 means "use the latest transform"
this->goal_fixed = this->tf_buffer->transform(goal, this->fixed_frame);
```
`tf_buffer->transform()` works on `geometry_msgs` types because of the `tf2_geometry_msgs` include. It throws on
failure, just like `lookupTransform`, so wrap it in `try` / `catch` and ignore the goal if it fails.

Why not store the goal in `base_link`? Because `base_link` **moves with the robot**. A goal "1 m in front of me" would
stay 1 m in front forever, and the robot would chase it across the arena.

Why set the stamp to 0? Foxglove stamps the point with *its* idea of the time. Asking TF for a transform at exactly that
moment can fail with an "extrapolation" error, if it's slightly newer than the latest transform TF has.

✅ **Checkpoint:** Clicking in Foxglove logs `New goal: (x, y) in 'odom'`. No Foxglove? Publish one by hand:
```bash
ros2 topic pub --once -w 2 /clicked_point geometry_msgs/msg/PointStamped \
    "{header: {frame_id: odom}, point: {x: -1.0, y: 0.0, z: 0.0}}"
```
(`-w 2` waits until two subscribers are connected, yours and mission control's. Without it, the message can be sent
before your node is listening, and be lost.)

---

## Part 2: The state machine
### 2a. Each cycle, find the goal relative to the robot
Create a 50 ms timer calling `update()`. If there's a goal, transform a copy of `goal_fixed` into `robot_frame` (stamp
0 again), then compute `distance` and `heading_error`. If the transform fails, **stop**. Not knowing where you are is
a reason to stop, not to guess.

### 2b. Run the state machine
Write `TrackCommand runStateMachine(double distance, double heading_error)` with a `switch (this->state)`:

| State | If... | Then... |
|-------|-------|---------|
| `TURNING` | `distance < arrive_distance_m` | arrive: clear the goal, go to `IDLE` |
| | `abs(heading_error) < heading_tolerance` | go to `DRIVING` |
| | otherwise | turn in place: `ω = turn_kp × heading_error` |
| `DRIVING` | `distance < arrive_distance_m` | arrive |
| | `abs(heading_error) > 3 × heading_tolerance` | go back to `TURNING` |
| | otherwise | `v = min(max_speed, drive_kp × distance)`, `ω = turn_kp × heading_error` |
| `IDLE` | | stay stopped |

Clamp `ω` to `±max_turn_radps`. Write a `setState` helper that logs every change (`TURNING -> DRIVING`).

**Why 3× the tolerance to go back to `TURNING`?** If both transitions used the same threshold, a heading error hovering
right at the edge would flip the state back and forth every cycle. Using two different thresholds is called
**hysteresis**. Thermostats do the same thing.

✅ **Checkpoint:** With no motor output yet, click points around the robot and watch the log. Points behind it give a heading
error near ±π, and points to the left give a positive error. Print the numbers to check.

---

## Part 3: Drive the motors
### 3a. Differential drive and limits
Turn `v` and `ω` into left and right track speeds using the formulas above (`track_separation_m` = 0.632 m for LANCE-2).
If either track's speed would be over `max_speed_mps`, scale **both** down by the same factor, so the robot still follows
the same curve. That's the same idea as `applyTrackLimits`, which you tested in Lesson 2.

### 3b. From m/s to motor commands
The tracks are commanded in **motor rotations per second**. Work out the conversion from
[`dynamics.hpp`](../../../../lance/src/robot/model/dynamics.hpp) (the `#elif LANCE >= 2` block):
```
motor rev per meter = TRACK_GEARING / (2π × TRACK_EFFECTIVE_OUTPUT_RADIUS_M)
```
You should get about **226.35**. Make it a parameter, `motor_rps_per_mps`.

### 3c. Publish, every cycle
Publish a `TalonCtrl` with `mode = TalonCtrlMsg::VELOCITY` and `value = speed_mps × motor_rps_per_mps` on
`lance/track_left/ctrl` and `lance/track_right/ctrl`. Use the same QoS as the robot's controller:
```cpp
rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()
```
**Publish every cycle, even when the command is zero.** (Safety rule 2.)

✅ **Checkpoint:** Run your node with `--ros-args -p use_sim_time:=true`, click **Enable** in Foxglove, and click a point a
meter or so away. The robot turns, drives, and stops on it. In testing, two goals in a row ended within 0.15 m of each
target.

> [!TIP]
> If the robot drives into a wall and stops making progress, click a point in a different direction. This simple
> controller has no idea obstacles exist. The real robot's path planner does.

---

## Part 4: Show the goal
Publish a `MarkerArray` with a green `SPHERE` at the goal, in `fixed_frame`, sized to the arrival distance, as in
Lesson 3. When there's no goal, publish the same `ns` + `id` with `action = MarkerMsg::DELETE` to remove it.

✅ **Done!** Record a short screen capture of the robot driving to two or three clicked points, and share it.

---

## Part 5: Break it on purpose (safely)
1. Click a far-away goal, and while the robot is driving, press `Ctrl` + `C` on your node. What does the robot do?
2. Click **Disable** in Foxglove. What happens now?
3. Talk with your mentor: how does the real robot protect against a crashed controller? (Hint: look at what
   `feedWatchdogStatus` in [`phx6_driver.cpp`](../../../../phoenix-driver/src/lance2/phx6_driver.cpp) does with the
   watchdog value, and at which node publishes it.) What *doesn't* that protect against?

---

## Check your understanding
1. Why is the goal stored in `odom` but compared in `base_link`?
2. Why must the node publish "stop" every cycle, instead of publishing once when it arrives?
3. What would go wrong if `TURNING → DRIVING` and `DRIVING → TURNING` used the same threshold?
4. You click a point in a 3D panel whose display frame is `map`, before the robot has localized. What happens, and why?
   (Lesson 3, Step 7.)
5. Open `traversal_controller.cpp` and find where it turns a forward speed and turn rate into track speeds, and where it
   limits them. What does it do that yours doesn't?

## Stretch goals
1. **Smooth acceleration.** Real tracks can't change speed instantly. Limit how much each track's speed may change
   per cycle (`max_accel × dt`). For stopping smoothly, `util::kmx::maxStartVel` from Lesson 2 tells you how fast you
   can be going and still stop in the remaining distance.
2. **Waypoints.** Queue up clicks in a `std::deque` (Lesson 3), and drive to them in order. Show them all as markers,
   joined by a line.
3. **Final heading.** Foxglove can also publish a *pose* (position plus direction) on `/move_base_simple/goal`. Add an
   `ALIGNING` state that turns to face the requested direction after arriving. You'll need to **wrap** the angle
   difference into −π..π yourself. Why doesn't `atan2` save you this time?
4. **Unit test the controller.** Move `runStateMachine` and the differential drive math into a header that doesn't
   depend on ROS, then test it with GoogleTest (Lesson 2). For example: "a goal straight behind gives zero forward speed",
   "a goal to the left turns left".
5. **Launch preset.** Add a `goto_point` action (Lesson 5) and a `gz_full` preset, like `lance2_ksc_click_drive`, that
   enables it **and** sets `robot_control` to `null` through `pragma:action_overrides`. Then one argument starts the
   whole setup safely.
6. **Report progress.** Publish your state and distance remaining as a custom message (Lesson 6).

## Pull request and cleanup
1. Open your final PR, from `training/<you>/lesson-7` into `training/<you>/main`. Include your screen capture and your
   answers to the questions above.
2. Once it's merged, tidy up: delete your merged `lesson-N` branches, on GitHub and locally (see
   [Finishing up](../../GIT.md#finishing-up)). Keep `training/<you>/main`.
3. Look at your history one more time: `git log --oneline --graph` in a terminal, or the **Graph** in Source Control.
   Every lesson you did, in order, with a review behind each one.

## Reference solution
[`solution/goto_point/`](solution/goto_point/) covers Parts 1–4.

---

🎓 **That's the end of the core lessons!** Next, ask a mentor about an open issue in the robot code, or try one of the
stretch goals above. Real work uses the same git routine you've been practicing, with one difference: your branch starts
from `main` (for example `jsmith/fix-hopper-limit`), and the pull request goes into `main`, reviewed by the team. Several of them, such as smooth acceleration, unit tests, and launch presets, are exactly the kind of
work the team does every day.
