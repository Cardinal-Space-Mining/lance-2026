# Lesson 3: Breadcrumb Trail Visualizer

**Time:** ~3–4 hours &nbsp;·&nbsp; **Prerequisite:** [Lesson 1](../01-motor-monitor/README.md) (Lesson 2 is helpful but not required)

## What you'll build
A node called `breadcrumbs` that:
- asks **TF** where the robot is, 10 times per second
- drops a "breadcrumb" every 5 cm the robot moves, and remembers the path
- keeps a running total of distance driven
- publishes **markers**: an orange line along the path, plus a floating label above the robot reading
  something like `1.90 m driven`

When you're done, you'll drive the robot around in the simulator and watch its trail draw itself in Foxglove's 3D
view.

## Why it matters
Every part of LANCE that needs to know *where something is* uses **TF**: perception, path planning, autonomous
mining, and the mission control display. Markers are how the team sees what the robot is "thinking" during
testing. The zone boxes, planned paths, and mining targets you see in Foxglove are all markers. See
[`zone_pub.hpp`](../../../../lance/src/mission_control/zone_pub.hpp) for the arena zones, and
[`tf_cache.hpp`](../../../../lance/src/robot/sensing/tf_cache.hpp) for how the robot controller does TF lookups.

## Background
Read these first (about 45 minutes):
- [Introducing tf2](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/Working-with-interfaces/Introduction-To-Tf2/Introduction-To-Tf2.html)
- [Writing a listener (C++)](https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Writing-A-Tf2-Listener-Cpp.html). Focus on the code, and skip the turtlesim setup.
- [Using markers (C++)](https://docs.ros.org/en/lyrical/Developer-Tools/Visualization/RViz/Marker-Display-types/Marker-Display-types.html). Foxglove understands the same markers as RViz.

### TF in one paragraph
A robot has many **coordinate frames**: the arena, the robot's body, the LiDAR, the hopper. Each one is a
point plus a direction for "forward". TF keeps track of how every frame relates to every other one, as a **tree**
of transforms, and different nodes publish different branches of that tree. You ask TF "where is frame B, as
seen from frame A?" and it chains the transforms together for you.

| Frame | What it is |
|-------|------------|
| `base_link` | The robot's body. Moves with the robot |
| `odom` | Where the robot **started**, according to its own odometry (LiDAR odometry from `cardinal_perception`). Smooth, but drifts slowly over time |
| `map` | The **arena**. Only connected to `odom` after the robot has localized itself against the arena's fiducial |
| `lidar_link`, `hopper_link` | Parts of the robot, attached to `base_link` |
| `gz_map`, `gz_base_link` | **Simulator only:** Gazebo's perfect "ground truth" pose |

### C++ you'll use
| Concept | Where you'll use it |
|---------|---------------------|
| Exceptions: `try` / `catch` | TF *throws* when it can't answer, and you need to *catch* it |
| `std::optional<T>` | A function that might not have an answer: "here's a point" or "nothing" |
| `std::unique_ptr` vs. `std::shared_ptr` | The TF buffer has one owner, and the listener is shared |
| **Member initialization order** | A real bug source: members are built in *declaration* order |
| `std::deque` | A list you can add to at the back and remove from at the front efficiently |
| References to vector elements | `MarkerMsg& m = msg.markers.emplace_back();` |
| Anonymous namespace helper functions | A small `distance2d()` function used only in this file |

---

## Setup
This lesson needs the **full simulation**. The motor simulator by itself doesn't publish TF.

Start `training/<you>/lesson-3` from `training/<you>/main`, then copy and build the starter:
```bash
cd ~/code/lance-ws
cp -r src/project/training/lessons/03-breadcrumbs/starter/breadcrumbs src/project/training/sandbox/
colcon build --symlink-install --packages-select breadcrumbs
```
Commit the starter (`Add Lesson 3 starter package`) before you change anything, and commit at each checkpoint.

In a separate terminal, start the simulation and click **Enable** in Foxglove:
```bash
./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled
```

> [!TIP]
> If Gazebo is too slow on your laptop, ask a mentor for a bag recording that includes `/tf`, `/tf_static`, and
> `/clock`, and play it with `ros2 bag play <bag> --clock`. Everything in this lesson works the same way.

---

## Step 1: Explore the TF tree
With the simulation running, in a new terminal (remember to `source install/setup.bash`):

```bash
ros2 run tf2_ros tf2_echo odom base_link
```
This prints where the robot is relative to where it started. Drive around and watch the numbers change.

Now look at the whole tree:
```bash
ros2 run tf2_tools view_frames
```
This creates a `frames_<date>.pdf` in the current folder. Open it.

✅ **Checkpoint:** Answer these:
1. How many separate trees are there? (Trees aren't connected to each other.)
2. Try `ros2 run tf2_ros tf2_echo map base_link`. What happens, and why? (Look back at the frame table above.)
3. Look at the `At time` lines from `tf2_echo`. Is that a normal clock time? (It's **simulated time**, the
   number of seconds since Gazebo started. You'll need this in Step 2.)

---

## Step 2: Look up the robot's position

### 2a. Member initialization order (read this first!)
You need two TF objects as members of your node:
```cpp
std::unique_ptr<tf2_ros::Buffer> tf_buffer;                  // stores transforms
std::shared_ptr<tf2_ros::TransformListener> tf_listener;     // fills the buffer from /tf
```
The listener's constructor takes a **reference to the buffer**, so the buffer must exist first.

In C++, class members are initialized in the order they are **declared in the class**, *not* the order you
write them in the constructor's initializer list. If you declare `tf_listener` above `tf_buffer`, the listener is
built with a reference to a buffer that doesn't exist yet. That's undefined behavior, which usually means a crash
that's hard to track down.

The `-Wall` flag in your `CMakeLists.txt` makes the compiler warn about this mistake: `will be initialized after
[-Wreorder]`, or `is used uninitialized`. Warnings don't stop the build, though, so **read your warnings**.

So: **declare the buffer before the listener**, and create both in the constructor's initializer list:
```cpp
BreadcrumbsNode() :
    Node("breadcrumbs"),
    tf_buffer{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
    tf_listener{std::make_shared<tf2_ros::TransformListener>(*tf_buffer)}
{
    ...
}
```
`*tf_buffer` *dereferences* the `unique_ptr` to get the buffer itself, which is what the listener's constructor wants.

### 2b. Parameters and a timer
Declare string parameters `fixed_frame` (default `"odom"`) and `robot_frame` (default `"base_link"`). Store them in
members. Then create a 100 ms timer that calls `update()`, just like in Lesson 1.

### 2c. The lookup
In `update()`:
```cpp
const auto tf = this->tf_buffer->lookupTransform(
    this->fixed_frame,      // "where is the robot, as seen from here..."
    this->robot_frame,      // "...this frame"
    tf2::TimePointZero);    // "...using the newest transform you have"
```
`tf.transform.translation.x` / `.y` / `.z` is the robot's position. Print `x` and `y` with `RCLCPP_INFO`.

Build and run it **with simulated time**:
```bash
ros2 run breadcrumbs breadcrumbs_node --ros-args -p use_sim_time:=true
```

**What happens?** Your node probably **crashes** within a second:
```
terminate called after throwing an instance of 'tf2::LookupException'
  what():  "odom" passed to lookupTransform argument target_frame does not exist.
[ros2run]: Aborted
```

When your node starts, its TF buffer is empty because no `/tf` messages have arrived yet. `lookupTransform`
**throws an exception** when it can't answer. Nothing catches it, so it escapes your callback, and the program ends.

### 2d. Catch the exception
Wrap the lookup in `try` / `catch`:
```cpp
try
{
    const auto tf = this->tf_buffer->lookupTransform(...);
    // use tf here
}
catch (const tf2::TransformException& e)
{
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Can't find robot position yet: %s", e.what());
    return;
}
```
`tf2::TransformException` is the *base class* of all TF errors, so catching it catches every kind of TF error.
`e.what()` gives the error message.

✅ **Checkpoint:** You see one warning when the node starts, then the robot's position, which changes as you drive.

### 2e. Why `use_sim_time`?
Add `this->now().seconds()` to your print. Run once **with** `-p use_sim_time:=true` and once **without**. What's
the difference?

<details>
<summary>Explanation</summary>

Without it, `now()` is the real clock: about 1.8 billion seconds since 1970. With it, `now()` follows the simulator's
`/clock` topic, which counts from when Gazebo started. Everything in the sim (TF, sensor data) is stamped in sim
time. If your node stamps messages in real time, other tools may decide your data is from the far future and
refuse to use it. **Rule: in simulation, always run with `use_sim_time:=true`.** The sim presets set it for the
robot's own nodes. Search [`robot_control.json`](../../../../lance/config/actions/robot_control.json) for `use_sim_time`.
</details>

---

## Step 3: Clean it up with `std::optional`
`update()` is getting long. Move the lookup into its own function:
```cpp
std::optional<PointMsg> lookupRobotPosition();
```
- On success, build a `geometry_msgs::msg::Point` from the translation and `return` it.
- In the `catch`, log the warning and `return std::nullopt;`.

Then `update()` becomes:
```cpp
const std::optional<PointMsg> position = this->lookupRobotPosition();
if (!position)
{
    return;   // no answer yet: try again next time
}
// use *position (or position->x) from here on
```

An `optional` makes "this might not have a value" part of the function's **type**, so the code that calls it
can't forget to check. Compare that with returning a special value like `(0, 0, 0)`, which is also a perfectly valid
position.

✅ **Checkpoint:** Same behavior as before, but cleaner code.

---

## Step 4: Drop breadcrumbs
Add members:
```cpp
std::deque<PointMsg> trail;
double distance_driven_m{0.0};
```
And a parameter `min_spacing_m` (default `0.05`).

In `update()`:
1. If the trail is empty, `push_back` the position.
2. Otherwise, work out how far the robot is from the **last** breadcrumb (`trail.back()`). If it's at least
   `min_spacing_m`, push the new position and add the distance to `distance_driven_m`.
3. Write the distance calculation as a small helper function **outside the class**, inside an anonymous
   namespace (`namespace { ... }`), like the test helpers in Lesson 2:
   ```cpp
   double distance2d(const PointMsg& a, const PointMsg& b);   // use std::hypot
   ```
   Why ignore `z`?

> [!TIP]
> **Why the minimum spacing?** Without it, a parked robot adds 10 identical points every second, forever.
> **Why a `deque`?** Stretch goal 1 caps the trail length. Removing the *oldest* point is
> `pop_front()`, which is fast for a `deque` but slow for a `std::vector` (every other element would shift).

✅ **Checkpoint:** Print `trail.size()` and `distance_driven_m`. Drive about 1 m forward: `distance_driven_m` is close to 1.0,
with somewhere between 10 and 20 points. (Why does the number of points depend on how fast you drive? Think about the 100 ms timer.)

---

## Step 5: Publish markers
Create a publisher of `visualization_msgs::msg::MarkerArray` on `breadcrumbs/markers`. At the end of `update()`,
build one array with two markers and publish it.

**The trail line.** Add a marker to the array and fill it in through a reference:
```cpp
MarkerArrayMsg msg;

MarkerMsg& line = msg.markers.emplace_back();   // add a marker, get a reference to it
line.header.frame_id = this->fixed_frame;       // the coordinates are in this frame
line.header.stamp = this->now();
line.ns = "trail";                              // ns + id identify a marker; reusing them
line.id = 0;                                    //   *replaces* the old one instead of adding more
line.type = MarkerMsg::LINE_STRIP;              // connect the points in order
line.action = MarkerMsg::ADD;
line.pose.orientation.w = 1.0;                  // "no rotation" (an all-zero quaternion is invalid!)
line.scale.x = 0.04;                            // line width (m)
line.color.r = 1.0f;  line.color.g = 0.6f;  line.color.a = 1.0f;   // orange. a = 0 is invisible!
line.points.assign(this->trail.begin(), this->trail.end());
```
`emplace_back()` returns a **reference** to the new element, so `line.x = ...` edits the marker *inside* the array.
What would happen if you wrote `MarkerMsg line = ...` without the `&`?

**The label.** Add a second marker with `type = MarkerMsg::TEXT_VIEW_FACING`, `ns = "label"`, positioned 0.8 m above
the robot's current position. `scale.z` is the text height (try `0.2`). Fill `text` using `std::snprintf`, as in
Lesson 1: `"%.2f m driven"`.

✅ **Checkpoint:** `ros2 topic echo /breadcrumbs/markers --once` shows both markers, with `frame_id: odom` and a growing
`points:` list.

---

## Step 6: See it in Foxglove 🗺️
1. In Foxglove, open the **3D** panel's settings. Under **Frame**, set the display frame to `odom`.
2. Under **Topics**, make sure `/breadcrumbs/markers` is visible.
3. Drive around and watch the trail draw itself. Try a figure eight!

✅ **Done!** Take a screenshot of your trail and share it with your mentor.

---

## Step 7: Break it on purpose
Run your node with the arena frame:
```bash
ros2 run breadcrumbs breadcrumbs_node --ros-args -p use_sim_time:=true -p fixed_frame:=map
```
You should get a warning every 2 seconds:
```
Could not find a connection between 'map' and 'base_link' because they are not part of the same tree.
```
Your node *keeps running* instead of crashing. That's your `try` / `catch` doing its job. Until the robot localizes
against the arena fiducial, nothing publishes `map → odom`, so TF really can't answer the question. Code that runs
on a robot must expect information to be missing **sometimes**, and keep working when it is.

---

## Step 8: Open your first pull request
Until now, you merged your lessons yourself. On the team, every change is **reviewed** before it's merged, through a
**pull request** (PR). From now on, your lessons work the same way.

1. Push your branch (**Publish Branch**, or **Sync Changes** if it's already published).
2. Open a PR, following [Pull requests](../../GIT.md#pull-requests) in GIT.md:
   - **base:** `training/<you>/main`. ⚠️ Not `main`! Double-check it before you click Create.
   - **merge:** `training/<you>/lesson-3`
   - **Title:** `Lesson 3: Breadcrumb trail visualizer`
   - **Description:** what your node does, your Foxglove screenshot (paste it straight into the box), and your answers to
     the **Check your understanding** questions below.
3. Add your mentor as a **reviewer**.
4. Look at the PR's **Files changed** tab. This is what your reviewer sees. Is there anything in it you didn't mean to
   include?

Wait for review before merging. Start Lesson 4 while you wait. (Lesson 4 starts from `training/<you>/main`, which
doesn't have Lesson 3 yet. That's fine, because Lesson 4 doesn't need it.)

✅ **Checkpoint:** Your mentor approved your PR, and you merged it on GitHub. Switch to `training/<you>/main` and click
**Sync Changes**: your breadcrumbs package is now there too.

---

## Check your understanding
1. What would happen if you declared `tf_listener` *above* `tf_buffer` in the class? Would it compile? Would it work?
2. Why is `tf_buffer` a `unique_ptr`, while publishers and timers are `shared_ptr`s?
3. What's the difference between the `odom` and `map` frames? Why does the robot need both?
4. A teammate says: "Just return `{0, 0, 0}` from `lookupRobotPosition` when TF fails." What goes wrong?
5. Why do all markers in your array get the same `ns` + `id` every time you publish, instead of a new `id` each time?

## Stretch goals
1. **Bounded memory.** Add a `max_points` parameter, and `pop_front()` old breadcrumbs once the trail gets too long.
2. **Odometry drift (sim only).** Also look up `gz_map → gz_base_link`, Gazebo's perfect pose, and track a second
   distance-driven total. Show both in the label. How far apart do they get after a few minutes of driving? This is why
   the robot re-localizes against the arena fiducial.
3. **Color by speed.** A `LINE_STRIP` can have a color per point (the `colors` field, the same length as `points`).
   Store a timestamp with each breadcrumb and color slow sections blue and fast sections red.
4. **Clear the trail.** Add a `std_srvs/srv/Trigger` service `breadcrumbs/clear` that empties the trail and resets
   the distance. (Lesson 6 covers services in depth. This is a preview.)
5. **Challenge: distance to the mining zone.** Mission control publishes the arena zones as markers on `/arena_zones`
   (see [`zone_pub.hpp`](../../../../lance/src/mission_control/zone_pub.hpp)). The mining zone is the marker with
   `ns: zones`, `id: 2`. Its `pose.position` is the center of the box, and `scale` is its size. Subscribe to it, and show
   the distance from the robot to the nearest edge of the mining zone in your label. The zones are in the `map` frame, so this
   only works once the robot has localized. Ask a mentor how to trigger localization in the sim.

## Reference solution
[`solution/breadcrumbs/`](solution/breadcrumbs/) covers Steps 1–7 and stretch goal 1.

**Next:** [Lesson 4: Port a Python node to C++](../04-hopper-sensor/README.md)
