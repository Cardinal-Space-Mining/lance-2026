# Lesson 5: Launch Your Node With the Robot

**Time:** ~2–3 hours &nbsp;·&nbsp; **Prerequisite:** [Lesson 4](../04-hopper-sensor/README.md). If you skipped it, copy
`lessons/04-hopper-sensor/solution/hopper_sensor_sim` into `sandbox/` and build it.

## What you'll do
So far, you've started your nodes by hand in a separate terminal with `ros2 run`. The real robot doesn't work that
way. One command, `./src/run.sh`, starts *everything*, configured by JSON files. In this lesson you'll make your
hopper sensor from Lesson 4 part of that system, so that:

```bash
./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled
```
starts your node automatically, with the right parameters and `use_sim_time`, and Foxglove's hopper fullness plot
works in the simulator without any extra steps.

This lesson is mostly **JSON and a little Python**, not C++. Understanding how the system is configured is what
lets you add your own work to it, and what lets you debug it when something doesn't start.

## Why it matters
Every node on the robot, from the motor drivers to perception and the controller, is launched this way. When something
"doesn't start" at a test, or starts with the wrong parameters, the cause is almost always in these files. The
same system decides what runs on the **real robot** versus the **simulator**. Getting that wrong could mean a
simulated sensor publishing fake data on the robot, or a real driver missing from it.

## Background
Read the [launch-utils README](../../../../launch-utils/README.md), sections 1–7 (about 20 minutes). The key ideas:

| Term | Meaning |
|------|---------|
| **Action** | Something that can be launched, usually one node, such as `motor_sim` or `robot_control` |
| **Preset** | One named configuration of an action. An action runs with **exactly one** preset, or not at all |
| `pragma:default` | Which preset an action uses if nobody picks one. `null` means "off unless asked for" |
| `pragma:derived` | A preset that copies another and overrides some values (inheritance) |
| `pragma:action_overrides` | A preset that also chooses presets for *other* actions. This is how `gz_full:=lance2_ksc` turns on a dozen things |
| `key:=value` arguments | Pick a preset from the command line: `./src/run.sh motor_sim:=lance2`. These win over everything else |

How a launch works, end to end:
```
./src/run.sh gz_full:=lance2_ksc
  └─ ros2 launch lance lance.launch.py gz_full:=lance2_ksc
       ├─ load lance/config/launch.json, plus every file in its "pragma:import" list
       ├─ preprocess: choose one preset per action (or remove the action), resolve inheritance
       └─ for each enabled action, a Python function in lance.launch.py creates the Node
```

### A tool to help: preview the config
`src/project/training/tools/preview_launch_config.py` runs the same preprocessing step **without launching anything**:
```bash
cd ~/code/lance-ws
source install/setup.bash
python3 src/project/training/tools/preview_launch_config.py gz_full:=lance2_ksc
python3 src/project/training/tools/preview_launch_config.py gz_full:=lance2_ksc show:=motor_sim
```
The first command lists every enabled action. The second shows the final parameters one action receives. Use it
constantly in this lesson. It's much faster than launching Gazebo to check a JSON change.

---

## Part 1: Explore the existing config (no editing)
Open these files and use the preview tool to answer the questions:
- [`lance/config/launch.json`](../../../../lance/config/launch.json)
- [`lance/config/actions/sim.json`](../../../../lance/config/actions/sim.json), the `motor_sim` action
- [`lance/config/presets/gz_sim_presets.json`](../../../../lance/config/presets/gz_sim_presets.json), the `gz_full` presets
- [`lance/launch/lance.launch.py`](../../../../lance/launch/lance.launch.py)

1. `gz_full:=lance2_ksc` enables `motor_sim`. Which line makes that happen, and which preset does it pick?
2. Run the preview with `gz_full:=lance2_ucf_left`. That preset doesn't mention `motor_sim`, so why is it still enabled?
3. Run the preview with `gz_full:=lance2_ksc motor_sim:=null`. What happened, and what does that tell you about
   which wins: the command line or `pragma:action_overrides`?
4. In `lance.launch.py`, follow what happens to the `motor_sim` config. Which function turns it into a `Node`? What
   does it do with the `"model"` key, and why does it `pop` it instead of passing it on as a parameter?
5. Open [`actions/hopper_fullness.json`](../../../../lance/config/actions/hopper_fullness.json), the real sensor's
   Python driver, and the `lance2_ksc` preset in [`robot_presets.json`](../../../../lance/config/presets/robot_presets.json).
   Does the real robot run the hopper fullness driver right now? Now suppose someone misspelled `pragma:default` as
   `prgama:default` in that file. What would the preprocessor do? (Read `preprocess_launch_json` in
   [`launch-utils/src/launch_utils/preprocess.py`](../../../../launch-utils/src/launch_utils/preprocess.py).) Would anyone notice?

<details>
<summary>Check your answers</summary>

1. `"motor_sim": "lance2_gz_sim"` in the `pragma:action_overrides` of the `lance2_ksc` preset under `gz_full`.
2. `lance2_ucf_left` has `"pragma:derived": "lance2_ksc"`, so it inherits all of `lance2_ksc`'s overrides and only
   replaces a few.
3. `motor_sim` was removed. Command-line arguments win over `pragma:action_overrides`, which win over
   `pragma:default`.
4. `get_motor_sim_action`. `model` chooses *which executable* to run (`lance1_motor_sim` or `lance2_motor_sim`).
   It isn't a ROS parameter, so it's removed before the rest of the config becomes the node's parameters.
5. No: the robot preset sets `"hopper_fullness": null`, and the action's own default is `null` too. With the typo, the
   preprocessor wouldn't find a `pragma:default`, so it would treat the default as `None` and disable the action. Here
   that happens to be the intended behavior, so **nobody would notice**. But the same typo in an action that's *supposed*
   to be on by default would silently turn it off, with no error. (This exact typo was in `hopper_fullness.json`
   until recently.) Typos in config keys rarely cause errors, so check the `[LAUNCH PREPROC]` output, or use the
   preview tool.
</details>

---

## Part 2: Create your action
Start `training/<you>/lesson-5` from `training/<you>/main`. Your main should already have your Lesson 4 package. As in
Lesson 2, you'll change the real `lance` package, and that's fine on your own branches. Commit at each checkpoint.

### 2a. The action file
Create `lance/config/actions/hopper_sensor_sim.json`:
```json
{
    "hopper_sensor_sim":
    {
        "pragma:default": null,

        "sim":
        {
            "source": "sim",
            "use_sim_time": true,
            "capacity_l": 50.0,
            "fill_l_per_trencher_rev": 0.005,
            "drain_l_per_belt_rev": 0.03,
            "dig_below_pos": 0.35,
            "dump_above_pos": 0.8,
            "noise_stddev": 0.01
        }
    }
}
```
Everything in a preset, except `pragma:` keys, becomes a **ROS parameter** of the node, with the same names as the
`declare_parameter` calls in your C++.

Why `"pragma:default": null`? This node publishes fake sensor data on a real topic. It must **never** turn on unless
someone asks for it, and it must certainly never turn on by accident on the robot.

### 2b. Derived presets
Add two more presets that **derive** from `sim`:
- `sim_fast_fill`: four times the fill rate, handy for demos.
- `sim_no_noise`: zero noise, handy for checking the model.

Each should set only the value that changes, plus `"pragma:derived": "sim"`.

### 2c. Register the file
Add `"actions/hopper_sensor_sim.json"` to the `pragma:import` list in `lance/config/launch.json`.

✅ **Checkpoint:**
```bash
python3 src/project/training/tools/preview_launch_config.py hopper_sensor_sim:=sim_fast_fill show:=hopper_sensor_sim
```
This shows the full `sim` config with only `fill_l_per_trencher_rev` changed.

> [!NOTE]
> **No rebuild needed.** You didn't rebuild `lance`, and the new file was still found. The workspace is built with
> `--symlink-install`, so `install/.../launch.json` is a *link* to the file in `src/`, and imports are looked up next to
> the real file. Without `--symlink-install`, you'd need to rebuild `lance` after every config change.

---

## Part 3: Teach the launch file about your node
The preprocessor now produces a `hopper_sensor_sim` config, but nothing launches it yet. In `lance.launch.py`, add a
function next to `get_hopper_fullness_action`:
```python
def get_hopper_sensor_sim_action(config):
    return NodeAction(config).format_node(
        package='hopper_sensor_sim',
        executable='hopper_sensor_sim_node',
        output='screen'
    )
```
Then call it from `get_robot_actions`, the same way `hopper_fullness` is handled.

`NodeAction(config).format_node(...)` flattens your preset into ROS parameters and creates a `launch_ros` `Node`.
It's the same as `ros2 run hopper_sensor_sim hopper_sensor_sim_node --ros-args -p source:=sim -p use_sim_time:=true ...`.

✅ **Checkpoint:** Pick your preset from the command line:
```bash
./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled hopper_sensor_sim:=sim
```
Look for `Configured action 'hopper_sensor_sim' with preset 'sim'` and `Simulating a 50.0 L hopper` in the output.
Then, in another terminal:
```bash
ros2 param get /hopper_sensor_sim use_sim_time      # Boolean value is: True
ros2 topic echo /lance/hopper_fullness
```

---

## Part 4: Turn it on automatically in the simulator
Typing `hopper_sensor_sim:=sim` every time is easy to forget. In `gz_sim_presets.json`, add
`"hopper_sensor_sim": "sim"` to the `pragma:action_overrides` of the **`lance2_ksc`** preset under **`gz_full`**.

Before launching, predict the answers, then check them with the preview tool:
1. Is it enabled for `gz_full:=lance2_ucf_left`? Why?
2. Is it enabled for `gz_full:=lance1_ksc`?
3. How would someone turn it **off** for one run without editing any files?
4. Is it enabled for any **robot** preset (`robot_presets.json`)? It must not be!

✅ **Checkpoint:** `./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled` starts your node with no extra arguments. Enable the
robot in Foxglove, then lower the hopper (right stick down) and hold the right trigger: `/lance/hopper_fullness`
climbs. (In testing, 12 s of digging reached about 0.17.)

---

## Part 5: Break it on purpose
What if your config change ends up somewhere your package isn't, for example if it's merged without the package, or a
teammate pulls your branch but doesn't rebuild? Find out by temporarily hiding your installed package:
```bash
cd ~/code/lance-ws
mv install/hopper_sensor_sim install/hopper_sensor_sim.hidden
./src/run.sh gz_full:=lance2_ksc
```
(Deleting the package from `src/` and rebuilding wouldn't be enough: colcon never removes packages that are already
in `install/`. That's a surprise worth remembering.)

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): "package 'hopper_sensor_sim' not found, ...
```
The **entire launch** fails. Gazebo, the controller, everything. One missing package takes down the whole robot. This is
why config changes that refer to a new package must be merged *together with* that package, in the same pull request.
Your branch has both, because your package is committed in the sandbox. But it's also why, after pulling, you should
always rebuild before you launch.

Put it back when you're done: `mv install/hopper_sensor_sim.hidden install/hopper_sensor_sim`.

---

## Part 6: A merge conflict 🔀
On a team, someone else is always changing the same files as you. Usually git combines both changes by itself. When two
people change the **same lines**, git stops and asks you to decide. That's a **merge conflict**. Here you'll make one on
purpose, playing both people, so the first real one isn't a surprise.

### 6a. Be the teammate
1. Commit everything on `training/<you>/lesson-5`. Source Control should show no changes.
2. Switch to `training/<you>/main`. Your Lesson 5 changes disappear from the files. They're safe on the other branch.
3. Your "teammate" wants Foxglove to open by itself in the simulator. In `lance/config/presets/gz_sim_presets.json`,
   change the end of the `lance2_ksc` overrides to:
   ```json
                   "foxglove_bridge": "all",
                   "foxglove_gui": "local"
   ```
4. Commit it with the message `Open Foxglove automatically in the KSC sim`.

### 6b. Merge, and get a conflict
1. Switch back to `training/<you>/lesson-5`.
2. `Ctrl` + `Shift` + `P` → **Git: Merge** → `training/<you>/main`.
3. VS Code reports a conflict. In Source Control, `gz_sim_presets.json` is listed under **Merge Changes**. Open it:
   ```
                   "foxglove_bridge": "all",
   <<<<<<< HEAD
                   "hopper_sensor_sim": "sim"
   =======
                   "foxglove_gui": "local"
   >>>>>>> training/<you>/main
   ```
   The top half is **yours** (the "current" branch, the one you're on), and the bottom half is **theirs** (the
   "incoming" branch, the one you're merging in). Both of you changed the line after `foxglove_bridge`.

### 6c. Resolve it, then check it
1. You want **both** lines, so click **Accept Both Changes** above the conflict.
2. Before you commit, run the preview tool:
   ```bash
   python3 src/project/training/tools/preview_launch_config.py gz_full:=lance2_ksc
   ```
   Look closely at the output. It prints a `Failed to load json data` error, and then **keeps going**, with almost
   nothing enabled. **Accept Both Changes** put the two lines together, but the first one has no comma. The whole
   `gz_sim_presets.json` file is now invalid, and the launch system *skips files it can't read*. A real launch would
   start, print one error that's easy to miss, and run almost nothing.
3. Add the missing comma, and run the preview again. Now both `hopper_sensor_sim` and `foxglove_gui` are enabled.
4. Stage the file (**+**). VS Code has already written a commit message (`Merge branch ...`). Click **Commit**.

✅ **Checkpoint:** `./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled` starts the simulator, your node, *and* opens
Foxglove by itself. (If you're on WSL with Foxglove installed on Windows, it won't open by itself. Check the preview
instead.) The **Graph** in Source Control shows your two branches coming back together.

> [!IMPORTANT]
> **Git only checks that the text merged, not that the result works.** After every conflict, build and run what you
> merged before you push it.

## Part 7: Pull request
Push, and open a PR from `training/<you>/lesson-5` into `training/<you>/main`, with your answers to the questions below.
Your PR's **Files changed** tab doesn't show the Foxglove change, because your main already has it. GitHub only shows
what the PR would *add* to its base.

---

## Check your understanding
1. In what order are presets chosen? Put these in order of priority: `pragma:default`, command-line `key:=value`,
   `pragma:action_overrides`.
2. Why does `hopper_sensor_sim` use `"pragma:default": null` rather than defaulting to `sim`?
3. A teammate changed a value in `hopper_sensor_sim.json`, but the robot didn't pick up the change. Name two
   possible reasons. (Think: which preset actually ran, and how was the workspace built?)
4. What's the difference between something in a preset (a ROS parameter) and something the launch function `pop`s,
   like `motor_sim`'s `model`?
5. Look at the `lance2_sim` preset in `robot_control.json`. How is it like your `sim_no_noise` preset?
6. In Part 6, git merged your branches with no complaints once the conflict markers were gone, but the result was
   broken. What other kinds of changes could merge cleanly but still break the robot?

## Stretch goals
1. **Fail gracefully.** Make `get_hopper_sensor_sim_action` print a warning and skip the node (return `None`, and make
   sure `get_robot_actions` doesn't add `None` to the list) if the package isn't installed. `launch_utils.common` has a
   `does_package_exist(pkg)` helper you can import. Then repeat Part 5. What's the trade-off of skipping silently?
2. **Remapping.** Add a preset that publishes on `sim/hopper_fullness` instead, using `pragma:node_options` with
   `remappings` (see section 6 of the launch-utils README). When would that be useful?
3. **Replay preset.** Add a `replay` preset that uses `source: file` with Lesson 4's sample log. The file path is a
   problem: a relative path is relative to the directory you ran `run.sh` from. How could you make it reliable?
   (Hint: `get_package_share_directory` in the launch function, and installing the file with the package.)
4. **Your other nodes.** Add actions for `motor_monitor` (Lesson 1) and `breadcrumbs` (Lesson 3) the same way.
   `breadcrumbs` would be a good fit for the `gz_full` presets too.

## Reference solution
- [`solution/hopper_sensor_sim.json`](solution/hopper_sensor_sim.json): the action file from Part 2
- [`solution/lance-launch-integration.patch`](solution/lance-launch-integration.patch): all the changes from Parts 2–4. Apply from
  `src/` with `git apply project/training/lessons/05-launch-integration/solution/lance-launch-integration.patch`

> [!NOTE]
> **For mentors:**
> - This lesson relies on the `lance` config *not* already having a `hopper_sensor_sim` action on `main`.
> - For a more realistic Part 6, play the teammate yourself: make the 6a commit on the trainee's `training/<name>/main`
>   with GitHub's web editor while they work on Parts 2–5. Their PR then shows "This branch has conflicts", and they
>   skip 6a and merge their main to fix it.

**Next:** [Lesson 6: Custom messages and services](../06-mining-stats/README.md)
