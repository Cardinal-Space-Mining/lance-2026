# AGENTS.md

Guidance for AI coding agents (and humans) working in the LANCE robot repo. The
[README](README.md) is the user-facing setup guide; this file covers how to work
in the code.

## What this is

ROS 2 workspace for LANCE-2 (2026), the team's lunar-mining competition robot, with
backward compatibility for LANCE-1. It runs on two machines: the **robot** and a
**client** (driver station). They talk through `net_adapter` over zenoh, not
through ROS discovery.

- Target: **ROS 2 Lyrical on Ubuntu 26.04** (native). **Jazzy** (Ubuntu 24.04)
  must keep building too, because CI builds both.
- This repo is cloned as `<ws>/src`. Run all commands from the **workspace root**
  (`<ws>`, the parent of this directory), not from `src/`.

## Layout

| Path | Package | What it is |
|---|---|---|
| `lance/` | `lance` | Robot control, mission control, motor sim, client node; all launch config (`config/`) and the single launch file |
| `net-adapter/` | `net_adapter` | Robot/client endpoints over zenoh; per-topic adapters; the MS136 lidar scan codec (`src/util/ms136_codec/`) and its offline tools (`tools/`) |
| `multiscan-driver/` | `multiscan_driver` | SICK multiScan136 lidar driver. *Submodule.* See its README; `include/multiscan_driver/multiscan_spec.hpp` defines the dense-buffer reduction the codec relies on |
| `phoenix-driver/` | `phoenix_ros_driver` | CTRE Phoenix 5/6 motor controller driver (CAN) |
| `cardinal-perception/` | `cardinal_perception` | Localization / mapping. *Submodule* (branch `functionality`) |
| `launch-utils/` | `launch_utils` | JSON preset launch framework. *Submodule* |
| `csm-utils/`, `csm-metrics/` | `csm_utils`, `csm_metrics` | Shared helpers, process/profiling metrics. *Submodules* |
| `orca/` | — | Orca, the web app that runs the robot from a laptop (launch, deploy, logs, config editing) over SSH. *Submodule*, not a ROS package (`COLCON_IGNORE`); its team files are in `project/orca/`. See its README |
| `csm-sim/` | `csm_sim` | Gazebo sim. **Separate GitLab repo, gitignored here** (`git clone https://gitlab.com/csm2.0/csm-sim` into `src/`) |
| `project/` | — | Non-package files: install docs (`doc/`), `cyclonedds.xml`, Foxglove layout, training curriculum (`training/`), Orca's machine inventory and launch scenarios (`orca/`) |
| `build.sh`, `run.sh`, `sync.sh` | — | Build, launch, and rsync-deploy scripts |

## Building

```bash
source /opt/ros/lyrical/setup.bash   # or jazzy
./src/build.sh                       # full build (colcon, --symlink-install)
```

`build.sh` flags:
- `--l1` / `--l2` / `--la`: build `lance` for LANCE-1, LANCE-2, or both (the default). Each target is compiled separately, so building one halves `lance` build time.
- `--no-sim`: skip `csm_sim`.
- `--tools` / `--no-tools`: build or skip the offline net-adapter tools, such as `ms136_codec_train`. **This setting persists in the CMake cache** until changed.
- `--native-arch`: build with `-march=native`. Don't deploy those binaries to another machine.
- `--clean`: deletes `build/ install/ log/`, which includes the codec scan cache in `build/ms136_scans`.

`build.sh` builds everything except `lance` first, then `lance` on its own. To
iterate on a single package, use the same flags as the script so the CMake cache
and symlinks stay consistent:

```bash
colcon build --symlink-install --packages-select net_adapter \
    --cmake-args -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
```

Compile commands are exported for IDEs (`build/compile_commands.json`).
Packages call `standard_compile_options()` from `csm_utils`: C++20, `Release`
unless set otherwise, and `-Wall -Wextra -Wpedantic`. Warnings aren't errors,
but keep new code warning-free.

## Running

```bash
./src/run.sh [--local] [--canbus] <preset>:=<name> [<action>:=<preset> ...]
```

- `--local`: CycloneDDS with localhost-only discovery (`project/cyclonedds.xml`). **Use this whenever another ROS machine might be on the network** (the lab, or a two-machine sim test). Otherwise DDS bridges topics directly between machines and bypasses `net_adapter`.
- `--canbus`: CAN bringup and teardown around the run (real motors only).
- **Presets** select whole setups: `robot:=`, `client:=`, `gz_full:=`, `gz_robot:=`, `gz_client:=`, `dev:=`, `replay:=`. Names are `<lance1|lance2>_<ksc|ucf_left|ucf_right>`.
- **Actions** swap out one component and override the preset's choice, e.g. `foxglove_bridge:=null`, `redux:=client_mac_eth`, `gz_gui:=enabled`.

Common runs:
```bash
./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled          # full sim on one machine
./src/run.sh dev:=motor_sim                                # control logic, simulated motors only
./src/run.sh replay:=lance2_perception bag:=<path>         # perception on a recorded bag
./src/run.sh --canbus --local robot:=lance2_ksc            # real robot
./src/run.sh --local client:=lance2_ksc                    # driver station
```

### Launch config

- Everything is in `lance/config/`: `launch.json` imports `actions/*.json` (one file per action, each with named presets) and `presets/*.json` (top-level setups that set several actions at once via `pragma:action_overrides`). `lance/launch/lance.launch.py` turns the resolved config into nodes.
- **The CLI can only choose presets, not individual values.** To change a value such as an IP or a parameter, add a preset that uses `pragma:derived` from an existing one and select it. Shared constants (IPs, frame IDs, topic names, arena zones) live in `actions/shared.json` under `pragma:constants` and are referenced as `${ ip.net1.lidar }`.
- Preview what a set of overrides resolves to without launching:
  `python3 src/project/training/tools/preview_launch_config.py gz_full:=lance2_ksc show:=motor_sim`
  (run after `source install/setup.bash`).
- Framework docs: `launch-utils/README.md`.
- `multiscan_driver` presets: `net1` and `net2` (which sensor IP) set every
  driver parameter (see `multiscan-driver/config/params.yaml` for what each
  means). `<net>_ghost_drop` and `<net>_ghost_annotate` turn on its ghost
  point filter. Select one with `multiscan_driver:=<preset>`.

### Robot ⇄ client networking

- `net_adapter`'s `robot_endpoint` and `client_endpoint` are zenoh peers. Each listens on **TCP 7447** and connects to the other side's IP. The IPs come from the `redux` action presets (`client_hostname` on the robot side, `robot_hostname` on the client side).
- In sim (`is_sim: true`), the endpoints use the sim lidar adapter and also forward the sim clock.
- Deploy code to the robot with `./src/sync.sh push --ws -u <user> -h <host> -r <remote ws>`. Pull recorded bags back with `./src/sync.sh pull --bags ...`. Bags are written to `bag_recordings/` in the directory you launched from (gitignored).

## Testing

- `net_adapter` has the only gtest suite (`test_ms136_codec`):
  ```bash
  colcon test --packages-select net_adapter && colcon test-result --verbose --test-result-base build/net_adapter
  ```
- CI (`.github/workflows/ros-ci.yaml`) only **builds**, in Docker, for Jazzy and Lyrical on x86 (arm64 builds run on manual dispatch). Nothing runs the tests automatically, so run them locally when you touch the codec.
- There is no lidar/CAN hardware in CI or on most dev machines. Use the Gazebo sim, `dev:=motor_sim`, or bag replay to check behaviour, and say clearly what could only be verified on hardware.

## MS136 lidar codec

Lidar scans are the dominant traffic between robot and client. They are reduced
to a 14×360 dense buffer (`multiscan_spec.hpp`, `ms136::redux`) and entropy-coded
with trained probability tables (`net-adapter/src/util/ms136_codec/`; see
`ms136_codec.hpp` for the format and design).

- **`ms136_codec_tables.hpp` is generated. Never edit it by hand.**
- Retrain: `./src/build.sh --tools`, then run `python3 src/net-adapter/tools/ms136_extract_scans.py` with no arguments (needs `python3-tk`). Its GUI picks bags, extracts and caches scans, trains, writes the header, and optionally rebuilds and runs the test. Command-line equivalents are documented in `tools/ms136_codec_train.cpp`.
- **Bump `TABLE_VERSION` whenever the tables change.** Bump `FORMAT_VERSION` as well if the model or wire format changes. The decoder rejects mismatched versions, so **the robot and client must be deployed together.**
- Prefer **raw** bags (straight from the driver, with `layer/index/range/reflective` fields) over client-side decoded ones. The extractor reproduces `MS136ScanAdapter::serializeMsg` exactly for raw clouds. If you change `reducePoint()` or `serializeMsg`, update `cloud_to_dense()` in the extractor to match and bump its `EXTRACT_VERSION`, which invalidates cached scans.

## Conventions

- C++: follow [Code style](#code-style) below.
- Branch from `main` and open PRs into `main`. Commit subjects are short, lower-case and imperative ("add bandwidth monitoring").
- **Submodules:** to change `multiscan-driver`, `cardinal-perception`, `launch-utils`, `csm-utils`, `csm-metrics` or `orca`, commit inside the submodule (on a branch, and push it there), then commit the updated submodule pointer here. Don't leave the parent pointing at an unpushed commit.
- `multiscan-driver` no longer contains vendored sick_scan_xd code; its decoder is checked against the old driver's output (`test/data/golden_clouds.txt`) and its SOPAS client against a recorded transcript of the real sensor. Run `colcon test --packages-select multiscan_driver` after changing it.
- Python launch/tool scripts use the system ROS Python. Don't add pip-only dependencies without updating the install docs (`project/doc/`) and the rosdep keys in `package.xml`.

## Code style

The reference is the `lance` package (`lance/src`): code should read like
it. These rules apply to the C++ in every package here. Not all existing
code follows them yet: apply them to code you write or change, and don't
restyle unrelated files in the same change.

`multiscan-driver/AGENTS.md` carries the same section, because the driver
is also used outside this workspace. Keep the two in step.

### Formatting

- Format every new or changed C++ file with `clang-format -i` and the
  repository's `.clang-format`, a copy of the LANCE workspace's:
  Google-based, 4-space indent, 80 columns, Allman braces.
- Always use braces, even for one-line `if` and loop bodies.
- Every C++ file starts with the Cardinal Space Mining copyright banner
  (copy it from any file here). A block comment after it says what the file
  is for and anything a reader must know first.
- Use `#pragma once`. In a `.cpp`, include its own header first. Then group
  includes, separated by blank lines: standard library, third-party (ROS,
  zlib, ...), then this project's headers.

### Names

- **Spell names out.** Write `num_points`, `receive_time_ns`, `datagram`,
  `offset`, `probability` and `config`, not `n`, `recv_ns`, `dgram`, `off`,
  `prob` and `cfg`. Readability is worth the column space.
- Exceptions:
  - `i`, `j` and `k` for plain loop indices;
  - `x`, `y` and `z` for coordinates;
  - names from a specification, explained where they appear (e.g. `phi` and
    `theta` from the operating instructions);
  - `a` and `b` in a sort comparator.
- Put the unit at the end of the name: `_ns`, `_us`, `_ms`, `_s`, `_m`,
  `_mm`, `_deg`, `_rad`, `_hz`, `_bytes`. Use `num_` for counts and `has_`
  or `is_` for booleans.
- Casing:
  - types: `PascalCase`;
  - functions and methods: `camelCase` (`addSegment`, `cellAt`);
  - variables and members: `snake_case`;
  - constants: `UPPER_SNAKE_CASE`;
  - enums: `enum class` with `PascalCase` values.

### Functions

- Keep functions short and about one thing. Break long ones into named steps
  (e.g. `FrameAssembler::add`: decode, move to the host clock, count lost
  telegrams, place the segment, accept it). A function that reads as a list
  of steps is the goal.
- If a function stays long, label its parts with `// --- Step ---` comments.
- Structure control flow plainly: early returns instead of deep nesting, and
  no `goto`.

### Macros over lambdas

- For a small piece of code repeated inside a function or file, prefer a
  macro to a local lambda: declaring parameters, filling fields, repeated
  checks with an early `return` (`EXIT_IF_INSUFFICIENT_SIZE`), or anything
  that needs a name as a string (`#name`). The LANCE package does this
  (`INIT_PARAM`, `ADD_MARKER`, `SET_BOUNDS`).
- Name macros in `UPPER_SNAKE_CASE`. Define them just above their first
  use, and `#undef` them when they are no longer needed. Wrap a multi-line
  body in braces, and put macro arguments in parentheses inside the body.
- Use a real function when the code is longer, needs its own types or
  locals, or is called from more than one file.
- Lambdas are still right where a callable is required: callbacks, thread
  bodies and sort comparators. Name a stored lambda like a function
  (`readList`, `isFloat32`).

### Classes and headers

- Group a class with repeated access labels, in this order:
  - `public:` types;
  - `public:` constructors;
  - `public:` methods (more than one block if they group naturally);
  - `protected:` types, then `protected:` methods, then `protected:` members.
- Use `protected` rather than `private` (the LANCE convention).
- Use a `struct` with public members for plain data. Give it methods only if
  they are small helpers.
- Initialise members in the declaration with braces: `uint32_t
  num_points{0};`.
- Always access members through `this->`.
- Group related members with `// --- Group ---` comments, and say which
  thread owns or which mutex guards shared state.
- Put two blank lines between top-level classes or structs, and one between
  closely related small structs.
- Put `/* ... */` doc comments above declarations, saying what something
  means, its units and its edge cases. Use `//` for short inline notes.

### Source files

- Mark the sections of a `.cpp` with `// --- Section ---` banners padded to
  80 columns, with three blank lines before each. Keep helpers in an
  anonymous namespace at the top.
- Log messages start with the node's prefix (e.g. the driver's
  `LOG_PREFIX`, `"[MULTISCAN DRIVER]: "`).

### Comments

- Explain why, not what: the sensor behaviour, the manual section, or the
  reason for an odd choice. Keep sentences short and plain.

### Refactoring numeric code

- Renaming and splitting must not change results. Keep the exact order of
  floating-point operations: `(a * b) / c` and `a * (b / c)` round
  differently. That alone has broken the driver's golden-cloud test.
- To check a refactor, compare outputs byte for byte against the previous
  version on recorded data, not just with the tests.

### Files that keep their own style

- Generated files, such as `ms136_codec_tables.hpp`.
- `multiscan-driver/include/multiscan_driver/multiscan_spec.hpp` is a public
  header that net_adapter's codec uses. Don't rename its API.
- `csm-sim` is a separate repository with its own conventions.

## Things that are deliberately the way they are

- `lance/test/stall_analyzer_test.cpp` is **commented out on purpose**. Stall detection was kept out of competition runs because it was untested, and training Lesson 2 has trainees re-enable it on a practice branch. Don't re-enable it on `main`.
- There is deliberately **no `hopper_sensor_sim` launch action** on `main`; training Lesson 5 has trainees add it. If you change either of these, update the lesson READMEs in `project/training/lessons/`.
- `project/training/lessons/` is `COLCON_IGNORE`d, so its starter/solution packages never build into the workspace. `project/training/sandbox/` is tracked.
- The hopper fullness sensor publishes a **0–1 fraction**, not a percentage.
- `echo_selection` for the lidar driver is **1-based** (1 = first return), and the driver falls back to the first echo when the sensor sends fewer echoes.
