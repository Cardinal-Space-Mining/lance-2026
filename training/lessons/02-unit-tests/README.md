# Lesson 2: Unit Testing the Stall Analyzer

**Time:** ~3–4 hours &nbsp;·&nbsp; **Prerequisite:** [Lesson 1](../01-motor-monitor/README.md)

## What you'll do
The robot has a **stall analyzer** that decides when a track or the trencher is stuck. It has a test file,
[`lance/test/stall_analyzer_test.cpp`](../../../lance/test/stall_analyzer_test.cpp), but the whole file is
commented out and never runs. You'll:

1. read and understand the stall detection logic
2. turn the existing tests back on and get them running with `colcon test`
3. prove the tests can actually catch a bug
4. write new tests for behavior nobody has tested yet
5. test the robot's kinematics helper functions, which are C++ templates

When you're done, `colcon test-result` shows a row of passing tests, all checking real robot code without a robot
or a simulator.

```
Summary: 24 tests, 0 errors, 0 failures, 0 skipped
```
(Your count will differ depending on how many tests you write.)

## Why it matters
When the stall constraint is enabled, [`MiningConstraints`](../../../lance/src/robot/control/shared/mining_controller.cpp)
(around line 128) **ends the mining run** as soon as `stall_state.anyStalled()` is true. (It can be toggled from the
controller during mining.) A false positive wastes competition time. A missed stall can grind a motor until it overheats.

The team kept stall detection out of its competition runs because the code hadn't been tested, and untested
code on competition day is a risk nobody wants. Unit tests are how code earns that trust: they run in
milliseconds, need no hardware, and catch regressions every time anyone changes the code. You're testing
real robot code here, not a toy example.

## Background
Read before starting (about 40 minutes):
- [GoogleTest primer](https://google.github.io/googletest/primer.html). Read up to and including "Test Fixtures".
- [Writing basic tests with C++ (ROS docs)](https://docs.ros.org/en/lyrical/Developer-Tools/Debugging/Testing/Cpp.html)
- [Running tests from the command line (ROS docs)](https://docs.ros.org/en/lyrical/Developer-Tools/Debugging/Testing/CLI.html)
- Keep the [GoogleTest assertions reference](https://google.github.io/googletest/reference/assertions.html) open while you work.

### C++ you'll use
| Concept | Where you'll see it |
|---------|---------------------|
| [Header vs. source files](https://www.learncpp.com/cpp-tutorial/header-files/) | `stall_analyzer.hpp` *declares* the class. `stall_analyzer.cpp` *defines* (implements) it |
| Static libraries and linking | The robot code is compiled into `lance2_core`, which your test links against |
| `enum class` | `TrackSide::LEFT` / `TrackSide::RIGHT` |
| Designated initializers | `{ .debounce_time_seconds = 0.2, ... }` in the test's `makeConfig()` |
| Anonymous namespaces | `namespace { ... }` keeps test helpers private to one file |
| Templates | `maxStartVel<F>` works for both `float` and `double` |
| Floating-point comparison | Why `0.1 + 0.2 != 0.3`, and what `EXPECT_NEAR` is for |

---

## Part 1: Understand the code (no coding)
Open [`stall_analyzer.hpp`](../../../lance/src/robot/core/stall_analyzer.hpp) and
[`stall_analyzer.cpp`](../../../lance/src/robot/core/stall_analyzer.cpp). The interesting function is
`StallAnalyzer::analyzeMotor`. Read it slowly and answer these questions in writing.

1. A motor is a **stall candidate** only when *four* things are all true. What are they?
2. What is `velocity_proportion`? If the command is `10` rps and the motor is spinning at `3` rps, what is it?
   What if the motor spins at `-3` rps?
3. "Debouncing" means a condition has to hold for some time before we act on it. Why would we want that
   here? What could go wrong without it?
4. Once a motor *is* stalled, what has to happen for it to become un-stalled?
5. `analyzeTrack` and `analyzeTrencher` both call `analyzeMotor`. What's different between them?
6. Where does `StallAnalyzerConfig` get its numbers on the real robot? (Hint: `fromParams`.)

<details>
<summary>Check your answers</summary>

1. The command is in `VELOCITY` mode; the command is outside the deadzone; the motor reports a
   `stator_current_limit_fault`; and `velocity_proportion` is below the configured minimum.
2. Actual velocity divided by commanded velocity: `0.3`. With `-3` rps it's `-0.3`. That's still below the minimum,
   so a motor spinning the *wrong way* counts as stalled.
3. Current spikes are normal for an instant, for example when a motor starts or the trencher hits a rock. Without
   debouncing, every spike would end the mining run.
4. The stall condition must be false continuously for the debounce time (`recovery_candidate_seconds`).
5. They use different `MotorConfig`s (`config.tracks` vs `config.trencher`) and separate `MotorState`s.
6. From ROS parameters, through `RobotParams`. The values are in the `stall_analyzer` section of
   [`lance/config/actions/robot_control.json`](../../../lance/config/actions/robot_control.json).
</details>

---

## Part 2: Turn the tests back on
You'll be changing the real `lance` package, so **make a branch first**:
```bash
cd ~/code/lance-ws/src
git switch -c training/<your-name>-lesson2
```

### 2a. Uncomment the test file
Open `lance/test/stall_analyzer_test.cpp`, select everything (`Ctrl` + `A`), and toggle comments
(`Ctrl` + `/`). **Read the tests.** Each `TEST(Suite, Name)` is a separate test. The small `make...()`
functions at the top build messages so each test stays short.

### 2b. Tell ROS that `lance` has tests
In `lance/package.xml`, un-comment the gtest line:
```xml
<test_depend>ament_cmake_gtest</test_depend>
```

### 2c. Tell CMake how to build the test
In `lance/CMakeLists.txt`, add this block **after** the `lance_build_target(...)` calls and **before** the
`# --- Export ---` section:

```cmake
# --- Tests --------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Tests link against one robot version's core lib (LANCE-2 if it's built)
    if(BUILD_LANCE2)
        set(test_core_lib lance2_core)
    else()
        set(test_core_lib lance1_core)
    endif()

    ament_add_gtest(stall_analyzer_test
        "test/stall_analyzer_test.cpp")
    target_link_libraries(stall_analyzer_test
        ${test_core_lib})
endif()
```

Line by line:
- `BUILD_TESTING` is on by default. Tests can be skipped with `--cmake-args -DBUILD_TESTING=OFF`.
- `ament_add_gtest` creates an executable from your test file, links GoogleTest into it, and registers it so
  `colcon test` knows to run it.
- `target_link_libraries(... lance2_core)` links in the compiled robot code. That's where the **definitions** of
  `StallAnalyzer`'s functions live. The test only `#include`s the header, which has the **declarations**.
  Try removing this line later and read the "undefined reference" linker error you get. It's worth recognizing.
- `lance_build_target(N)` is what creates `lance1_core` / `lance2_core`, and you can't link against a target that
  doesn't exist yet. That's why this block goes after those calls.

### 2d. Build and run
```bash
cd ~/code/lance-ws
colcon build --symlink-install --packages-select lance --cmake-args -DROBOT_TARGET=LANCE2
colcon test --packages-select lance
colcon test-result --verbose
```
`-DROBOT_TARGET=LANCE2` builds only the LANCE-2 version, which halves the build time. The first build of `lance`
takes about a minute; after that only changed files rebuild.

`colcon test` saves results to files, and `colcon test-result` summarizes them. For faster feedback with
colored output, run the test executable directly:
```bash
./build/lance/stall_analyzer_test
```

✅ **Checkpoint:** All 7 original tests pass.

---

## Part 3: Make sure the tests can fail
A test that can never fail is worse than no test, because it gives you false confidence. Prove these work by
**breaking the code on purpose**:

1. In `stall_analyzer.cpp`, find
   `state.stall_candidate_seconds >= motor_config.debounce_time_seconds` and change `>=` to `>`.
2. Rebuild and run `./build/lance/stall_analyzer_test`.
3. Which tests fail? Read one failure message and explain *why* that one-character change breaks it.
   (Hint: look at the `dt` values the tests pass in, compared with the debounce time in `makeConfig()`.)
4. Put the code back: `git restore lance/src/robot/core/stall_analyzer.cpp`.

Try one more "mutation" of your own. For example, delete the `faults.stator_current_limit_fault &&` line. Do the
tests catch it?

✅ **Checkpoint:** You've seen the tests fail for a real reason and pass again after the fix.

---

## Part 4: Write new stall analyzer tests
The existing tests skip a lot of behavior. Add a new test for each item below. Put them at the
bottom of the file, inside the namespaces, and **run the tests after each one**.

Here's the first one, done for you:
```cpp
TEST(StallAnalyzerTest, NonVelocityModesNeverStall)
{
    StallAnalyzer analyzer{makeConfig()};
    TalonCtrlMsg command;
    command.mode = TalonCtrlMsg::PERCENT_OUTPUT;
    command.value = 10.0;

    // Stall-like conditions for a full second, but not in velocity mode
    for (int i = 0; i < 10; i++)
    {
        EXPECT_FALSE(analyzer
                         .analyzeTrack(
                             TrackSide::LEFT,
                             makeStatus(0.0),
                             makeFaults(true),
                             command,
                             0.1)
                         .is_stalled);
    }
}
```

Now write these yourself:

| Test name | What it should check |
|-----------|---------------------|
| `LeftAndRightTracksAreIndependent` | Stalling the left track doesn't make the right track report a stall |
| `TimeStalledAccumulates` | `time_stalled_seconds` equals the debounce time when a stall starts, then grows by `dt` each call. Use `EXPECT_NEAR`, not `EXPECT_EQ`. Why? |
| `NegativeTimeStepsAreIgnored` | A negative `dt` (for example a clock glitch) can't cause a stall |
| `DetectsStallWhileReversing` | With a command of `-10` rps, `-9` rps is fine but `-1` rps is a stall |
| `ResetClearsStall` | After `reset()`, a stalled motor immediately reports not stalled |
| `StallStateTest.AnyStalledReportsTrencherStall` | Use the `StallState` class with `RobotMotorStatus`, `RobotMotorFaults`, and `RobotMotorCommands` (from [`motor_interface.hpp`](../../../lance/src/robot/core/motor_interface.hpp)). Stall only the trencher and check `anyStalled()` |

> [!TIP]
> **`ASSERT_*` vs. `EXPECT_*`:** `EXPECT` records a failure and keeps going, while `ASSERT` stops the test. Use
> `ASSERT` when the rest of the test makes no sense if the check fails, for example "the motor must be
> stalled before I can check how long it's been stalled".

✅ **Checkpoint:** All your new tests pass. For at least one of them, you broke the code on purpose and watched it fail.

---

## Part 5: Test the kinematics templates
[`lance/src/robot/model/kinematics.hpp`](../../../lance/src/robot/model/kinematics.hpp) has small physics
helpers used by the autonomous driving code. They're **templates**, so the same code works for `float` and
`double`, and they're **header-only**, so there's no `.cpp` and nothing to link.

1. Create `lance/test/kinematics_test.cpp` and register it in `CMakeLists.txt`, inside the same
   `if(BUILD_TESTING)` block:
   ```cmake
   # kinematics.hpp is header-only, so there's nothing to link
   ament_add_gtest(kinematics_test
       "test/kinematics_test.cpp")
   ```
2. Write tests for:
   - **`maxStartVel(end_vel, dist, decel)`**: work out expected values **by hand** with `v² = v₀² + 2ax`.
     Choose numbers that give round answers. (`maxStartVel(3.0, 2.0, 4.0)` should be exactly `5`. Why?)
   - **`decellDist(vel, decel)`**: also check that a negative `decel` gives the same answer as a positive one.
   - **Round trip**: `decellDist(maxStartVel(0, x, a), a)` should give back `x`. Test it with `float`
     and several values of `x`. Do you need `EXPECT_NEAR`?
   - **`applyTrackLimits(Vl, Vr, Vmax, Vl_out, Vr_out)`** (the 5-argument version):
     - targets within the limit come out unchanged
     - `(2, 1)` with `Vmax = 1` becomes `(1, 0.5)`. The ratio stays the same, so the robot still drives the same arc
     - a negative target with the larger magnitude, like `(-3, 1.5)`, scales correctly
     - zero targets like `(0, 0)` and `(0, 3)` don't produce `NaN`. (Look at the code. Where *could* a divide by zero
       happen, and why is it OK?)

✅ **Final checkpoint:**
```bash
colcon build --symlink-install --packages-select lance --cmake-args -DROBOT_TARGET=LANCE2
colcon test --packages-select lance
colcon test-result --verbose
```
Everything passes. Take a screenshot and show your mentor.

---

## Wrapping up
Commit your work on your branch:
```bash
cd ~/code/lance-ws/src
git add lance/package.xml lance/CMakeLists.txt lance/test/
git commit -m "Re-enable stall analyzer tests, add kinematics tests"
```
This branch is **for practice and stays unmerged**, so don't open a pull request. Show your mentor your branch
and your test results instead. Compare your tests with another trainee's: did either of you catch behavior
the other missed?

To get back to the main code afterwards: `git switch main`. Your branch is still there if you want to come back to it.

## Check your understanding
1. Your test only `#include`s `stall_analyzer.hpp`. Why does it still need to link against `lance2_core`,
   while `kinematics_test` doesn't need to link anything?
2. Why is `EXPECT_EQ(0.1 + 0.2, 0.3)` a bad test? When is `EXPECT_DOUBLE_EQ` okay to use?
3. `MotorState::initialized` is set but never read. What would you do about it, and how would tests help you
   make that change safely?
4. What is something about stall detection that unit tests *can't* tell you? (Think about the real robot.)
5. Why does each test create its own `StallAnalyzer` instead of sharing one?

## Stretch goals
1. **Detective work 🔍.** `kinematics.hpp` has a second `applyTrackLimits` with 8 arguments, and the author
   left a comment: `WARNING: BROKEN.` (It isn't used anywhere, so it's safe to poke at.) Find an input where its
   output is clearly wrong, and write a test that proves it. Name the test `DISABLED_...` so the suite stays green,
   then run it on purpose with `./build/lance/kinematics_test --gtest_also_run_disabled_tests`.
   <details><summary>Hint</summary>

   Try a target that spins the robot in place one way (`Vl = 1, Vr = -1`) when the previous velocities and
   the per-step limit (`Vd_max`) make that impossible right now. Which way does the output turn? Look at step 3
   of the function. Can `t` be negative, and what does a negative `t` do to the motion?
   </details>
2. **Test fixtures.** Many stall tests repeat the same setup. Rewrite some of them using a `TEST_F` fixture.
3. **Typed tests.** Use `TYPED_TEST` to run your kinematics tests for both `float` and `double` without
   copy-pasting them. See [Advanced GoogleTest](https://google.github.io/googletest/advanced.html).
4. **Table-driven tests.** Use `TEST_P` to check `velocity_proportion` thresholds against a table of
   (command, velocity, expected) rows.

## Reference solution
- [`solution/test/stall_analyzer_test.cpp`](solution/test/stall_analyzer_test.cpp): the original tests plus the Part 4 tests
- [`solution/test/kinematics_test.cpp`](solution/test/kinematics_test.cpp): Part 5 plus the detective stretch goal
- [`solution/lance-test-setup.patch`](solution/lance-test-setup.patch): the `package.xml` and `CMakeLists.txt` changes.
  Apply from `src/` with `git apply training/lessons/02-unit-tests/solution/lance-test-setup.patch`

> [!NOTE]
> **For mentors:** This lesson relies on `lance/test/stall_analyzer_test.cpp` staying commented out and the test
> setup staying out of `lance/CMakeLists.txt` on `main`. If that ever changes, have trainees start from an older
> commit (`git switch -c training/<name> <commit>`), or change Part 2 to "read how the tests are wired up".

**Next:** [Lesson 3: Breadcrumb trail visualizer](../03-breadcrumbs/README.md)
