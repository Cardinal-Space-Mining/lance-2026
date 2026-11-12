# LANCE Software Training

A set of hands-on mini-projects for learning **C++** and **ROS 2** on the LANCE codebase.
Every lesson ends with something you can see working on the simulated robot.

You don't need robot hardware. Everything runs in the Gazebo simulation, and some lessons can run with just the
lightweight motor simulator.

## Before you start
Follow the [full install guide](../doc/full-install-guide.md) until you can run the simulation
and connect Foxglove. If you're on Windows, also read the [WSL guide](../doc/wsl-guide.md).

You'll also need a **GitHub account** with access to the team's repository: you'll save each lesson on your own
branch and push it for review. Lesson 0 walks you through it, and [GIT.md](GIT.md) is the reference for everything git.

**New to C++?** These lessons assume you can write loops, functions, and simple classes. If you can't yet, work through
chapters 1–8 of [learncpp.com](https://www.learncpp.com/) first. The lessons link to specific learncpp pages as new
topics come up.

## Lessons

| # | Lesson | You'll build | You'll learn |
|---|--------|--------------|--------------|
| 0 | [Drive the robot](lessons/00-drive-the-robot/README.md) | Nothing yet. You'll explore the running system, and set up git | Workspaces, nodes, topics, messages, services, the `ros2` CLI, your first commit and push |
| 1 | [Motor health monitor](lessons/01-motor-monitor/README.md) | A node that watches motor current and warns when it's too high | Your first C++ node: classes, lambdas, subscriptions, publishers, parameters, timers, QoS, `package.xml`/`CMakeLists.txt` |
| 2 | [Unit testing the stall analyzer](lessons/02-unit-tests/README.md) | Working tests for the robot's stall detection and kinematics code | Headers vs. sources, linking, templates, floating-point comparison, GoogleTest, `colcon test` |
| 3 | [Breadcrumb trail visualizer](lessons/03-breadcrumbs/README.md) | A trail of where the robot has been, drawn in Foxglove | TF2, coordinate frames, sim time, visualization markers, exceptions, `std::optional` |
| 4 | [Port a Python node to C++](lessons/04-hopper-sensor/README.md) | A C++ hopper fullness driver that replays logs or simulates the hopper | Python → C++, parsing untrusted input, test-first development, integrating over time, sim time |
| 5 | [Launch your node with the robot](lessons/05-launch-integration/README.md) | Your Lesson 4 node starting automatically with the simulator | The `launch-utils` JSON config system: actions, presets, inheritance, overrides, and how a launch fails |
| 6 | [Custom messages and services](lessons/06-mining-stats/README.md) | A mining stats node with its own message type, and a reset service | Interface packages, `.msg`/`.srv` files, constants, generated code, services, edge detection |
| 7 | [Click to drive (capstone)](lessons/07-click-to-drive/README.md) | Click in Foxglove and the robot drives there | Differential drive, `atan2` geometry, state machines with hysteresis, proportional control, transforming points with TF, motor-command safety |

## C++ concepts by lesson
For mentors: each lesson introduces a few new C++ ideas and reuses the earlier ones.

| Lesson | New C++ concepts |
|--------|------------------|
| 0 | None. This lesson is about ROS tooling |
| 1 | Classes and inheritance, constructors, `struct`s with default member values, `std::vector`, `std::string`, range-based `for`, `std::shared_ptr`, lambdas and captures (`[this, i]`), `const&` parameters, type aliases (`using`), calling templates (`create_subscription<T>`), `std::chrono` literals, `snprintf` formatting, why capturing an index beats capturing a reference into a vector |
| 2 | Declarations vs. definitions, header vs. source files, static libraries and linker errors, `enum class`, designated initializers, anonymous namespaces, writing for function templates (`float` vs. `double`), floating-point comparison, GoogleTest macros (`EXPECT_*` / `ASSERT_*`, `DISABLED_`) |
| 3 | Exceptions (`try` / `catch`, exception base classes), `std::optional`, `std::unique_ptr` vs. `std::shared_ptr` ownership, member initialization order (and the warnings that catch mistakes), `std::deque`, references into containers (`emplace_back()` returning `T&`), free helper functions, `<cmath>` (`std::hypot`) |
| 4 | Python → C++ translation, `std::string_view`, `std::from_chars` and structured bindings, error codes vs. exceptions (choosing between them), throwing from constructors and catching in `main()`, `std::ifstream` / `std::getline`, `rclcpp::Time` arithmetic, `<random>` (`mt19937`, `normal_distribution`), `std::clamp`, Euler integration, `static_cast<unsigned char>` and undefined behavior with `<cctype>` |
| 5 | None new. This lesson is JSON configuration and a little Python (the launch file) |
| 6 | Generated code (structs built from `.msg` files), nested types (`Srv::Request`/`Response`), `static constexpr` class constants, `switch`, fixed-width integers (`uint8_t`, `uint32_t`), explicit narrowing with `static_cast<float>`, edge detection, `std::optional::value_or`, callbacks that fill in an output parameter |
| 7 | `enum class` state machines with `switch` (and the `-Wswitch` warning), nested types inside a class, `static` member functions, `std::numbers::pi`, `std::atan2` / `std::hypot` / `std::clamp`, `std::optional::reset()` |

## Git skills by lesson
Each lesson also practices part of the team's git workflow, in VS Code. See [GIT.md](GIT.md).

| Lesson | Git skill |
|--------|-----------|
| 0 | GitHub access, `user.name`/`user.email`, the Source Control view, creating your own branch, first commit, **Publish Branch** |
| 1 | A branch per lesson, staging only what belongs together, commit messages, reading diffs, merging into your own main |
| 2 | Discarding changes, stashing, using diffs to check a deliberate break, committing changes to shared code on a practice branch |
| 3 | **First pull request**, into your own main (choosing the base branch!), and asking for review |
| 4 | Responding to review: more commits on an open PR, **Commit (Amend)** |
| 5 | **A merge conflict**, created on purpose and resolved in the merge editor, then checking the result still works |
| 6 | Keeping up with the team: fetch and merge `origin/main`, the history graph |
| 7 | Final PR, then cleaning up merged branches |

> [!NOTE]
> **For mentors:**
> - Trainees need **write access** to the repository (add them to a team) to push branches.
> - `main` should be **protected** (require a pull request), so a mistaken `git push origin main` is rejected.
> - Trainee branches are named `training/<github-username>/...`. That means no branch may be named exactly
>   `training`: git can't have both `training` and `training/...`.
> - Lesson 0 has trainees start `training/<you>/main` from `origin/main`. If the training material isn't on `main`
>   yet, tell them which branch to start from instead.
> - Reviewing PRs is the main way to check work from Lesson 3 on. Each lesson's "Check your understanding" answers go
>   in the PR description.

## How the lessons are laid out

```
project/training/
├── README.md               ← you are here
├── GIT.md                  ← how to save and share your work with git
├── sandbox/                ← YOUR work goes here (committed on your own branches)
├── tools/                  ← helper scripts (e.g. preview_launch_config.py)
└── lessons/                ← lesson material (ignored by colcon)
    └── 01-motor-monitor/
        ├── README.md       ← the lesson
        ├── starter/        ← copy this into sandbox/ to begin
        └── solution/       ← reference solution. Try it yourself first!
```

Each lesson has:
1. **Why it matters**: where the concept appears in the real robot code.
2. **Background**: just enough theory, plus links to the official ROS 2 docs.
3. **Steps with checkpoints**: build and run after every step. Each step does something you can see.
4. **Check your understanding**: questions to discuss with a mentor (from Lesson 3 on, answer them in your pull request).
5. **Stretch goals**: for when you finish early.

## Tips
- **Build often, commit often.** Small steps make compiler errors easy to understand, and commits at each checkpoint
  mean you can always get back to something that worked.
- **Read the compiler error from the top.** The first error is usually the real one. Later ones are often fallout.
- **Ask for help early.** If you're stuck for more than 20 minutes, ask in the team channel. Include what you tried and the exact error message.
- **Isolate your ROS network when testing at the lab.** If the real robot might be on the same network, run
  `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in every terminal you use (or use `./src/run.sh --local`)
  so you never talk to it by accident.
