# Sandbox

Your workspace for training lessons. Copy starter packages here, for example:

```bash
cp -r src/project/training/lessons/01-motor-monitor/starter/motor_monitor src/project/training/sandbox/
```

`colcon` finds and builds packages here, so they work like any other package in the workspace.

Your work here is tracked by git. Commit it on **your own branches** (`training/<you>/...`), never on `main`.
See [GIT.md](../GIT.md).

> [!NOTE]
> `./src/build.sh` builds the sandbox too. If a half-finished lesson package breaks the full build,
> finish it, switch to a branch that doesn't have it, or temporarily add an empty `COLCON_IGNORE` file inside it.
