# Sandbox

Your workspace for training lessons. Copy starter packages here, for example:

```bash
cp -r src/project/training/lessons/01-motor-monitor/starter/motor_monitor src/project/training/sandbox/
```

Everything in this folder except this README is **gitignored**, so your work stays on your machine.
`colcon` still finds and builds packages here, so they work like any other package in the workspace.

> [!NOTE]
> `./src/build.sh` builds the sandbox too. If a half-finished lesson package breaks the full build,
> finish it, delete it, or temporarily add an empty `COLCON_IGNORE` file inside it.
