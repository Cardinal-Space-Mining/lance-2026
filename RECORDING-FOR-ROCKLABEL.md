# Recording MCAPs for rocklabel

Short version: **record on the laptop (client), not on the robot.** The client
preset already writes exactly what rocklabel needs, and the robot-side `raw`
preset writes something rocklabel mis-reads unless you change its config.

## The command

On the laptop, with the robot code already running:

```bash
./src/run.sh --local client:=lance2_ksc redux:=client_panda
```

Swap the `redux:=` target for whichever machine is actually running the robot
code (`client_panda` = mochapanda at 10.11.11.13, `client_mac_eth` /
`client_mac_wifi` = the Mac VM, `client_dev_1` = dev_1). Swap `ksc` for
`ucf_left` / `ucf_right` for the arena you're in. Everything else can stay.

The `client:=lance2_*` presets pull in `bag_record:=all`, which runs

```
ros2 bag record -s mcap -o bag_recordings/lance_all_data_<timestamp> <topics...> --all-services
```

The file lands in `lance-ws/bag_recordings/lance_all_data_<timestamp>/`. Copy
the whole directory, or just the `.mcap` inside it — rocklabel only reads the
`.mcap` and ignores `metadata.yaml`.

On the robot side, any `robot:=lance2_*` preset is fine; it does not need to be
recording anything for this.

## What rocklabel actually requires

Three things, all of which `bag_record:=all` already captures:

| | |
|---|---|
| `/multiscan/lidar_scan` | `sensor_msgs/msg/PointCloud2` |
| `/tf` | `odom -> base_link`, published every scan |
| `/tf_static` | `base_link -> lidar_link` |

MCAP storage, not sqlite3 — `get_bag_record_action` in launch-utils passes
`-s mcap` by default, so this is already the case.

Check a recording before you build a dataset from it:

```bash
rocklabel inspect bag_recordings/lance_all_data_<timestamp>/*.mcap
```

It prints the topic list, the point-cloud field layout and the TF tree, and
says outright whether the frames it needs are present. A good one ends with
`configured frames ('odom', 'base_link', 'lidar_link'): all present`.

## Presets that do NOT work

- **`robot:=lance1_*`** → `bag_record:=telemetry`. No LiDAR, no TF. Useless here.
- **`robot:=lance2_*`** → `bag_record:=raw`, i.e. `ros2 bag record --all` on the
  robot. It has the right topics, but the driver's cloud is a fixed 5040-point
  grid that includes every beam that got **no return**, written as (0, 0, 0).
  That is ~8% of every scan, and rocklabel has no reason to treat those as
  anything but real measurements, so they pile into a dense blob sitting on the
  sensor. Only use a robot-side recording if you set `min_range_m: 0.7` in the
  rocklabel config, which drops points near the robot base.
- **`--local` without a `redux:=` target**, or the redux link down. The laptop
  then records a bag with no `/multiscan/lidar_scan` in it at all.

## Why the reduced cloud is fine

The client does not see the LiDAR directly. `net-adapter`'s MS136 scan adapter
(`src/net-adapter/src/adapters/ms136_scan_adapter.cpp`) compresses each scan
down to a range-per-beam buffer to get it over the network, and the client
re-projects it into a 4-field cloud (`x`, `y`, `z`, `reflective`).

That sounds lossy, and geometrically it is not. The re-projection uses the same
per-layer elevation and azimuth tables the driver itself uses, and range is
quantized to 1 mm. Reconstructing a robot-side raw scan through that path and
comparing against the original cloud: **median error 0 cm, worst case 1.1 mm**,
over 90k points. It also drops the no-return points described above, which is
the behaviour you want.

Two things genuinely are lost, neither of which matters for rocklabel:

- The `intensity` channel. The client's `reflective` field is a 0/1 reflector
  flag, not a brightness value, and rocklabel correctly declines to read it as
  intensity — it warns and writes `"intensity_available": false`. Two separate
  training sweeps found the brightness channel earns nothing, so this costs no
  accuracy.
- Returns closer than 0.2 m or further than 32.767 m. Both are outside anything
  the arena contains.
