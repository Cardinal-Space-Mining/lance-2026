## Controls

LANCE is driven with an Xbox-layout gamepad (e.g. a Logitech controller in X mode) through the `/joy` topic. The bindings are defined in [`src/robot/core/hid_bindings.hpp`](src/robot/core/hid_bindings.hpp); if this page and that file disagree, the code is right.

The controller has two layers:

* **Robot controls** (the default): mission control passes the joystick through to the robot. These only do anything while the robot is in **teleop** mode.
* **Mission control override**: press **GUIDE** (the center logo button) to switch to this layer, and press it again to switch back. While override is active, the joystick is *not* passed to the robot. The buttons change the control mode and options or place targets instead.

Button names: **BACK** is the left center button (View), and **START** is the right center button (Menu).

### Robot controls (teleop)

#### Manual driving

* **LEFT STICK**: drives the tracks, arcade style (up/down = forward/back, left/right = turn).
* **RIGHT STICK (up/down)**: raises and lowers the hopper.
* **RIGHT TRIGGER**: spins the trencher. Hold **RB** to reverse it.
* **LEFT TRIGGER**: runs the hopper belt. Hold **LB** to reverse it.
* **B / Y / X**: set the driving speed multiplier to LOW / MEDIUM / HIGH.
* **A** (hold): cancels any assisted routine and stops all motors (soft e-stop). It returns the robot to manual driving.

#### Assisted routines

* **BACK**: starts **Assisted Mining**. The robot lowers the hopper, spins the trencher, and drives forward while running the hopper belt. While it is running:
  * **LEFT STICK (up/down)** speeds up or slows down the forward driving.
  * **RIGHT STICK (up/down)** adjusts the digging depth.
  * **RIGHT TRIGGER** slows the trencher.
  * **LEFT TRIGGER** runs the belt by hand, but only when the hopper-full constraint is turned off (see below). Otherwise the belt is automatic.
  * Press **BACK** again to finish: the robot raises the trencher out of the ground (the left stick can still drive straight while it raises).
  * Mining also stops automatically when an enabled constraint is reached.
* **START**: starts **Assisted Offload**. The robot raises the hopper, runs the belt to dump, and lowers the hopper again. While it is running, the **LEFT STICK (up/down)** drives straight forward or back (turning is disabled). Press **START** again to skip ahead to lowering the hopper.
* **A** (hold) cancels either routine.

#### Mining constraints

Each D-pad direction toggles one condition that can end assisted mining. **All four start enabled.**

* **D-PAD UP**: obstacle (stop before a detected obstacle)
* **D-PAD RIGHT**: hopper full (stop when the hopper reaches its target volume; this also puts the belt under automatic control)
* **D-PAD DOWN**: zone boundary (stop before the trencher leaves the mining zone; needs arena localization)
* **D-PAD LEFT**: motor stall (stop if a motor stalls)

### Mission control override (press GUIDE)

#### Mode and options

* **A**: disable the robot.
* **B**: switch to teleop mode.
* **X**: switch to autonomous mode.
* **Y**: toggle *test mode* (limits how low the hopper goes; for testing and simulation).
* **D-PAD DOWN**: toggle *quick auto* (keeps the time spent in autonomous short, for quick points).
* **D-PAD UP**: toggle *assist as auto* (makes the assisted mining/offload routines run as autonomous routines with preset targets).

When the robot is in **autonomous** mode and override is *not* active, pressing **A** also disables the robot.

#### Placing targets (teleop mode only)

Each of these buttons opens a cursor (shown in Foxglove) for one kind of target. Pressing the same button again closes it without sending anything.

* **RB**: traversal cursor. It starts at the robot's current pose and needs odometry or arena localization.
* **BACK**: mining cursor. It starts at the center of the mining zone and needs arena localization.
* **START**: offload cursor. It starts in the offload zone and needs arena localization.

While a cursor is open:

* **LEFT STICK** moves the cursor and **RIGHT STICK (left/right)** rotates it.
* For the offload cursor only:
  * **RIGHT TRIGGER / LEFT TRIGGER** move the dump position further out or closer in.
  * **LEFT STICK button** re-aligns the cursor to face the offload zone.
* **LB** confirms the target and sends it to the robot.

You can also send a traversal target in teleop mode by clicking a point in Foxglove (the `/clicked_point` topic).
