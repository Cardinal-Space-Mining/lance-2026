# Running LANCE on Windows (WSL)

This guide covers what's **different** when you run LANCE inside **WSL 2** (Windows Subsystem for Linux) instead of on a real Ubuntu install. Follow the [full install guide](full-install-guide.md) for the actual setup, and use this page alongside it for the WSL-specific steps and fixes.

> [!IMPORTANT]
> The most important part is [section 3](#3-make-gazebo-use-your-gpu). Without it, Gazebo runs on your **CPU** instead of your graphics card and is extremely slow, even though everything looks correctly installed.

---

## Table of Contents

1. [Install WSL and Ubuntu 26.04](#1-install-wsl-and-ubuntu-2604)
2. [Changes to the Full Install Guide](#2-changes-to-the-full-install-guide)
3. [Make Gazebo Use Your GPU](#3-make-gazebo-use-your-gpu)
4. [Give WSL Enough Memory](#4-give-wsl-enough-memory)
5. [Everyday Tips](#5-everyday-tips)
6. [Hardware: USB and CAN](#6-hardware-usb-and-can)
7. [Upgrading an Existing Ubuntu 24.04 WSL Install](#7-upgrading-an-existing-ubuntu-2404-wsl-install)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Install WSL and Ubuntu 26.04

Run these in **PowerShell on Windows** (not inside Ubuntu).

Update WSL itself first. Older versions don't support the graphics features Gazebo needs:
```powershell
wsl --update
wsl --version
```
`wsl --version` should report **WSL version 2.x** and a **WSLg version**. WSLg is the part that lets Linux windows (like Gazebo) show up on your Windows desktop.

Install Ubuntu 26.04:
```powershell
wsl --install -d Ubuntu-26.04
```
When it finishes, it asks you to create a Linux username and password. These are separate from your Windows login. Remember the password, because `sudo` asks for it.

> [!TIP]
> Already have another Ubuntu in WSL? You can keep it. `wsl -l -v` lists your installs, and `wsl -d Ubuntu-26.04` opens a specific one. Installing a new one side by side is safer than upgrading in place (see [section 7](#7-upgrading-an-existing-ubuntu-2404-wsl-install)).

---

## 2. Changes to the Full Install Guide

Follow the [full install guide](full-install-guide.md) inside your Ubuntu terminal, with these changes:

- **Step 0 (Linux basics):** You open Ubuntu from the Start menu or Windows Terminal instead of with `Ctrl` + `Alt` + `T`. Paste with **right-click** or `Ctrl` + `Shift` + `V`.
- **Step 2 (VS Code):** Install VS Code on **Windows**, not inside Ubuntu, and add the **WSL** extension. Then run `code ~/code/lance-ws` from the Ubuntu terminal to open the project. VS Code runs on Windows but edits and builds inside Ubuntu.
- **Step 3 (Foxglove):** You can install Foxglove on Windows instead. It connects to `ws://localhost:8765` the same way, because WSL forwards `localhost` to Windows.
- **Step 4 (NVIDIA drivers): skip it entirely.** Your normal **Windows** NVIDIA driver already gives WSL access to the GPU. **Do not install any NVIDIA driver inside Ubuntu.** It replaces the special WSL graphics libraries and breaks GPU support. To check that the GPU is visible, run `nvidia-smi` inside Ubuntu; your graphics card should show up.
- **After step 6:** Do the GPU fix in [section 3](#3-make-gazebo-use-your-gpu) below.

---

## 3. Make Gazebo Use Your GPU

### The problem
WSL gives Linux access to your graphics card through a translation layer (Linux OpenGL → Windows Direct3D 12). The graphics library Ubuntu uses (**Mesa**) includes a driver for it, called `d3d12`, but **doesn't choose it automatically**. It falls back to `llvmpipe`, which draws everything on the **CPU**. Gazebo still opens, but it crawls and the CPU sits near 100%.

This happens on both Ubuntu 24.04 (Mesa 25) and 26.04 (Mesa 26).

### Check whether you're affected
```bash
sudo apt install mesa-utils
glxinfo -B | grep -E "Device|Accelerated"
```
- `Device: llvmpipe ...` and `Accelerated: no` mean **you need the fix**.
- `Device: D3D12 (<your GPU name>)` and `Accelerated: yes` mean you're already set.

### The fix
Open `~/.bashrc`:
```bash
code ~/.bashrc
```
Add this at the **bottom**, after the line that sources ROS:
```bash
# WSL: Mesa defaults to llvmpipe (CPU rendering); force the D3D12 GPU driver
if grep -qi microsoft /proc/version 2>/dev/null; then
    export GALLIUM_DRIVER=d3d12
fi
```
The `if` checks that you're running inside WSL, so the same `~/.bashrc` is safe to copy to a real Linux machine.

Save, **open a new terminal**, and check again:
```bash
glxinfo -B | grep -E "Device|Accelerated"
```
It should now show your GPU, e.g. `Device: D3D12 (NVIDIA GeForce RTX 3070)` and `Accelerated: yes`.

### Confirm Gazebo uses it
```bash
gz sim shapes.sdf
```
A Gazebo window with a few shapes should open and rotate smoothly. Close it, then check which renderer it used:
```bash
grep GL_RENDERER ~/.gz/rendering/ogre2.log | tail -1
```
It should name your GPU (`D3D12 (...)`), not `llvmpipe`.

> [!NOTE]
> The fix also covers **headless** simulation (no Gazebo window). GPU sensors like the simulated lidar render on the graphics card either way.

---

## 4. Give WSL Enough Memory

By default, WSL can use only **half your computer's RAM**. Building LANCE uses a lot of memory, and if WSL runs out, the build freezes or gets killed. To raise the limit, create (or edit) the file `C:\Users\<your Windows username>\.wslconfig` on **Windows**:
```ini
[wsl2]
memory=12GB
swap=8GB
```
Choose a `memory` value a few GB below your total RAM, so Windows keeps enough for itself. Then restart WSL from PowerShell for the change to take effect:
```powershell
wsl --shutdown
```
Open Ubuntu again. Run `free -h` to see the new limit.

Building one robot version at a time (`./src/build.sh --l2`) also helps; see the full guide's troubleshooting section.

---

## 5. Everyday Tips

- **Keep the code in your Linux home folder** (`~/code/lance-ws`), **not** under `/mnt/c/...`. Files on the Windows drive are many times slower to access from Linux, which makes builds painfully slow.
- **Opening Linux files from Windows:** In File Explorer, type `\\wsl$` in the address bar, or run `explorer.exe .` in an Ubuntu terminal to open the current folder.
- **Restarting WSL:** If something gets stuck (Gazebo windows not appearing, networking broken), run `wsl --shutdown` in PowerShell and reopen Ubuntu. This is the WSL equivalent of rebooting.
- **Wrong clock after sleep:** After your laptop sleeps, WSL's clock can lag behind. That breaks `apt update` ("Release file is not valid yet") and confuses ROS timestamps. Fix it with:
  ```bash
  sudo hwclock -s
  ```
  or with `wsl --shutdown`.
- **Gazebo windows:** They appear like normal Windows apps, with Linux-style title bars. If none appear at all, check that `echo $DISPLAY` prints something (usually `:0`). If it's empty, run `wsl --update` and `wsl --shutdown`.
- **Renaming a WSL install:** WSL has no rename command. With WSL shut down (`wsl --shutdown`), this PowerShell snippet changes the name. Replace the two names:
  ```powershell
  Get-ChildItem HKCU:\Software\Microsoft\Windows\CurrentVersion\Lxss |
    Where-Object { (Get-ItemProperty $_.PSPath).DistributionName -eq 'OLD-NAME' } |
    Set-ItemProperty -Name DistributionName -Value 'NEW-NAME'
  ```
  Afterwards, reconnect VS Code to the new name ("WSL: Connect to WSL using Distro…"). Windows Terminal and Start menu shortcuts keep the old label until you rename them yourself.

---

## 6. Hardware: USB and CAN

The **simulation works fully in WSL**. Connecting **real hardware** is more limited.

### USB devices
WSL can't see USB devices plugged into Windows by default. Use [usbipd-win](https://github.com/dorssel/usbipd-win) to pass a device through. In **PowerShell as Administrator**:
```powershell
winget install usbipd       # one time
usbipd list                 # find the device's BUSID, e.g. 2-3
usbipd bind --busid 2-3     # one time per device
usbipd attach --wsl --busid 2-3
```
The device then shows up in Ubuntu (check with `lsusb`). Re-run `attach` after unplugging the device or restarting WSL.

### CAN (`./src/run.sh --canbus`)
> [!WARNING]
> **CAN bringup does not work on the stock WSL kernel.** `phoenix-driver/scripts/can_bringup.sh` uses `slcand` to talk to the CANable adapters, which needs the Linux `slcan` kernel module. Microsoft's WSL kernel doesn't include it. You can pass the CANable through with usbipd, but `slcand` will fail to create the CAN interface.

To drive real CAN actuators, use a native Ubuntu install, or build a custom WSL kernel with `CONFIG_CAN_SLCAN` enabled (advanced, not covered here). You can check whether a kernel supports it with:
```bash
zcat /proc/config.gz | grep CAN_SLCAN
```
`CONFIG_CAN_SLCAN=m` or `=y` means it's supported; `is not set` means it isn't.

### Networking to the robot
WSL normally sits behind its own private network, so ROS on WSL may not discover the robot on your Wi-Fi/LAN. On Windows 11, **mirrored networking** puts WSL directly on your computer's network. Add to `.wslconfig`:
```ini
[wsl2]
networkingMode=mirrored
```
Then run `wsl --shutdown`. This doesn't affect the simulation, which runs entirely on `localhost`.

---

## 7. Upgrading an Existing Ubuntu 24.04 WSL Install

**Recommended:** install `Ubuntu-26.04` as a new install ([section 1](#1-install-wsl-and-ubuntu-2604)), set it up from scratch, and delete the old one once you're happy (`wsl --unregister Ubuntu-24.04`). **This permanently deletes everything in it**, so copy off anything you need first.

If you'd rather upgrade in place, **back up first** in PowerShell:
```powershell
wsl --export Ubuntu-24.04 D:\ubuntu2404-backup.tar
```
Then, inside Ubuntu, remove everything built for 24.04 before upgrading. Check what each `purge` would remove by running it with `-s` first:
```bash
# ROS 2 Jazzy, Gazebo Harmonic, and zenoh (all built only for 24.04)
sudo apt purge 'ros-jazzy-*' ros-dev-tools
sudo apt purge gz-harmonic 'gz-*' 'libgz-*' 'python3-gz-*' 'libsdformat*' 'sdformat*' 'python3-sdformat*'
sudo apt purge libzenohc-dev libzenohcpp-dev libzenohc
sudo apt autoremove --purge
sudo rm -rf /opt/ros/jazzy

# Package sources hard-coded to 24.04 ("noble")
sudo rm /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/gazebo-stable.list

# Stop sourcing Jazzy, and clear the old build
sed -i '\|source /opt/ros/jazzy/setup.bash|d' ~/.bashrc
rm -rf ~/code/lance-ws/build ~/code/lance-ws/install ~/code/lance-ws/log

# Upgrade
sudo apt update && sudo apt full-upgrade
sudo do-release-upgrade
```
After the upgrade:
- Turn the remaining third-party sources (phoenix6, Foxglove, Node.js) back on in `/etc/apt/sources.list.d/`. The upgrade disables them.
- Follow the [full install guide](full-install-guide.md) from step 5 to install Lyrical, Gazebo Jetty, and the project.
- Re-check the [GPU fix](#3-make-gazebo-use-your-gpu).
- The install keeps its old name (`Ubuntu-24.04`). See [renaming](#5-everyday-tips) if that bothers you.

---

## 8. Troubleshooting

<details>
<summary><b>Gazebo is very slow / CPU at 100% while the simulation runs</b></summary>

Gazebo is almost certainly rendering on the CPU. Run `glxinfo -B | grep Device`. If it says `llvmpipe`, apply the [GPU fix](#3-make-gazebo-use-your-gpu) and open a **new** terminal. If it still says `llvmpipe`:
- Check that `/dev/dxg` exists (`ls /dev/dxg`). If it doesn't, WSL has no GPU access: update your **Windows** NVIDIA driver, run `wsl --update`, then `wsl --shutdown`.
- Make sure you never installed a Linux NVIDIA driver inside Ubuntu (`dpkg -l | grep nvidia-driver` should print nothing). If you did, remove it with `sudo apt purge 'nvidia-*'` and run `wsl --shutdown`.
</details>

<details>
<summary><b><code>nvidia-smi: command not found</code> inside Ubuntu</b></summary>

WSL provides `nvidia-smi` from Windows at `/usr/lib/wsl/lib/nvidia-smi`. If it's missing, update your Windows NVIDIA driver and run `wsl --shutdown`. Don't install `nvidia-utils` packages inside Ubuntu.
</details>

<details>
<summary><b>Gazebo crashes or shows a black/garbled window with the GPU fix on</b></summary>

This sometimes happens after a Windows graphics driver update. Try these in order:
1. Update the Windows NVIDIA driver and run `wsl --update`.
2. Use Gazebo's older renderer: `gz sim --render-engine ogre shapes.sdf`.
3. Last resort, CPU rendering (slow, but it works): `LIBGL_ALWAYS_SOFTWARE=1 gz sim shapes.sdf`.

You can also run the simulation headless (without `gz_gui:=enabled`) and use Foxglove only.
</details>

<details>
<summary><b>Build freezes or gets "Killed"</b></summary>

WSL ran out of memory. Raise the limit in `.wslconfig` ([section 4](#4-give-wsl-enough-memory)) and build one version at a time with `./src/build.sh --l2`.
</details>

<details>
<summary><b><code>apt update</code> says "Release file ... is not valid yet"</b></summary>

WSL's clock fell behind after sleep. Run `sudo hwclock -s` (or `wsl --shutdown` from PowerShell) and try again.
</details>

---

*Written for Windows 11 + WSL 2.6 + Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Jetty. Last updated: 10/26/26*
