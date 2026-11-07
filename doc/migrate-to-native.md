# Migrating from the Container to Native (ROS 2 Lyrical)

**Who this is for:** anyone who set up LANCE with the distrobox container (ROS 2 Jazzy). ROS 2 Lyrical and Gazebo now run directly on Ubuntu 26.04, so the native setup is the new standard and the container is no longer needed.

This guide moves your existing setup **out of the container and onto your computer directly** ("natively"). It assumes VS Code, Foxglove, NVIDIA drivers, distrobox, and the `ros-jazzy` container are set up, and the code is cloned and built in `~/code/lance-ws`. You keep your code, your VS Code and Foxglove setup, and your NVIDIA drivers; you only install ROS and the project's dependencies again on your computer and rebuild.

> Setting up a new computer from scratch instead? Use the [main install guide](full-install-guide.md).

When you're done, you will build and run everything from a **normal terminal**, without `distrobox enter`.

> [!NOTE]
> **What changes?** Each Ubuntu version has a matching ROS 2 version, and each ROS 2 version has a matching Gazebo version:
>
> | | Container setup (old) | Native setup (this guide) |
> | - | - | - |
> | Ubuntu | 24.04 (in distrobox) | **26.04** (your computer) |
> | ROS 2 | Jazzy | **Lyrical** |
> | Gazebo | Harmonic | **Jetty** |
>
> The project code supports both, so you don't need to change any code. Gazebo Jetty is a newer simulator release, so if the simulation behaves noticeably differently from what teammates on the container setup see, let the software lead know.

**Everything in this guide runs in a normal 🖥️ HOST terminal.** If your prompt starts with `[BOX:ros-jazzy]`, you're in the container. Type `exit` to leave it first.

---

## Table of Contents

1. [Install ROS 2 Lyrical](#1-install-ros-2-lyrical)
2. [Check That ROS Loads Automatically](#2-check-that-ros-loads-automatically)
3. [Install Git LFS](#3-install-git-lfs)
4. [Install Project Dependencies (and Gazebo)](#4-install-project-dependencies-and-gazebo)
5. [Rebuild the Project](#5-rebuild-the-project)
6. [Run the Simulation](#6-run-the-simulation)
7. [Clean Up the Container (Optional)](#7-clean-up-the-container-optional)
8. [Everyday Workflow Cheat Sheet](#8-everyday-workflow-cheat-sheet)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Install ROS 2 Lyrical

These steps are the same ones you ran inside the container, now run on your computer. They follow the [official ROS 2 Lyrical install guide](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html).

### 1a. Check your locale
ROS needs a UTF-8 "locale" (language setting). A normal Ubuntu desktop install already has one. Check with:
```bash
locale | grep LANG=
```
If the result ends in `UTF-8` (for example `LANG=en_US.UTF-8`), you're good. If not, see the [official guide's locale step](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html).

### 1b. Enable the "universe" software source
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
If it says `Press [ENTER] to continue`, press Enter. It may also say universe is already enabled, which is fine.

### 1c. Add the ROS 2 software source
Copy and paste these lines all together:
```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
This automatically picks the version for Ubuntu 26.04 (`resolute`). The last lines of output should mention `Setting up ros2-apt-source`.

### 1d. Install the ROS development tools
```bash
sudo apt update && sudo apt install ros-dev-tools -y
```

### 1e. Update your system, then install ROS 2 Lyrical
ROS packages are built against an up-to-date Ubuntu, so update first:
```bash
sudo apt upgrade
```
Then install ROS:
```bash
sudo apt install ros-lyrical-ros-base -y
```
This is a big download, so give it a few minutes.

> [!TIP]
> `ros-lyrical-ros-base` matches what we installed in the container and is all the project needs. If you also want ROS GUI tools such as RViz, install `ros-lyrical-desktop` instead (it includes everything in `ros-base`).

---

## 2. Check That ROS Loads Automatically

You don't need to edit `~/.bashrc` again. The snippet you added when setting up the container ([step 4 of the container guide](container-install-guide.md#4-customize-your-terminal-prompt)) already handles the native setup. Its `else` branch loads **ROS 2 Lyrical** whenever you're **not** in the container:

```bash
else
    if [ -f /opt/ros/lyrical/setup.bash ]; then
        source /opt/ros/lyrical/setup.bash
    fi
fi
```

Until now that branch did nothing, because Lyrical wasn't installed. **Close your terminal and open a new one** (or run `source ~/.bashrc`), then check:

```bash
printenv | grep ROS_DISTRO
```
This should print `ROS_DISTRO=lyrical`.

```bash
ros2 pkg list
```
This should print a long list of package names.

> [!NOTE]
> Your prompt is how you tell the two setups apart:
> - `user@computer:~$` means **native**: ROS 2 **Lyrical** is loaded.
> - `[BOX:ros-jazzy] user@computer:~$` means **container**: ROS 2 **Jazzy** is loaded.

---

## 3. Install Git LFS

Git LFS was installed inside the container, not on your computer. Your code is already cloned (it lives in your shared home folder), so you only need the tool itself:
```bash
sudo apt install git-lfs
```
Without it, `git pull` in `csm-sim` would only download small placeholder files instead of the real 3D models.

---

## 4. Install Project Dependencies (and Gazebo)

The container has its own separate copy of Ubuntu, so none of the libraries you installed there exist on your computer yet. These are the same steps as before, now run natively.

Go to the workspace:
```bash
cd ~/code/lance-ws
```

### 4a. Install ROS dependencies with rosdep
`rosdep` installs everything the project's packages need for **Lyrical**, including **Gazebo Jetty** (through `csm-sim`).

First-time setup on this computer:
```bash
sudo rosdep init
rosdep update
```
> If `sudo rosdep init` says the file `already exists`, that's fine. Just continue with `rosdep update`.

Then install everything:
```bash
rosdep install --ignore-src --from-paths src -r -y
```
This downloads a lot of packages, so it can take a while.

> [!IMPORTANT]
> Run `rosdep` in a terminal where `ROS_DISTRO=lyrical` (see [step 2](#2-check-that-ros-loads-automatically)). It uses that variable to decide which ROS version's packages to install.

### 4b. Add extra software sources
**Phoenix 6** (motor controller library):
```bash
YEAR=2026
sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr${YEAR}.list "https://deb.ctr-electronics.com/ctr${YEAR}.list"
```

**OSRF/Gazebo source** (used to get the `zenoh` networking libraries):
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

### 4c. Install the remaining packages
```bash
sudo apt update
sudo apt install libpcl-dev libopencv-dev python3-netifaces phoenix6 patchelf libzenohc-dev libzenohcpp-dev
```

---

## 5. Rebuild the Project

> [!WARNING]
> **You must clean the old build first.** The existing `build/`, `install/`, and `log/` folders were built inside the container against ROS 2 **Jazzy**. Building on top of them with **Lyrical** produces confusing errors.

From `~/code/lance-ws`:
```bash
cd ~/code/lance-ws
./src/build.sh --clean
./src/build.sh
```

`--clean` deletes the `build/`, `install/`, and `log/` folders, and the second command builds from scratch. As before, the first build takes a while (10+ minutes is normal), and `--l1` / `--l2` build for a single robot to save time. When it finishes, you should see `>> Build finished in XXX seconds.`

> [!NOTE]
> The container and the native setup **share the same workspace folder**, because it's in your home folder. If you ever switch back to building inside the container, run `./src/build.sh --clean` first there too. A build only works with the ROS version that created it.

---

## 6. Run the Simulation

In a normal terminal, from `~/code/lance-ws`:
```bash
./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled
```
The Gazebo window should open, now running directly on your computer. Everything else works the same as in the container:
- Leave off `gz_gui:=enabled` to run the simulator headless.
- Other options (`gz_robot`, `gz_client`, other robots and arenas) are listed in [step 10 of the main install guide](full-install-guide.md#10-run-the-simulation) and in the [README](../README.md#running).
- Connect Foxglove to `ws://localhost:8765` exactly as before.
- Press `Ctrl` + `C` in the terminal to stop.

> [!TIP]
> Now that everything runs natively, `foxglove_gui:=local` can open Foxglove for you automatically when the simulation starts:
> ```bash
> ./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled foxglove_gui:=local
> ```

---

## 7. Clean Up the Container (Optional)

Once the native build and simulation work, you can remove the container to free up disk space. **Your code and files in `~` are not affected**, because they live in your home folder, not in the container.

> [!CAUTION]
> Only do this once you're happy with the native setup. Getting the container back means redoing the [container install guide](container-install-guide.md) from step 3.

Delete the container and the downloaded Ubuntu 24.04 image:
```bash
distrobox rm ros-jazzy
podman image rm docker.io/library/ubuntu:24.04
```

If you don't plan to use containers again, you can also uninstall the container tools. The NVIDIA Container Toolkit only lets *containers* use your GPU; native programs use the NVIDIA driver directly, so you don't need it anymore:
```bash
sudo apt remove distrobox podman nvidia-container-toolkit
sudo apt autoremove
```

> [!NOTE]
> Leave the `~/.bashrc` snippet as it is. The container part simply never runs if there's no container, and the native part is what loads ROS 2 Lyrical.

---

## 8. Everyday Workflow Cheat Sheet

```bash
# Open a terminal (no distrobox needed!) and go to the workspace
cd ~/code/lance-ws

# Pull the latest code (optional)
cd src && git pull && git submodule update --init --recursive && cd ..

# Build after any code change
./src/build.sh

# Run
./src/run.sh gz_full:=lance2_ksc gz_gui:=enabled
```

- **Edit code** in VS Code (`code ~/code/lance-ws`).
- **Build, run, and view data** all happen on your computer. Every new terminal tab is ready to use ROS right away.

---

## 9. Troubleshooting

<details>
<summary><b><code>ros2: command not found</code>, or <code>ROS_DISTRO</code> isn't <code>lyrical</code></b></summary>

- Open a **new** terminal (or run `source ~/.bashrc`).
- Make sure you're **not** in the container: your prompt should **not** start with `[BOX:ros-jazzy]`.
- Check that `/opt/ros/lyrical/setup.bash` exists. If it doesn't, ROS didn't install. Redo [step 1e](#1e-update-your-system-then-install-ros-2-lyrical).
- Check that the snippet from [step 4 of the container guide](container-install-guide.md#4-customize-your-terminal-prompt) is at the bottom of `~/.bashrc`. If it's missing, add this line instead:
  ```bash
  echo '[ -f /opt/ros/lyrical/setup.bash ] && source /opt/ros/lyrical/setup.bash' >> ~/.bashrc
  ```
</details>

<details>
<summary><b>The build fails with errors mentioning <code>/opt/ros/jazzy</code></b></summary>

Old build files from the container are still around. Clean and rebuild:
```bash
cd ~/code/lance-ws
./src/build.sh --clean
./src/build.sh
```
</details>

<details>
<summary><b>The build fails with "package not found" / missing dependency errors</b></summary>

Re-run the dependency steps from `~/code/lance-ws`, in a terminal where `ROS_DISTRO=lyrical`:
```bash
rosdep update
rosdep install --ignore-src --from-paths src -r -y
```
Also double-check that the apt packages from [step 4c](#4c-install-the-remaining-packages) installed without errors.
</details>

<details>
<summary><b>Gazebo models look broken or missing / errors about invalid mesh files</b></summary>

Make sure Git LFS is installed ([step 3](#3-install-git-lfs)), then from inside `~/code/lance-ws/src/csm-sim`:
```bash
git lfs install
git lfs fetch
git lfs checkout
```
Then rebuild.
</details>

<details>
<summary><b>Gazebo is very slow</b></summary>

Run `nvidia-smi` and check that your GPU shows up. If it doesn't, revisit [step 4 of the main install guide](full-install-guide.md#4-install-nvidia-drivers-nvidia-gpus-only). You don't need the container-only steps (the NVIDIA Container Toolkit and CDI spec) for the native setup.
</details>

<details>
<summary><b>I want to go back to the container setup</b></summary>

As long as you haven't deleted the container ([step 7](#7-clean-up-the-container-optional)), you can switch back at any time:
```bash
distrobox enter ros-jazzy
cd ~/code/lance-ws
./src/build.sh --clean
./src/build.sh
```
Remember to `--clean` again when you switch back to native.
</details>

---

*Written for Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Jetty, natively. Last updated: 10/26/26*
