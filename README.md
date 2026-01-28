## LANCE-2026
This repo houses all client and robot code used for running LANCE-2 (2026) as well as backwards compatibility for LANCE-1. If setting up from scratch, you will want to read this entire document.

## Setup
1. Install [ROS2](https://docs.ros.org/en/jazzy/Installation.html) if not already done (we are using Jazzy for 2025-2026).

2. Setup your workspace and clone this repo.
    - Create directory:
        ```bash
        mkdir lance-ws && cd lance-ws
        ```
    - Clone this repo into the `src` directory:
        ```bash
        git clone --recurse-submodules -b main https://github.com/Cardinal-Space-Mining/lance-2026 src
        ```
    - If you previously cloned and forgot to clone submodules, or need to update them:
        ```bash
        git submodule update --init --recursive
        ```

3. Use rosdep to install dependencies.
    - Initialize rosdep if not already done:
        ```bash
        sudo rosdep init
        ```
    - Update and install:
        ```bash
        rosdep update
        rosdep install --ignore-src --from-paths . -r -y
        ```

4. Install apt dependencies.
    - Add phoenix6 apt sources:
        ```bash
        YEAR=2026
        sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
        sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr${YEAR}.list "https://deb.ctr-electronics.com/ctr${YEAR}.list"
        ```
    - Install apt packages:
        ```bash
        sudo apt update
        sudo apt install libpcl-dev libopencv-dev python3-netifaces phoenix6 patchelf
        ```

5. Install `zenoh-cpp` using the provided script (installs Rust, then clones, builds and installs):
    ```bash
    ./src/zenoh_install.sh
    ```

5. Build the project using the included script.
    ```bash
    ./src/build.sh
    ```

## Running
The easiest way to run the entire project is to use the provided script. This automatically sources all packages in the workspace and handles setting up/tearing down a canbus when required. To run without canbus support:
```bash
./src/run.sh
```

And to enable canbus support:
```bash
./src/run.sh --canbus
```

Reconfiguration for the various deployment contexts is accomplished with the help of [launch-utils](https://github.com/Cardinal-Space-Mining/launch-utils) and config presets. For example, to run the full project attached to a gazebo simulation, run:
```bash
./src/run.sh preset:=gz_dev
```
Presets for the robot and mission control have yet to be defined, so more info on these will follow later.

## Foxglove Studio
A foxglove studio layout configuration (`foxglove_layout.json`) is included which provides a main control dashboard as well as tabs for each perception stage and motor status info. This can be loaded by clicking the **"LAYOUT"** dropdown in the top right corner of foxglove studio, then clicking **"Import from file..."** and navigating to the json.

## Simulation
Simulation assets (Gazebo and Nvidia Isaac) and launch utilities are encapsulated in a separate repo since including them here by default would make the repo quite bloated. Conveniently, the repo just needs to be cloned alongside the other packages to be built and used (see included readme for dependencies!):
```bash
pushd src && git clone https://gitlab.com/csm2.0/csm-sim && popd
```

## VSCode
The build script exports compile commands which can help VSCode's C/C++ extension resolve correct syntax highlighting. To ensure this is working, paste the following code into the `c_cpp_properties.json` file (under .vscode directory in a workspace):
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "intelliSenseMode": "linux-gcc-x64",
            "cStandard": "c17",
            "cppStandard": "c++20",
            "compileCommands": [
                "build/compile_commands.json"
            ]
        }
    ],
    "version": 4
}
```
__*Last updated: 1/21/26*__
