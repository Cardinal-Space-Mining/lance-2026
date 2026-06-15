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
    - Add gazebo apt sources (hacky way to install zenohc and zenohcpp as apt packages):
        ```bash
        sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
        ```
    - Install apt packages:
        ```bash
        sudo apt update
        sudo apt install libpcl-dev libopencv-dev python3-netifaces phoenix6 patchelf libzenohc-dev libzenohcpp-dev
        ```

> [!TIP]
> Installing `zenohc` and `zenohcpp` using the gazebo apt repository is a hacky workaround to avoid building and installing locally, which takes a while and requires the Rust toolchain. If this method fails in the future or manual install is deemed beneficial, the included script automatically builds and installs zenoh locally:
> ```bash
> ./src/net-adapter/zenoh_install.sh
> ```

5. Build the project using the included script.
    ```bash
    ./src/build.sh
    ```

> [!TIP]
> Certain nodes are compiled twice with different compile options (for each robot version), resulting in doubled build time. When only targetting one version, flags can be used to limit which versions are built:
> * `./src/build.sh --l1` : Build for LANCE-1 only
> * `./src/build.sh --l2` : Build for LANCE-2 only
> * `./src/build.sh --la` : Explicitly build for both (default behavior)

## Running
The easiest way to run the entire project is to use the provided script. This automatically sources all packages in the workspace and handles any other environment startup/shutdown actions:
```bash
./src/run.sh
```

The following flags may be used to enable different features:
* `--canbus` : Runs `phoenix-driver/scripts/can_bringup.sh` on startup and `phoenix-driver/scripts/can_shutdown.sh` on shutdown. This is required when running CANbus actuators!
* `--local` : Sets the required environment variables so that ROS uses cyclonedds configured for localhost-only discovery. This is necessary to enforce bandwidth reduction strategies.
* `--help` : Displays a usage message.

Following any flag arguments come any number of "launch overrides" which directly interface with the config/launch system and determine what software is run. More info on the underlying system which manages this can be found [here](https://github.com/Cardinal-Space-Mining/launch-utils). There is a vast combination of launch overrides and presets availble for use, but a few simple and common examples are listed below:

1. Run the **robot code** for LANCE-2 in the KSC arena:
    ```bash
    ./src/run.sh --canbus --local robot:=lance2_ksc
    ```
2. Run the **robot code** for LANCE-1 int the UCF left-side arena:
    ```bash
    ./src/run.sh --canbus --local robot:=lance1_ucf_left
    ```
3. Run the **client code** for LANCE-2 in the UCF right-side arena:
    ```bash
    ./src/run.sh --local client:=lance2_ucf_right
    ```
4. Run a Gazebo simulation of LANCE-1 in the UCF left-side arena:
    ```bash
    ./src/run.sh gz_motor_sim:=lance1_ucf_right
    ```
5. Run the **robot code** for LANCE-1 in the UCF right-side arena but explitlcy enable a foxglove server for debugging:
    ```bash
    ./src/run.sh --canbus --local robot:=lance1_ucf_right foxglove_bridge:=all
    ```
6. Run the **client code** for LANCE-2 in the KSC arena but targetting a development mac-mini using it's ethernet interface (assumes standard networking layout which matches that defined in the config):
    ```bash
    ./src/run.sh --local client:=lance2_ksc redux:=client_mac_eth
    ```

> [!TIP]
> All available presets can be found in the `lance/config/presets` directory, and all low-level action configs can be found in the `lance/config/actions` directory.

## Foxglove Studio
A foxglove studio layout configuration (`foxglove_layout.json`) is included which provides a main control dashboard as well as tabs for each perception stage and motor status info. This can be loaded by clicking the **"LAYOUT"** dropdown in the top right corner of foxglove studio, then clicking **"Import from file..."** and navigating to the json.

## Simulation
Simulation assets (Gazebo and Nvidia Isaac) and launch utilities are contained in a separate repo since including them here by default would make the repo quite bloated. Conveniently, the repo just needs to be cloned alongside the other packages to be built and used (see included readme for dependencies!):
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

### Python Formatting
Install the `autopep8` extension and add the following block to `.vscode/settings.json`:
```json
{
    "[python]": {
        "editor.defaultFormatter": "ms-python.autopep8",
        "editor.formatOnSave": false,
    },
    "autopep8.args": [
        "--max-line-length", "80",
        "--ignore", "E302,E303,E402"
    ]
}
```
__*Last updated: 2/20/26*__
