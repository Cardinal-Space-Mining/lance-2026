#!/bin/bash

set -e

# --- Configuration ---
ZENOH_VERSION="1.7.2"  # Release version to install
ZENOH_C_REPO="https://github.com/eclipse-zenoh/zenoh-c.git"
ZENOH_C_DIR="$HOME/zenoh-c"
ZENOH_CPP_REPO="https://github.com/eclipse-zenoh/zenoh-cpp.git"
ZENOH_CPP_DIR="$HOME/zenoh-cpp"

# --- Helper Functions ---
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

echo "Checking prerequisites..."

# 1. Git
if ! command_exists git; then
    echo "Git is not installed. Please install Git and rerun this script."
    exit 1
fi

# 2. CMake
if ! command_exists cmake; then
    echo "CMake is not installed. Please install CMake and rerun this script."
    exit 1
fi

# 3. Make
if ! command_exists make; then
    echo "Make is not installed. Please install Make and rerun this script."
    exit 1
fi

# 4. GCC/G++
if ! command_exists gcc || ! command_exists g++; then
    echo "GCC/G++ is not installed. Please install them and rerun this script."
    exit 1
fi

# 5. Rust (cargo)
if ! command_exists cargo; then
    if [ -f "$HOME/.cargo/env" ]; then
        echo "Sourcing Rust environment..."
        source "$HOME/.cargo/env"
    fi
fi

if ! command_exists cargo; then
    echo "Rust (cargo) not found. Installing Rust via rustup..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
fi

# --- Function to build Zenoh library ---
build_zenoh() {
    local dir=$1
    local name=$2

    cd "$dir"
    # Check if the current tag matches ZENOH_VERSION
    current_tag=$(git describe --tags --exact-match 2>/dev/null || echo "")
    if [ "$current_tag" == "$ZENOH_VERSION" ]; then
        echo "$name version $ZENOH_VERSION already checked out."
    else
        echo "Checking out $name tag $ZENOH_VERSION..."
        git fetch --tags
        git checkout "$ZENOH_VERSION"
    fi

    # Build only if build/ doesn't exist or Makefile is missing
    if [ ! -d build ] || [ ! -f build/Makefile ]; then
        echo "Building $name..."
        mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release
        make -j$(nproc)
        sudo make install
        sudo ldconfig
    else
        echo "$name build directory exists. Skipping rebuild."
    fi
}

# --- Install Zenoh C ---
if [ ! -d "$ZENOH_C_DIR" ]; then
    echo "Cloning Zenoh C..."
    git clone $ZENOH_C_REPO $ZENOH_C_DIR
fi
build_zenoh "$ZENOH_C_DIR" "Zenoh C"

# --- Install Zenoh C++ ---
if [ ! -d "$ZENOH_CPP_DIR" ]; then
    echo "Cloning Zenoh C++..."
    git clone $ZENOH_CPP_REPO $ZENOH_CPP_DIR
fi
build_zenoh "$ZENOH_CPP_DIR" "Zenoh C++"

echo "Zenoh C and C++ version $ZENOH_VERSION installed and up-to-date!"
