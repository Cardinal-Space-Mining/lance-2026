# Base image with minimal ROS 2
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-perception

# Install essential build tools + rosdep
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    curl \
    gnupg \
    lsb-release

# Set working directory
WORKDIR /workspace

# Copy workspace in
COPY . ./src

# Install dependencies
RUN rosdep install --from-paths . --ignore-src -r -y

# Add pheonix6
RUN curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg" && \
    curl -s --compressed -o /etc/apt/sources.list.d/ctr2026.list "https://deb.ctr-electronics.com/ctr2026.list" && \
    curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install additional stuff
RUN apt-get update && apt-get install -y \
    python3-netifaces \
    phoenix6 \
    patchelf \
    libzenohc-dev \
    libzenohcpp-dev \

# Default command
CMD ["bash"]
