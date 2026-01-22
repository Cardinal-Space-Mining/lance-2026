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
RUN sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg" && \
    YEAR=2026 \
    sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr${YEAR}.list "https://deb.ctr-electronics.com/ctr${YEAR}.list"

# Install additional stuff
RUN sudo apt update && \
    sudo apt install \
        python3-netifaces \
        phoenix6 \
        patchelf

# Default command
CMD ["bash"]
