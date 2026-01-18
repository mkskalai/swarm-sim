# Drone Swarm Simulation Platform - Docker Image
# Multi-stage build for ArduPilot SITL + Gazebo Harmonic + ROS2 Jazzy
#
# Build: docker compose build
# Run:   docker compose --profile dev up
#
# Build variants:
#   Full:    docker build -t swarm:full .                    (~15-20GB)
#   Minimal: docker build -t swarm:minimal --build-arg MINIMAL=true .  (~8-12GB)
#
# Minimal excludes: PyTorch, YOLO, OpenCV, GStreamer (perception stack)

# =============================================================================
# Stage 1: Base image with system dependencies
# =============================================================================
FROM ubuntu:24.04 AS base

# Build argument for minimal install (excludes perception stack)
ARG MINIMAL=false
ENV MINIMAL=${MINIMAL}

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Set locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install base system packages
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    python3-pip \
    python3-venv \
    python3-dev \
    lsb-release \
    gnupg \
    software-properties-common \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo Harmonic repository
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Add ROS2 Jazzy repository
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Jazzy + Gazebo bridge
# Minimal: ros-base only, Full: desktop with cv-bridge and image transport
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    && if [ "$MINIMAL" = "false" ]; then \
        apt-get install -y \
            ros-jazzy-desktop \
            ros-jazzy-ros-gz-image \
            ros-jazzy-cv-bridge; \
    else \
        apt-get install -y \
            ros-jazzy-ros-base; \
    fi \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update --rosdistro jazzy

# =============================================================================
# Stage 2: Build ArduPilot SITL
# =============================================================================
FROM base AS ardupilot-builder

# Re-declare ARG after FROM (ARGs don't persist across stages)
ARG MINIMAL=false

WORKDIR /build

# Install ArduPilot build dependencies manually
# (The install-prereqs-ubuntu.sh script has Python 2 deps that don't exist on Ubuntu 24.04)
RUN apt-get update && apt-get install -y \
    ccache \
    g++ \
    gawk \
    make \
    valgrind \
    libtool \
    libxml2-dev \
    libxslt1-dev \
    python3-numpy \
    python3-pyparsing \
    python3-psutil \
    python3-matplotlib \
    python3-serial \
    python3-scipy \
    python3-yaml \
    python3-pip \
    && if [ "$MINIMAL" = "false" ]; then \
        apt-get install -y python3-opencv; \
    fi \
    && rm -rf /var/lib/apt/lists/*

# Install Python build dependencies for ArduPilot
RUN pip3 install --break-system-packages --no-cache-dir \
    future \
    lxml \
    pymavlink \
    MAVProxy \
    pexpect \
    empy==3.3.4

# Clone ArduPilot
RUN git clone --depth 1 --branch Copter-4.5 https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git submodule update --init --recursive --depth 1

# Build ArduPilot SITL (waf doesn't like running as root, but we can force it)
RUN cd ardupilot \
    && ./waf configure --board sitl \
    && ./waf copter -j$(nproc)

# =============================================================================
# Stage 3: Build ArduPilot Gazebo Plugin
# =============================================================================
FROM base AS gazebo-plugin-builder

# Re-declare ARG after FROM
ARG MINIMAL=false

WORKDIR /build

# Install plugin build dependencies
# Minimal excludes OpenCV and GStreamer (video streaming)
RUN apt-get update && apt-get install -y \
    libgz-sim8-dev \
    rapidjson-dev \
    && if [ "$MINIMAL" = "false" ]; then \
        apt-get install -y \
            libopencv-dev \
            libgstreamer1.0-dev \
            libgstreamer-plugins-base1.0-dev \
            gstreamer1.0-plugins-bad \
            gstreamer1.0-libav \
            gstreamer1.0-gl; \
    fi \
    && rm -rf /var/lib/apt/lists/*

# Clone and build ardupilot_gazebo
RUN git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git \
    && cd ardupilot_gazebo \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j$(nproc)

# =============================================================================
# Stage 4: Final image
# =============================================================================
FROM base AS final

# Re-declare ARG after FROM
ARG MINIMAL=false
ENV MINIMAL=${MINIMAL}

# Note: Running as root in container for simplicity
# Container isolation provides security boundary

# Install additional runtime dependencies
# Minimal excludes OpenCV and GStreamer
RUN apt-get update && apt-get install -y \
    libgz-sim8-dev \
    rapidjson-dev \
    python3-future \
    python3-lxml \
    && if [ "$MINIMAL" = "false" ]; then \
        apt-get install -y \
            libopencv-dev \
            gstreamer1.0-plugins-bad \
            gstreamer1.0-libav \
            gstreamer1.0-gl; \
    fi \
    && rm -rf /var/lib/apt/lists/*

# Copy ArduPilot from builder stage
COPY --from=ardupilot-builder /build/ardupilot /opt/ardupilot

# Copy ardupilot_gazebo plugin from builder stage
COPY --from=gazebo-plugin-builder /build/ardupilot_gazebo /opt/ardupilot_gazebo

# Setup working directory
WORKDIR /app

# Copy requirements first for better caching
COPY requirements.txt /app/

# Install Python dependencies
# Use --break-system-packages since we're in a container
# Use --ignore-installed to skip Debian-managed packages that can't be uninstalled
RUN pip3 install --break-system-packages --no-cache-dir --ignore-installed -r requirements.txt

# Install ArduPilot Python dependencies (required for sim_vehicle.py)
RUN pip3 install --break-system-packages --no-cache-dir --ignore-installed \
    pymavlink \
    MAVProxy \
    pexpect \
    empy==3.3.4

# Install PyTorch with CUDA support (for YOLO GPU acceleration)
# Only in full build - minimal skips this (~5GB savings)
RUN if [ "$MINIMAL" = "false" ]; then \
        pip3 install --break-system-packages --no-cache-dir --ignore-installed \
            torch torchvision --index-url https://download.pytorch.org/whl/cu121; \
    fi

# Copy project files
COPY swarm/ /app/swarm/
COPY swarm_ros/ /app/swarm_ros/
COPY models/ /app/models/
COPY worlds/ /app/worlds/
COPY scripts/ /app/scripts/
COPY tests/ /app/tests/
COPY pyproject.toml /app/

# Setup ROS2 workspace
RUN mkdir -p /app/ros2_ws/src \
    && ln -s /app/swarm_ros /app/ros2_ws/src/swarm_ros

# Build ROS2 workspace
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash \
    && cd /app/ros2_ws \
    && colcon build --packages-select swarm_ros

# Copy entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Environment variables
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build
ENV GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds:/app/models:/app/worlds
ENV PYTHONPATH=/app:${PYTHONPATH}
ENV PATH=/opt/ardupilot/Tools/autotest:${PATH}
ENV SWARM_PROJECT_ROOT=/app

WORKDIR /app

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
