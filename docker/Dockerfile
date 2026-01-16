# Drone Swarm Simulation Platform - Docker Image
# Multi-stage build for ArduPilot SITL + Gazebo Harmonic + ROS2 Jazzy
#
# Build: docker compose build
# Run:   docker compose --profile dev up
#
# Image size: ~15-20GB (ROS2 + Gazebo + ArduPilot are large)

# =============================================================================
# Stage 1: Base image with system dependencies
# =============================================================================
FROM ubuntu:24.04 AS base

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

# Install ROS2 Jazzy Desktop (full) + Gazebo bridge
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-cv-bridge \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update --rosdistro jazzy

# =============================================================================
# Stage 2: Build ArduPilot SITL
# =============================================================================
FROM base AS ardupilot-builder

WORKDIR /build

# Clone ArduPilot
RUN git clone --depth 1 --branch Copter-4.5 https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git submodule update --init --recursive --depth 1

# Install ArduPilot prerequisites (this installs many packages)
RUN cd ardupilot \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduPilot SITL
RUN cd ardupilot \
    && ./waf configure --board sitl \
    && ./waf copter -j$(nproc)

# =============================================================================
# Stage 3: Build ArduPilot Gazebo Plugin
# =============================================================================
FROM base AS gazebo-plugin-builder

WORKDIR /build

# Install plugin build dependencies
RUN apt-get update && apt-get install -y \
    libgz-sim8-dev \
    rapidjson-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
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

# Create non-root user for running simulations
ARG USERNAME=swarm
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install additional runtime dependencies
RUN apt-get update && apt-get install -y \
    libgz-sim8-dev \
    rapidjson-dev \
    libopencv-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    python3-future \
    python3-lxml \
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
RUN pip3 install --break-system-packages --no-cache-dir -r requirements.txt

# Install PyTorch with CUDA support (for YOLO GPU acceleration)
# Falls back to CPU if no GPU available at runtime
RUN pip3 install --break-system-packages --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cu121

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

# Set ownership of app directory
RUN chown -R $USERNAME:$USERNAME /app /opt/ardupilot /opt/ardupilot_gazebo

# Environment variables
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build
ENV GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds:/app/models:/app/worlds
ENV PYTHONPATH=/app:${PYTHONPATH}
ENV PATH=/opt/ardupilot/Tools/autotest:${PATH}
ENV SWARM_PROJECT_ROOT=/app

# Switch to non-root user
USER $USERNAME
WORKDIR /app

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
