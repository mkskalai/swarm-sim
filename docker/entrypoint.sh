#!/bin/bash
# Entrypoint script for Drone Swarm Simulation container
#
# Sets up environment variables and sources ROS2 before executing commands

set -e

# =============================================================================
# Environment Setup
# =============================================================================

# Gazebo plugin and resource paths
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds:/app/models:/app/worlds:${GZ_SIM_RESOURCE_PATH}"

# Python path for swarm module
export PYTHONPATH="/app:${PYTHONPATH}"
export SWARM_PROJECT_ROOT="/app"

# ArduPilot tools path
export PATH="/opt/ardupilot/Tools/autotest:${PATH}"

# =============================================================================
# ROS2 Setup
# =============================================================================

# Source ROS2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source ROS2 workspace if built
if [ -f /app/ros2_ws/install/setup.bash ]; then
    source /app/ros2_ws/install/setup.bash
fi

# =============================================================================
# GPU Detection
# =============================================================================

if command -v nvidia-smi &> /dev/null; then
    echo "[Docker] NVIDIA GPU detected"
    nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null || true

    # Configure for NVIDIA rendering
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
    export __NV_PRIME_RENDER_OFFLOAD=1
else
    echo "[Docker] No NVIDIA GPU detected - using CPU rendering"
    echo "[Docker] YOLO inference will use CPU (slower but functional)"
fi

# =============================================================================
# Display Setup
# =============================================================================

if [ -n "$DISPLAY" ]; then
    echo "[Docker] Display: $DISPLAY"

    # Fix X11 permissions if needed
    if [ -f /home/swarm/.Xauthority ]; then
        export XAUTHORITY=/home/swarm/.Xauthority
    fi
else
    echo "[Docker] No display - running headless"
    export QT_QPA_PLATFORM=offscreen
fi

# =============================================================================
# Create logs directory
# =============================================================================

mkdir -p /app/logs

# =============================================================================
# Execute command
# =============================================================================

echo "[Docker] Environment ready"
echo "[Docker] GZ_SIM_RESOURCE_PATH includes:"
echo "         - /opt/ardupilot_gazebo/models"
echo "         - /app/models"
echo "         - /app/worlds"
echo ""

exec "$@"
