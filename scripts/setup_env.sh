#!/bin/bash
# Environment setup script for drone swarm simulation
#
# Usage:
#   source scripts/setup_env.sh
#
# This script sets up the environment variables needed for:
# - Gazebo Harmonic (gz-sim)
# - ArduPilot SITL
# - ardupilot_gazebo plugin
# - ROS2 Jazzy (optional)

set -e

echo "=== Drone Swarm Environment Setup ==="

# Clean snap library pollution (VSCode snap injects conflicting glibc)
if [[ -n "$VSCODE_CLI" ]] || [[ "$LD_LIBRARY_PATH" == */snap/* ]]; then
    echo "Cleaning snap library paths from environment..."
    # Remove any snap paths from LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/snap/' | tr '\n' ':' | sed 's/:$//')
    # Prevent snap library preloading
    unset LD_PRELOAD 2>/dev/null || true
fi

# Detect ardupilot_gazebo location
ARDUPILOT_GAZEBO_PATH="$HOME/ardupilot_gazebo"
if [ ! -d "$ARDUPILOT_GAZEBO_PATH" ]; then
    echo "Warning: ardupilot_gazebo not found at $ARDUPILOT_GAZEBO_PATH"
    echo "Please set ARDUPILOT_GAZEBO_PATH to the correct location"
fi

# Gazebo plugin and resource paths
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO_PATH}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="${ARDUPILOT_GAZEBO_PATH}/models:${ARDUPILOT_GAZEBO_PATH}/worlds:${GZ_SIM_RESOURCE_PATH}"

# Add project worlds and models to Gazebo path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
export GZ_SIM_RESOURCE_PATH="${PROJECT_ROOT}/worlds:${PROJECT_ROOT}/models:${GZ_SIM_RESOURCE_PATH}"

# Python virtual environment
VENV_PATH="$HOME/venv-ardupilot"
if [ -d "$VENV_PATH" ]; then
    echo "Activating Python venv: $VENV_PATH"
    source "$VENV_PATH/bin/activate"
else
    echo "Warning: Python venv not found at $VENV_PATH"
fi

# ROS2 Jazzy (optional - comment out if not using ROS2)
ROS2_SETUP="/opt/ros/jazzy/setup.bash"
if [ -f "$ROS2_SETUP" ]; then
    echo "Sourcing ROS2 Jazzy..."
    source "$ROS2_SETUP"

    # Source ROS2 workspace if it exists
    if [ -f ~/ros2_ws/install/setup.bash ]; then
        echo "Sourcing ROS2 workspace..."
        source ~/ros2_ws/install/setup.bash
    fi
else
    echo "Note: ROS2 Jazzy not found (optional)"
fi

# Hybrid GPU (Intel/NVIDIA) rendering fix
# Gazebo's OGRE2 renderer needs to use the NVIDIA GPU for proper EGL support
if command -v nvidia-smi &> /dev/null && [ -f "/usr/share/glvnd/egl_vendor.d/10_nvidia.json" ]; then
    echo "NVIDIA GPU detected - configuring for hybrid GPU rendering..."
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
    export __NV_PRIME_RENDER_OFFLOAD=1
fi

# Add project to Python path
export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH}"

echo ""
echo "Environment configured!"
echo "  GZ_SIM_SYSTEM_PLUGIN_PATH: ${GZ_SIM_SYSTEM_PLUGIN_PATH}"
echo "  Project root: ${PROJECT_ROOT}"
echo ""
echo "NOTE: If project path has spaces, use symlink: ln -s \"${PROJECT_ROOT}\" ~/swarm"
echo ""
echo "Quick start commands:"
echo ""
echo "  # Multi-drone simulation (Terminal 1):"
echo "  python scripts/run_phase3_test.py --num-drones 3 --skip-test"
echo ""
echo "  # ROS2 bridge (Terminal 2, after simulation ready):"
echo "  ros2 launch swarm_ros simulation.launch.py num_drones:=3"
echo ""
echo "  # Monitor (Terminal 3):"
echo "  ros2 topic echo /swarm/status"
echo ""
echo "  # Fly formation (Terminal 4):"
echo "  ros2 service call /swarm/connect std_srvs/srv/Trigger"
echo "  ros2 service call /swarm/takeoff_all std_srvs/srv/Trigger"
echo "  ros2 service call /swarm/set_formation swarm_ros/srv/SetFormation '{formation_type: 1}'"
echo ""
