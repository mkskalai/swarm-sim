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
echo "Quick start commands (use 3 terminals):"
echo ""
echo "  # Terminal 1: Start Gazebo"
echo "  cd ~/ardupilot_gazebo && gz sim -r ~/swarm/worlds/single_drone.sdf"
echo ""
echo "  # Terminal 2: Start SITL (JSON frame + MAVSDK port)"
echo "  cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f JSON --console --out=udp:127.0.0.1:14540"
echo ""
echo "  # Terminal 3: Run test"
echo "  source ~/venv-ardupilot/bin/activate && cd ~/swarm && python scripts/test_connection.py"
echo ""
