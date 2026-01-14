"""
Launch file for simulation mode.

Starts SwarmBridge that connects to running SITL instances and
exposes them via ROS2 topics/services.

Prerequisites:
    1. Start Gazebo + SITL instances:
       python scripts/run_phase3_test.py --num-drones 3 --skip-test

    2. Then launch this:
       ros2 launch swarm_ros simulation.launch.py num_drones:=3

Topics:
    /swarm/status - Aggregated swarm status
    /drone_N/state - Per-drone state

Services:
    /swarm/connect - Connect to drones
    /swarm/set_formation - Change formation
    /swarm/takeoff_all, /swarm/land_all, etc.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

# Find the swarm project root (relative to this launch file's installed location)
# The swarm package lives at ~/Desktop/python projects/claude/swarm/swarm
# We need to add the parent directory to PYTHONPATH
SWARM_PROJECT_ROOT = str(Path.home() / "Desktop" / "python projects" / "claude" / "swarm")


def generate_launch_description():
    # Declare arguments
    num_drones_arg = DeclareLaunchArgument(
        "num_drones",
        default_value="3",
        description="Number of drones in simulation",
    )

    base_port_arg = DeclareLaunchArgument(
        "base_port",
        default_value="14540",
        description="Base UDP port for drone connections",
    )

    auto_connect_arg = DeclareLaunchArgument(
        "auto_connect",
        default_value="true",
        description="Automatically connect to drones on startup",
    )

    # Add swarm project to PYTHONPATH so we can import swarm.coordination, etc.
    current_pythonpath = os.environ.get("PYTHONPATH", "")
    new_pythonpath = f"{SWARM_PROJECT_ROOT}:{current_pythonpath}" if current_pythonpath else SWARM_PROJECT_ROOT

    set_pythonpath = SetEnvironmentVariable(
        name="PYTHONPATH",
        value=new_pythonpath,
    )

    # SwarmBridge node
    swarm_bridge_node = Node(
        package="swarm_ros",
        executable="swarm_bridge",
        name="swarm_bridge",
        output="screen",
        parameters=[
            {
                "num_drones": LaunchConfiguration("num_drones"),
                "base_port": LaunchConfiguration("base_port"),
                "auto_connect": LaunchConfiguration("auto_connect"),
            }
        ],
    )

    return LaunchDescription(
        [
            num_drones_arg,
            base_port_arg,
            auto_connect_arg,
            set_pythonpath,
            swarm_bridge_node,
        ]
    )
