"""
Launch perception nodes for drone swarm.

Launches PerceptionNode for each drone and optionally the ros_gz_bridge
for camera topics.

Usage:
    ros2 launch swarm_ros perception.launch.py num_drones:=3
    ros2 launch swarm_ros perception.launch.py num_drones:=6 use_gpu:=true
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Path to main swarm package for PYTHONPATH
SWARM_PROJECT_ROOT = str(
    Path.home() / "Desktop" / "python projects" / "claude" / "swarm"
)


def generate_launch_description():
    """Generate launch description for perception nodes."""

    # Declare launch arguments
    num_drones_arg = DeclareLaunchArgument(
        "num_drones",
        default_value="3",
        description="Number of drones in the swarm",
    )

    model_arg = DeclareLaunchArgument(
        "model_name",
        default_value="yolo11n.pt",
        description="YOLO model to use (yolo11n.pt, yolo11s.pt, yolo11m.pt)",
    )

    use_gpu_arg = DeclareLaunchArgument(
        "use_gpu",
        default_value="true",
        description="Use GPU for inference if available",
    )

    confidence_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.4",
        description="Detection confidence threshold (0.0-1.0)",
    )

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="perception_test_world",
        description="Gazebo world name for camera topic paths",
    )

    # Set PYTHONPATH to include main swarm package
    current_pythonpath = os.environ.get("PYTHONPATH", "")
    new_pythonpath = (
        f"{SWARM_PROJECT_ROOT}:{current_pythonpath}"
        if current_pythonpath
        else SWARM_PROJECT_ROOT
    )
    set_pythonpath = SetEnvironmentVariable(name="PYTHONPATH", value=new_pythonpath)

    # Create perception nodes for each drone
    # Note: We create nodes for up to 6 drones, each conditionally enabled
    perception_nodes = []
    for i in range(6):
        node = Node(
            package="swarm_ros",
            executable="perception_node",
            name=f"perception_{i}",
            output="screen",
            parameters=[
                {
                    "drone_id": i,
                    "model_name": LaunchConfiguration("model_name"),
                    "use_gpu": LaunchConfiguration("use_gpu"),
                    "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                    "publish_rate_limit": 10.0,
                    "world_name": LaunchConfiguration("world_name"),
                }
            ],
            # Only launch if drone_id < num_drones
            condition=IfCondition(
                PythonExpression([str(i), " < ", LaunchConfiguration("num_drones")])
            ),
        )
        perception_nodes.append(node)

    return LaunchDescription(
        [
            num_drones_arg,
            model_arg,
            use_gpu_arg,
            confidence_arg,
            world_name_arg,
            set_pythonpath,
            *perception_nodes,
        ]
    )
