"""
Launch file for individual drone (hardware deployment).

Runs DroneNode that connects to local flight controller and
participates in P2P swarm communication.

Usage:
    ros2 launch swarm_ros drone.launch.py drone_id:=0 num_drones:=6

For SITL testing (UDP instead of serial):
    ros2 launch swarm_ros drone.launch.py \\
        drone_id:=0 \\
        num_drones:=3 \\
        use_udp:=true \\
        udp_port:=14540
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    drone_id_arg = DeclareLaunchArgument(
        "drone_id",
        default_value="0",
        description="This drone's ID (0-indexed)",
    )

    num_drones_arg = DeclareLaunchArgument(
        "num_drones",
        default_value="3",
        description="Total number of drones in swarm",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port for flight controller",
    )

    serial_baud_arg = DeclareLaunchArgument(
        "serial_baud",
        default_value="115200",
        description="Serial baud rate",
    )

    use_udp_arg = DeclareLaunchArgument(
        "use_udp",
        default_value="false",
        description="Use UDP instead of serial (for SITL testing)",
    )

    udp_port_arg = DeclareLaunchArgument(
        "udp_port",
        default_value="14540",
        description="UDP port for SITL connection",
    )

    # DroneNode
    drone_node = Node(
        package="swarm_ros",
        executable="drone_node",
        name="drone_node",
        output="screen",
        parameters=[
            {
                "drone_id": LaunchConfiguration("drone_id"),
                "num_drones": LaunchConfiguration("num_drones"),
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baud": LaunchConfiguration("serial_baud"),
                "use_udp": LaunchConfiguration("use_udp"),
                "udp_port": LaunchConfiguration("udp_port"),
            }
        ],
    )

    return LaunchDescription(
        [
            drone_id_arg,
            num_drones_arg,
            serial_port_arg,
            serial_baud_arg,
            use_udp_arg,
            udp_port_arg,
            drone_node,
        ]
    )
