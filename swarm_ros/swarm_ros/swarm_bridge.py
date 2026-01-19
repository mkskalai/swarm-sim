"""
SwarmBridge - Bridge SwarmController to ROS2 topics and services.

For simulation mode: wraps the existing SwarmController (pymavlink-based)
and exposes it via ROS2 topics and services. This enables ROS2 tooling
(RViz2, rosbag2) without changing the proven SwarmController implementation.

Example:
    # Terminal 1: Start Gazebo + SITL
    python scripts/run_phase3_test.py --num-drones 3 --skip-test

    # Terminal 2: Start bridge
    ros2 run swarm_ros swarm_bridge --ros-args -p num_drones:=3

    # Terminal 3: Monitor
    ros2 topic echo /swarm/status
    ros2 service call /swarm/set_formation swarm_ros/srv/SetFormation ...
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


class SwarmBridge(Node):
    """
    ROS2 node that bridges SwarmController to ROS2 topics/services.

    Publishers:
        /swarm/status (SwarmStatus) @ 1Hz - Aggregated swarm status
        /drone_N/state (DroneState) @ 10Hz - Per-drone state

    Services:
        /swarm/set_formation (SetFormation) - Change formation
        /swarm/takeoff_all (Trigger) - Takeoff all drones
        /swarm/land_all (Trigger) - Land all drones
        /swarm/emergency (EmergencyCommand) - Emergency commands
    """

    def __init__(self):
        super().__init__("swarm_bridge")

        # Declare parameters
        self.declare_parameter("num_drones", 3)
        self.declare_parameter("base_port", 14540)
        self.declare_parameter("auto_connect", True)

        self.num_drones = self.get_parameter("num_drones").value
        self.base_port = self.get_parameter("base_port").value
        self.auto_connect = self.get_parameter("auto_connect").value

        # Import swarm modules (will fail if not in path)
        try:
            from swarm.coordination import SwarmController, SwarmConfig, FormationType
            from swarm.coordination.formations import FormationConfig

            self._SwarmController = SwarmController
            self._SwarmConfig = SwarmConfig
            self._FormationType = FormationType
            self._FormationConfig = FormationConfig
        except ImportError as e:
            self.get_logger().error(
                f"Failed to import swarm modules: {e}. "
                "Make sure the swarm package is in your Python path."
            )
            raise

        # Import ROS2 message types
        try:
            from swarm_ros.msg import DroneState, SwarmStatus
            from swarm_ros.srv import SetFormation, EmergencyCommand

            self._DroneState = DroneState
            self._SwarmStatus = SwarmStatus
            self._SetFormation = SetFormation
            self._EmergencyCommand = EmergencyCommand
        except ImportError as e:
            self.get_logger().error(
                f"Failed to import swarm_ros messages: {e}. "
                "Run 'colcon build' first."
            )
            raise

        # Create SwarmController
        config = self._SwarmConfig(
            num_drones=self.num_drones,
            base_port=self.base_port,
        )
        self.controller = self._SwarmController(config)

        # QoS profiles
        self._reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.status_pub = self.create_publisher(
            self._SwarmStatus, "/swarm/status", self._reliable_qos
        )
        self.drone_state_pubs = {}
        for i in range(self.num_drones):
            pub = self.create_publisher(
                self._DroneState, f"/drone_{i}/state", self._best_effort_qos
            )
            self.drone_state_pubs[i] = pub

        # Services
        self.create_service(
            self._SetFormation, "/swarm/set_formation", self._handle_set_formation
        )
        self.create_service(Trigger, "/swarm/connect", self._handle_connect)
        self.create_service(Trigger, "/swarm/takeoff_all", self._handle_takeoff)
        self.create_service(Trigger, "/swarm/land_all", self._handle_land)
        self.create_service(Trigger, "/swarm/arm_all", self._handle_arm)
        self.create_service(Trigger, "/swarm/disarm_all", self._handle_disarm)
        self.create_service(
            self._EmergencyCommand, "/swarm/emergency", self._handle_emergency
        )

        # Timers
        self.create_timer(1.0, self._publish_swarm_status)  # 1 Hz
        self.create_timer(0.1, self._publish_drone_states)  # 10 Hz

        # State tracking
        self._connected = False
        self._armed = False
        self._in_flight = False
        self._current_formation = None
        self._mission_name = ""
        self._mission_progress = 0.0

        # Lock for thread-safe controller access
        self._lock = threading.Lock()

        self.get_logger().info(
            f"SwarmBridge initialized for {self.num_drones} drones "
            f"on ports {self.base_port}-{self.base_port + self.num_drones - 1}"
        )

        # Auto-connect if configured
        if self.auto_connect:
            self.get_logger().info("Auto-connecting to drones...")
            self._do_connect()

    def _do_connect(self) -> tuple[bool, str]:
        """Connect to all drones (blocking)."""
        with self._lock:
            if self._connected:
                return True, "Already connected"

            try:
                success = self.controller.connect_all()
                if success:
                    # Wait for EKF
                    self.get_logger().info("Waiting for EKF convergence...")
                    ekf_ok = self.controller.wait_for_ekf_all()
                    if ekf_ok:
                        self._connected = True
                        return True, "Connected and EKF converged"
                    else:
                        return False, "EKF convergence timeout"
                else:
                    return False, "Connection failed"
            except Exception as e:
                return False, f"Connection error: {e}"

    def _publish_swarm_status(self) -> None:
        """Publish aggregated swarm status."""
        msg = self._SwarmStatus()
        msg.num_drones_configured = self.num_drones
        msg.num_drones_connected = 0
        msg.num_drones_armed = 0
        msg.num_drones_in_flight = 0
        msg.num_drones_failed = 0

        positions = {}
        active = set()
        failed = set()

        if self._connected:
            with self._lock:
                active = self.controller.failure_handler.get_active_drones()
                failed = self.controller.failure_handler.get_failed_drones()
                try:
                    positions = self.controller._get_all_positions(timeout=0.1)
                except Exception:
                    pass

            msg.num_drones_connected = len(active)
            msg.num_drones_failed = len(failed)

            if self._armed:
                msg.num_drones_armed = len(active)
            if self._in_flight:
                msg.num_drones_in_flight = len(active)

        # Build drone_states array
        now = self.get_clock().now().to_msg()
        for drone_id in range(self.num_drones):
            drone_msg = self._DroneState()
            drone_msg.drone_id = drone_id

            if drone_id in positions:
                pos = positions[drone_id]
                drone_msg.position_ned = [pos[0], pos[1], pos[2]]
            else:
                drone_msg.position_ned = [0.0, 0.0, 0.0]

            drone_msg.velocity_ned = [0.0, 0.0, 0.0]
            drone_msg.yaw_deg = 0.0

            if not self._connected:
                drone_msg.state = drone_msg.STATE_DISCONNECTED
            elif not self._armed:
                drone_msg.state = drone_msg.STATE_CONNECTED
            elif not self._in_flight:
                drone_msg.state = drone_msg.STATE_ARMED
            else:
                drone_msg.state = drone_msg.STATE_IN_FLIGHT

            drone_msg.battery_percent = 100
            drone_msg.is_healthy = drone_id in active
            drone_msg.has_gps_lock = True
            drone_msg.ekf_ok = True
            drone_msg.stamp = now

            msg.drone_states.append(drone_msg)

        # Set swarm state
        if not self._connected:
            msg.swarm_state = msg.SWARM_IDLE
        elif not self._armed:
            msg.swarm_state = msg.SWARM_READY
        elif not self._in_flight:
            msg.swarm_state = msg.SWARM_ARMING
        elif self._current_formation is not None:
            msg.swarm_state = msg.SWARM_IN_FORMATION
            msg.current_formation_type = self._current_formation
        else:
            msg.swarm_state = msg.SWARM_EXECUTING_MISSION

        msg.current_mission_name = self._mission_name
        msg.mission_progress = self._mission_progress
        msg.stamp = now

        self.status_pub.publish(msg)

    def _publish_drone_states(self) -> None:
        """Publish individual drone states."""
        if not self._connected:
            return

        with self._lock:
            try:
                positions = self.controller._get_all_positions(timeout=0.1)
            except Exception:
                return

        for drone_id, pub in self.drone_state_pubs.items():
            msg = self._DroneState()
            msg.drone_id = drone_id

            if drone_id in positions:
                pos = positions[drone_id]
                msg.position_ned = [pos[0], pos[1], pos[2]]
            else:
                msg.position_ned = [0.0, 0.0, 0.0]

            msg.velocity_ned = [0.0, 0.0, 0.0]  # TODO: get velocity from controller
            msg.yaw_deg = 0.0

            # Set state based on bridge state
            if not self._connected:
                msg.state = msg.STATE_DISCONNECTED
            elif not self._armed:
                msg.state = msg.STATE_CONNECTED
            elif not self._in_flight:
                msg.state = msg.STATE_ARMED
            else:
                msg.state = msg.STATE_IN_FLIGHT

            msg.battery_percent = 100  # TODO: get from telemetry
            msg.is_healthy = drone_id in self.controller.failure_handler.get_active_drones()
            msg.has_gps_lock = True
            msg.ekf_ok = True
            msg.stamp = self.get_clock().now().to_msg()

            pub.publish(msg)

    # ----- Service Handlers -----

    def _handle_connect(self, request, response):
        """Handle connect service call."""
        success, message = self._do_connect()
        response.success = success
        response.message = message
        return response

    def _handle_set_formation(self, request, response):
        """Handle set_formation service call."""
        if not self._connected:
            response.success = False
            response.message = "Not connected"
            return response

        if not self._armed:
            response.success = False
            response.message = "Not armed"
            return response

        # Map integer formation type to FormationType enum
        formation_map = {
            0: self._FormationType.LINE,
            1: self._FormationType.GRID,
            2: self._FormationType.STAR,
        }

        if request.formation_type not in formation_map:
            response.success = False
            response.message = f"Invalid formation type: {request.formation_type}. Valid: 0=LINE, 1=GRID, 2=STAR"
            return response

        formation_type = formation_map[request.formation_type]

        # Build formation config
        config = self._FormationConfig(
            spacing=request.spacing if request.spacing > 0 else 5.0,
            altitude=request.altitude if request.altitude > 0 else 10.0,
            heading=request.heading,
        )

        duration = request.duration if request.duration > 0 else 30.0
        tolerance = request.arrival_tolerance if request.arrival_tolerance > 0 else 2.0

        self.get_logger().info(
            f"Flying formation {formation_type.name} for {duration}s"
        )

        # Run formation in separate thread to not block service
        def fly_formation():
            with self._lock:
                self._current_formation = request.formation_type
                try:
                    success = self.controller.fly_formation(
                        formation_type,
                        duration=duration,
                        config=config,
                        arrival_tolerance=tolerance,
                        arrival_timeout=30.0,
                    )
                    return success
                finally:
                    self._current_formation = None

        if request.wait_for_arrival:
            start = time.time()
            success = fly_formation()
            elapsed = time.time() - start

            response.success = success
            response.message = "Formation complete" if success else "Formation failed"
            response.arrival_time_seconds = elapsed
            response.drones_in_position = self.num_drones if success else 0
        else:
            # Non-blocking: start in thread
            thread = threading.Thread(target=fly_formation, daemon=True)
            thread.start()
            response.success = True
            response.message = "Formation started"
            response.arrival_time_seconds = 0.0
            response.drones_in_position = 0

        return response

    def _handle_takeoff(self, request, response):
        """Handle takeoff service call."""
        if not self._connected:
            response.success = False
            response.message = "Not connected"
            return response

        with self._lock:
            try:
                # Set mode to GUIDED first
                self.controller.set_mode_all("GUIDED")

                # Arm if not armed
                if not self._armed:
                    self.controller.arm_all()
                    self._armed = True

                # Takeoff
                success = self.controller.takeoff_all(altitude=10.0)
                if success:
                    self._in_flight = True
                    response.success = True
                    response.message = "Takeoff successful"
                else:
                    response.success = False
                    response.message = "Takeoff failed"
            except Exception as e:
                response.success = False
                response.message = f"Takeoff error: {e}"

        return response

    def _handle_land(self, request, response):
        """Handle land service call."""
        if not self._connected:
            response.success = False
            response.message = "Not connected"
            return response

        with self._lock:
            try:
                self.controller.land_all()
                self._in_flight = False
                response.success = True
                response.message = "Land command sent"
            except Exception as e:
                response.success = False
                response.message = f"Land error: {e}"

        return response

    def _handle_arm(self, request, response):
        """Handle arm service call."""
        if not self._connected:
            response.success = False
            response.message = "Not connected"
            return response

        with self._lock:
            try:
                self.controller.set_mode_all("GUIDED")
                success = self.controller.arm_all()
                if success:
                    self._armed = True
                    response.success = True
                    response.message = "Armed successfully"
                else:
                    response.success = False
                    response.message = "Arm failed"
            except Exception as e:
                response.success = False
                response.message = f"Arm error: {e}"

        return response

    def _handle_disarm(self, request, response):
        """Handle disarm service call."""
        if not self._connected:
            response.success = False
            response.message = "Not connected"
            return response

        with self._lock:
            try:
                self.controller.disarm_all()
                self._armed = False
                self._in_flight = False
                response.success = True
                response.message = "Disarmed"
            except Exception as e:
                response.success = False
                response.message = f"Disarm error: {e}"

        return response

    def _handle_emergency(self, request, response):
        """Handle emergency command."""
        if not self._connected:
            response.success = False
            response.message = "Not connected"
            response.drones_acknowledged = 0
            return response

        cmd = request.command
        drone_id = request.drone_id

        with self._lock:
            try:
                if cmd == request.CMD_RTL:
                    self.controller.return_to_launch_all()
                    self._in_flight = False
                    response.message = "RTL command sent"
                elif cmd == request.CMD_LAND:
                    self.controller.land_all()
                    self._in_flight = False
                    response.message = "Land command sent"
                elif cmd == request.CMD_HOVER:
                    # Stop current operation (formations will exit naturally)
                    response.message = "Hover command acknowledged"
                elif cmd == request.CMD_DISARM:
                    self.controller.disarm_all()
                    self._armed = False
                    self._in_flight = False
                    response.message = "Emergency disarm sent"
                else:
                    response.success = False
                    response.message = f"Unknown command: {cmd}"
                    return response

                response.success = True
                response.drones_acknowledged = self.num_drones
            except Exception as e:
                response.success = False
                response.message = f"Emergency error: {e}"
                response.drones_acknowledged = 0

        return response

    def destroy_node(self):
        """Clean up on shutdown."""
        self.get_logger().info("Shutting down SwarmBridge...")
        if self._connected:
            with self._lock:
                try:
                    if self._in_flight:
                        self.controller.land_all()
                    self.controller.disconnect_all()
                except Exception as e:
                    self.get_logger().error(f"Shutdown error: {e}")
        super().destroy_node()


def main(args=None):
    """Entry point for swarm_bridge node."""
    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    rclpy.init(args=args)

    try:
        node = SwarmBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"SwarmBridge failed: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
