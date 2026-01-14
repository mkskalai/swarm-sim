#!/usr/bin/env python3
"""
Multi-drone swarm flight using pymavlink.

This script demonstrates coordinated multi-drone flight using pymavlink
for reliable port-isolated connections. Unlike MAVSDK which routes by
system ID (causing issues when all SITL instances have sysid=1), pymavlink
uses raw sockets where each connection is isolated by port.

Features:
    - Concurrent connection to multiple SITL instances
    - EKF convergence check before arming (fixes "Need Position Estimate" error)
    - Formation flying with continuous position updates at 4Hz
    - Vertical separation between drones to prevent collisions
    - Graceful interrupt handling (Ctrl+C lands all drones)

Available Formations:
    - triangle: Drones at center, front-right, front-left (different altitudes)
    - line: Drones in a line along East axis (different altitudes)
    - hover: All drones hover at their current position

Setup:
    1. Start Gazebo with multi-drone world:
       gz sim -v4 -r ~/ardupilot_gazebo/worlds/tutorial_multi_drone.sdf

    2. Start SITL instances with MAVProxy forwarding (3 separate terminals):
       sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0 --out=udp:127.0.0.1:14540
       sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I1 --out=udp:127.0.0.1:14541
       sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I2 --out=udp:127.0.0.1:14542

Usage:
    # Default: 3 drones, triangle then line formation
    python scripts/fly_swarm_pymavlink.py

    # Custom formations and timing
    python scripts/fly_swarm_pymavlink.py --formations hover,triangle,line --formation-time 10

    # Different number of drones
    python scripts/fly_swarm_pymavlink.py --num-drones 2 --formations hover

Arguments:
    --num-drones      Number of drones to connect (default: 3)
    --base-port       Starting UDP port (default: 14540)
    --altitude        Takeoff altitude in meters (default: 10.0)
    --formation-time  Seconds to hold each formation (default: 15.0)
    --formations      Comma-separated formation names (default: triangle,line)

Note:
    Each drone's NED frame origin is where it was armed, so formation positions
    are relative to each drone's spawn location. Vertical separation (8m, 10m,
    12m) prevents collisions during formation transitions.
"""

import time
import argparse
import sys
from pymavlink import mavutil

# Formation positions (NED: North, East, Down - Down is negative for up)
# Each drone's NED origin is where it armed, so these are relative positions.
# Vertical separation (8m, 10m, 12m) prevents collisions during transitions.
FORMATIONS = {
    'triangle': [
        (0.0, 0.0, -10.0),    # Drone 0: center, 10m
        (5.0, 5.0, -12.0),    # Drone 1: front-right, 12m (flies OVER)
        (5.0, -5.0, -8.0),    # Drone 2: front-left, 8m (flies UNDER)
    ],
    'line': [
        (0.0, 0.0, -10.0),    # Drone 0: 10m
        (0.0, 5.0, -12.0),    # Drone 1: 12m
        (0.0, 10.0, -8.0),    # Drone 2: 8m
    ],
    'hover': [
        (0.0, 0.0, -10.0),    # Drone 0: hover in place
        (0.0, 0.0, -12.0),    # Drone 1: hover in place
        (0.0, 0.0, -8.0),     # Drone 2: hover in place
    ],
}


def create_connection(port: int, timeout: float = 30.0) -> mavutil.mavlink_connection:
    """Create a MAVLink connection to a SITL instance."""
    print(f"Connecting to udpin:0.0.0.0:{port}...")
    conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{port}')

    # Wait for heartbeat
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            print(f"  Connected to system {conn.target_system} on port {port}")
            return conn

    raise TimeoutError(f"No heartbeat received on port {port} within {timeout}s")


def wait_for_ekf(conn: mavutil.mavlink_connection, timeout: float = 60.0) -> bool:
    """
    Wait for EKF to converge (position estimate valid).

    "Need Position Estimate" error means wait for EKF, not just GPS.
    EKF_STATUS_REPORT flags:
      - bit 3 (0x08): const_pos_mode (using constant position)
      - bit 4 (0x10): pred_pos_horiz_abs (predictive horizontal position absolute)
    """
    print(f"  Waiting for EKF convergence on system {conn.target_system}...")
    start = time.time()

    while time.time() - start < timeout:
        msg = conn.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
        if msg:
            # Check if position is OK (bits 3 or 4 set)
            pos_ok = (msg.flags & 0x18) != 0
            if pos_ok:
                print(f"  EKF converged on system {conn.target_system} (flags=0x{msg.flags:02x})")
                return True

    print(f"  WARNING: EKF timeout on system {conn.target_system}")
    return False


def set_mode(conn: mavutil.mavlink_connection, mode: str) -> bool:
    """Set flight mode."""
    mode_mapping = conn.mode_mapping()
    if mode not in mode_mapping:
        print(f"  Unknown mode: {mode}")
        return False

    mode_id = mode_mapping[mode]
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    # Wait for mode change confirmation
    start = time.time()
    while time.time() - start < 5.0:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            print(f"  Mode set to {mode} on system {conn.target_system}")
            return True

    return False


def arm(conn: mavutil.mavlink_connection, timeout: float = 10.0) -> bool:
    """Arm the drone."""
    print(f"  Arming system {conn.target_system}...")

    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm
        0, 0, 0, 0, 0, 0  # unused params
    )

    # Wait for arm confirmation
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print(f"  System {conn.target_system} armed")
            return True

    print(f"  WARNING: Arm timeout on system {conn.target_system}")
    return False


def disarm(conn: mavutil.mavlink_connection) -> bool:
    """Disarm the drone."""
    print(f"  Disarming system {conn.target_system}...")

    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        0,  # disarm
        0, 0, 0, 0, 0, 0  # unused params
    )

    time.sleep(1)
    return True


def takeoff(conn: mavutil.mavlink_connection, altitude: float = 10.0) -> bool:
    """Command takeoff to specified altitude."""
    print(f"  Takeoff system {conn.target_system} to {altitude}m...")

    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0,  # pitch (ignored for copter)
        0,  # empty
        0,  # empty
        0,  # yaw (NaN for current)
        0,  # latitude (0 for current)
        0,  # longitude (0 for current)
        altitude  # altitude
    )

    return True


def land(conn: mavutil.mavlink_connection) -> bool:
    """Command landing."""
    print(f"  Landing system {conn.target_system}...")

    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,  # confirmation
        0,  # abort altitude (0 for none)
        0,  # land mode
        0,  # empty
        0,  # yaw (NaN for current)
        0,  # latitude (0 for current)
        0,  # longitude (0 for current)
        0   # altitude (ignored)
    )

    return True


def get_position(conn: mavutil.mavlink_connection) -> tuple:
    """Get current position (lat, lon, alt)."""
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        return (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)
    return (0, 0, 0)


def get_position_ned(conn: mavutil.mavlink_connection, timeout: float = 1.0) -> tuple:
    """Get current position in local NED frame."""
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg:
        return (msg.x, msg.y, msg.z)
    return None


def wait_for_altitude(conn: mavutil.mavlink_connection, target_alt: float,
                      tolerance: float = 1.0, timeout: float = 30.0) -> bool:
    """Wait for drone to reach target altitude."""
    start = time.time()
    while time.time() - start < timeout:
        _, _, alt = get_position(conn)
        if abs(alt - target_alt) < tolerance:
            return True
        time.sleep(0.5)
    return False


def wait_for_position_ned(conn: mavutil.mavlink_connection,
                          target_n: float, target_e: float, target_d: float,
                          tolerance: float = 2.0, timeout: float = 30.0) -> bool:
    """Wait for drone to reach target NED position while sending commands."""
    start = time.time()
    interval = 0.25  # 4Hz

    while time.time() - start < timeout:
        # Keep sending position commands
        send_position_ned(conn, target_n, target_e, target_d)

        # Check current position
        pos = get_position_ned(conn, timeout=0.3)
        if pos:
            dist = ((pos[0] - target_n)**2 + (pos[1] - target_e)**2 + (pos[2] - target_d)**2)**0.5
            if dist <= tolerance:
                return True

        time.sleep(interval)

    return False


def send_position_ned(conn: mavutil.mavlink_connection,
                      north: float, east: float, down: float, yaw: float = 0):
    """Send position setpoint in local NED frame (must be called repeatedly)."""
    type_mask = 0b0000111111111000  # Position only, ignore velocity/acceleration

    conn.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        north, east, down,  # position
        0, 0, 0,            # velocity (ignored)
        0, 0, 0,            # acceleration (ignored)
        yaw, 0              # yaw, yaw_rate
    )


def fly_formation(connections: list, formation_name: str,
                  duration: float = 15.0, rate: float = 4.0,
                  arrival_timeout: float = 30.0, arrival_tolerance: float = 2.0):
    """
    Fly all drones to a formation and hold for specified duration.

    First waits for all drones to reach their formation positions,
    then holds for the specified duration.

    ArduPilot GUIDED mode requires continuous position updates (at least 1Hz).
    We send at 4Hz by default for smooth control.
    """
    if formation_name not in FORMATIONS:
        print(f"  Unknown formation: {formation_name}")
        return False

    positions = FORMATIONS[formation_name]
    if len(positions) < len(connections):
        print(f"  Formation {formation_name} doesn't have enough positions")
        return False

    print(f"  Flying to {formation_name} formation...")
    for i, conn in enumerate(connections):
        n, e, d = positions[i]
        print(f"    Drone {i}: N={n}, E={e}, Alt={-d}m")

    # Phase 1: Wait for all drones to reach their positions
    print(f"  Waiting for drones to reach positions (tolerance={arrival_tolerance}m)...")
    arrived = [False] * len(connections)
    interval = 1.0 / rate
    start = time.time()

    while time.time() - start < arrival_timeout:
        all_arrived = True
        for i, conn in enumerate(connections):
            n, e, d = positions[i]
            send_position_ned(conn, n, e, d)

            if not arrived[i]:
                pos = get_position_ned(conn, timeout=0.3)
                if pos:
                    dist = ((pos[0] - n)**2 + (pos[1] - e)**2 + (pos[2] - d)**2)**0.5
                    if dist <= arrival_tolerance:
                        arrived[i] = True
                        print(f"    Drone {i} arrived (dist={dist:.1f}m)")
                    else:
                        all_arrived = False
                else:
                    all_arrived = False

        if all_arrived:
            break

        time.sleep(interval)

    # Check if any drones didn't arrive
    for i, a in enumerate(arrived):
        if not a:
            print(f"    WARNING: Drone {i} did not reach position in time")

    # Phase 2: Hold formation for duration
    print(f"  Holding formation for {duration}s...")
    start = time.time()

    while time.time() - start < duration:
        for i, conn in enumerate(connections):
            n, e, d = positions[i]
            send_position_ned(conn, n, e, d)
        time.sleep(interval)

    print(f"  Formation {formation_name} complete.")
    return True


def main():
    parser = argparse.ArgumentParser(description='Fly multi-drone swarm with pymavlink')
    parser.add_argument('--num-drones', type=int, default=3, help='Number of drones')
    parser.add_argument('--base-port', type=int, default=14540, help='Base UDP port')
    parser.add_argument('--altitude', type=float, default=10.0, help='Takeoff altitude (m)')
    parser.add_argument('--formation-time', type=float, default=15.0, help='Time per formation (s)')
    parser.add_argument('--formations', type=str, default='triangle,line',
                        help='Comma-separated formation names (triangle, line, hover)')
    args = parser.parse_args()

    connections = []

    print(f"\n{'='*60}")
    print(f"Multi-Drone Swarm Flight (pymavlink)")
    print(f"{'='*60}")
    print(f"Drones: {args.num_drones}")
    print(f"Ports: {args.base_port} - {args.base_port + args.num_drones - 1}")
    print(f"Altitude: {args.altitude}m")
    print(f"Formations: {args.formations}")
    print(f"{'='*60}\n")

    try:
        # Phase 1: Connect to all drones
        print("[Phase 1] Connecting to drones...")
        for i in range(args.num_drones):
            port = args.base_port + i
            conn = create_connection(port)
            connections.append(conn)
        print(f"All {args.num_drones} drones connected.\n")

        # Phase 2: Wait for EKF on all drones
        print("[Phase 2] Waiting for EKF convergence...")
        for conn in connections:
            if not wait_for_ekf(conn):
                print(f"EKF failed on system {conn.target_system}, aborting.")
                return 1
        print("All drones have valid position estimates.\n")

        # Phase 3: Set GUIDED mode on all drones
        print("[Phase 3] Setting GUIDED mode...")
        for conn in connections:
            if not set_mode(conn, 'GUIDED'):
                print(f"Failed to set GUIDED mode on system {conn.target_system}")
                return 1
        print("All drones in GUIDED mode.\n")

        # Phase 4: Arm all drones
        print("[Phase 4] Arming drones...")
        for conn in connections:
            if not arm(conn):
                print(f"Failed to arm system {conn.target_system}")
                return 1
        print("All drones armed.\n")

        # Phase 5: Takeoff all drones
        print("[Phase 5] Taking off...")
        for conn in connections:
            takeoff(conn, args.altitude)

        # Wait for all drones to reach altitude
        print("Waiting for drones to reach altitude...")
        for i, conn in enumerate(connections):
            print(f"  Waiting for drone {i}...")
            if wait_for_altitude(conn, args.altitude, tolerance=2.0, timeout=30.0):
                print(f"  Drone {i} reached {args.altitude}m")
            else:
                print(f"  WARNING: Drone {i} altitude timeout")
        print("All drones airborne.\n")

        # Phase 6: Formation flying
        formation_list = [f.strip() for f in args.formations.split(',')]
        print(f"[Phase 6] Flying formations: {', '.join(formation_list)}")
        for i, formation in enumerate(formation_list):
            print(f"\n  [{i+1}/{len(formation_list)}] {formation.upper()} formation")
            fly_formation(connections, formation, duration=args.formation_time)
        print("\nFormation flying complete.\n")

        # Phase 7: Land all drones
        print("[Phase 7] Landing...")
        for conn in connections:
            land(conn)

        print("Waiting for landing...")
        time.sleep(15)

        # Phase 8: Disarm
        print("[Phase 8] Disarming...")
        for conn in connections:
            disarm(conn)

        print("\n" + "="*60)
        print("Flight complete!")
        print("="*60 + "\n")
        return 0

    except KeyboardInterrupt:
        print("\n\nInterrupted! Landing all drones...")
        for conn in connections:
            try:
                land(conn)
            except:
                pass
        time.sleep(5)
        for conn in connections:
            try:
                disarm(conn)
            except:
                pass
        return 1

    except Exception as e:
        print(f"\nError: {e}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
