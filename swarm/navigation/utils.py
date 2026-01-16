"""Coordinate transform utilities for navigation."""

import numpy as np
from typing import Tuple


def rotation_matrix_from_euler(
    roll: float, pitch: float, yaw: float
) -> np.ndarray:
    """Create 3x3 rotation matrix from Euler angles (ZYX convention).

    Args:
        roll: Rotation around X axis (radians)
        pitch: Rotation around Y axis (radians)
        yaw: Rotation around Z axis (radians)

    Returns:
        3x3 rotation matrix (body to NED frame)
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    # ZYX Euler rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    return R


def euler_from_rotation_matrix(R: np.ndarray) -> Tuple[float, float, float]:
    """Extract Euler angles from rotation matrix (ZYX convention).

    Args:
        R: 3x3 rotation matrix

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    # Handle gimbal lock
    sy = -R[2, 0]
    if np.abs(sy) > 0.99999:
        # Gimbal lock: pitch is ±90 degrees
        pitch = np.sign(sy) * np.pi / 2
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    else:
        pitch = np.arcsin(sy)
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])

    return roll, pitch, yaw


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert Euler angles to quaternion [w, x, y, z].

    Args:
        roll: Rotation around X axis (radians)
        pitch: Rotation around Y axis (radians)
        yaw: Rotation around Z axis (radians)

    Returns:
        Quaternion as [w, x, y, z] numpy array
    """
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z], dtype=np.float64)


def euler_from_quaternion(q: np.ndarray) -> Tuple[float, float, float]:
    """Convert quaternion [w, x, y, z] to Euler angles.

    Args:
        q: Quaternion as [w, x, y, z] numpy array

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    w, x, y, z = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rotation_matrix_from_quaternion(q: np.ndarray) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix.

    Args:
        q: Quaternion as [w, x, y, z] numpy array

    Returns:
        3x3 rotation matrix
    """
    w, x, y, z = q

    # Ensure unit quaternion
    norm = np.linalg.norm(q)
    if norm > 0:
        w, x, y, z = q / norm

    R = np.array(
        [
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - w * z),
                2 * (x * z + w * y),
            ],
            [
                2 * (x * y + w * z),
                1 - 2 * (x * x + z * z),
                2 * (y * z - w * x),
            ],
            [
                2 * (x * z - w * y),
                2 * (y * z + w * x),
                1 - 2 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )
    return R


def quaternion_from_rotation_matrix(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion.

    Args:
        R: 3x3 rotation matrix

    Returns:
        Quaternion as [w, x, y, z] numpy array
    """
    trace = np.trace(R)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions (Hamilton product).

    Args:
        q1: First quaternion [w, x, y, z]
        q2: Second quaternion [w, x, y, z]

    Returns:
        Product quaternion [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """Compute quaternion conjugate.

    Args:
        q: Quaternion [w, x, y, z]

    Returns:
        Conjugate quaternion [w, -x, -y, -z]
    """
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quaternion_from_rotation_vector(rv: np.ndarray) -> np.ndarray:
    """Convert rotation vector (axis-angle) to quaternion.

    Args:
        rv: Rotation vector (axis * angle) in radians

    Returns:
        Quaternion [w, x, y, z]
    """
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        # Small angle approximation
        return np.array([1.0, rv[0] / 2, rv[1] / 2, rv[2] / 2], dtype=np.float64)

    axis = rv / angle
    half_angle = angle / 2
    sin_ha = np.sin(half_angle)

    return np.array(
        [np.cos(half_angle), axis[0] * sin_ha, axis[1] * sin_ha, axis[2] * sin_ha],
        dtype=np.float64,
    )


def body_to_ned(vector: np.ndarray, attitude: np.ndarray) -> np.ndarray:
    """Convert body frame vector to NED frame.

    Args:
        vector: [x, y, z] in body frame
        attitude: [roll, pitch, yaw] in radians OR quaternion [w, x, y, z]

    Returns:
        [north, east, down] in NED frame
    """
    if len(attitude) == 3:
        R = rotation_matrix_from_euler(attitude[0], attitude[1], attitude[2])
    else:
        R = rotation_matrix_from_quaternion(attitude)

    return R @ vector


def ned_to_body(vector: np.ndarray, attitude: np.ndarray) -> np.ndarray:
    """Convert NED frame vector to body frame.

    Args:
        vector: [north, east, down] in NED frame
        attitude: [roll, pitch, yaw] in radians OR quaternion [w, x, y, z]

    Returns:
        [x, y, z] in body frame
    """
    if len(attitude) == 3:
        R = rotation_matrix_from_euler(attitude[0], attitude[1], attitude[2])
    else:
        R = rotation_matrix_from_quaternion(attitude)

    return R.T @ vector


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """Create skew-symmetric matrix from vector.

    Args:
        v: 3-element vector

    Returns:
        3x3 skew-symmetric matrix
    """
    return np.array(
        [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=np.float64
    )


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range.

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def pose_to_transformation_matrix(
    position: np.ndarray, quaternion: np.ndarray
) -> np.ndarray:
    """Create 4x4 homogeneous transformation matrix from pose.

    Args:
        position: [x, y, z] position
        quaternion: [w, x, y, z] orientation

    Returns:
        4x4 transformation matrix
    """
    R = rotation_matrix_from_quaternion(quaternion)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = position
    return T


def transformation_matrix_to_pose(
    T: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Extract pose from 4x4 homogeneous transformation matrix.

    Args:
        T: 4x4 transformation matrix

    Returns:
        Tuple of (position, quaternion)
    """
    position = T[:3, 3].copy()
    quaternion = quaternion_from_rotation_matrix(T[:3, :3])
    return position, quaternion


def invert_transformation_matrix(T: np.ndarray) -> np.ndarray:
    """Invert a 4x4 homogeneous transformation matrix.

    Args:
        T: 4x4 transformation matrix

    Returns:
        Inverted transformation matrix
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def relative_pose(
    pose1_position: np.ndarray,
    pose1_quaternion: np.ndarray,
    pose2_position: np.ndarray,
    pose2_quaternion: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute relative pose between two poses (pose2 in pose1 frame).

    Args:
        pose1_position: Position of frame 1
        pose1_quaternion: Orientation of frame 1
        pose2_position: Position of frame 2
        pose2_quaternion: Orientation of frame 2

    Returns:
        Tuple of (relative_position, relative_quaternion)
    """
    T1 = pose_to_transformation_matrix(pose1_position, pose1_quaternion)
    T2 = pose_to_transformation_matrix(pose2_position, pose2_quaternion)
    T_rel = invert_transformation_matrix(T1) @ T2
    return transformation_matrix_to_pose(T_rel)
