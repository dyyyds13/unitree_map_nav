import numpy as np
from scipy.spatial.transform import Rotation

from .types import Pose


def wrap(angle: float) -> float:
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


def rot2(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def transform_xy(points: np.ndarray, pose: Pose) -> np.ndarray:
    return points[:, :2] @ rot2(pose.yaw).T + pose.xy()


def gravity_rotation(quaternion: np.ndarray) -> np.ndarray:
    """Body -> gravity-aligned body, without importing absolute IMU yaw."""
    roll, pitch, _ = Rotation.from_quat(np.roll(quaternion, -1)).as_euler("xyz")
    return Rotation.from_euler("xyz", [roll, pitch, 0.]).as_matrix()


def body_to_map(point: np.ndarray, pose: Pose) -> np.ndarray:
    return rot2(pose.yaw) @ point[:2] + pose.xy()


def bresenham(start: tuple[int, int], end: tuple[int, int]):
    x, y = start
    x2, y2 = end
    dx, dy = abs(x2 - x), abs(y2 - y)
    sx, sy = (1 if x < x2 else -1), (1 if y < y2 else -1)
    err = dx - dy
    while True:
        yield x, y
        if x == x2 and y == y2:
            break
        e = 2 * err
        if e > -dy:
            err -= dy
            x += sx
        if e < dx:
            err += dx
            y += sy
