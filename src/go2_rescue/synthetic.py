"""Deterministic software fixtures, NOT a physical Go2 or MuJoCo simulator.

Ground truth stays in this module. Autonomy receives only generated sensor frames.
This validates algorithms/data flow, never terrain ability or real robot success.
"""
from __future__ import annotations

import cv2
import numpy as np

from .config import Config
from .geometry import rot2, wrap
from .types import Frame, Pose, Velocity


def football_image(config: Config, center=(180, 165), radius=21, occlusion=0., seed=0):
    width, height = config.camera_size
    rng = np.random.default_rng(seed)
    image = np.full((height, width, 3), 112, dtype=np.uint8)
    x, y = map(int, center)
    radius = int(radius)
    cv2.circle(image, (x, y), radius, (230, 230, 230), -1)
    vertices = np.array([[x + .32 * radius * np.cos(a), y + .32 * radius * np.sin(a)]
                         for a in np.linspace(-np.pi / 2, 3 * np.pi / 2, 5, endpoint=False)], dtype=np.int32)
    cv2.fillConvexPoly(image, vertices, (20, 20, 20))
    for a in np.linspace(0, 2 * np.pi, 5, endpoint=False):
        pos = (int(x + .72 * radius * np.cos(a)), int(y + .72 * radius * np.sin(a)))
        cv2.circle(image, pos, max(2, radius // 5), (20, 20, 20), -1)
    if occlusion > 0:
        cut = int(x + radius * (1 - 2 * occlusion))
        cv2.rectangle(image, (cut, y - radius - 2), (x + radius + 3, y + radius + 3), (90, 90, 90), -1)
    noise = rng.normal(0, 1.2, image.shape)
    return np.clip(image.astype(float) + noise, 0, 255).astype(np.uint8)


class SensorWorld:
    def __init__(self, config: Config, seed: int = 0, mission_fixture: bool = False):
        self.c = config
        self.rng = np.random.default_rng(seed)
        self.pose = Pose()
        self.velocity = Velocity()
        self.time = 0.
        self.mission_fixture = mission_fixture
        self.obstacles = np.array([[1.8, .8, .4], [-1.5, 1.6, .45], [1.6, -1.6, .5]])

    def frame(self) -> Frame:
        # Four walls plus circular columns, raycast at 72 azimuths and 9 heights.
        points = []
        xy = self.pose.xy()
        for angle in np.linspace(-np.pi, np.pi, 72, endpoint=False):
            direction = np.array([np.cos(angle + self.pose.yaw), np.sin(angle + self.pose.yaw)])
            distances = []
            for axis in range(2):
                if abs(direction[axis]) > 1e-9:
                    for wall in (-3.5, 3.5):
                        value = (wall - xy[axis]) / direction[axis]
                        if value > 0:
                            distances.append(value)
            for ox, oy, radius in self.obstacles:
                offset = xy - [ox, oy]
                b = float(offset @ direction)
                discriminant = b * b - (offset @ offset - radius * radius)
                if discriminant > 0 and -b - np.sqrt(discriminant) > 0:
                    distances.append(-b - np.sqrt(discriminant))
            distance = min(distances)
            for elevation in np.linspace(-.75, .25, 9):
                floor_distance = self.c.base_height / -np.tan(elevation) if elevation < -.01 else np.inf
                ray_distance = min(distance, floor_distance)
                z = -self.c.base_height if floor_distance < distance else distance * np.tan(elevation)
                points.append([ray_distance * np.cos(angle), ray_distance * np.sin(angle), z])
        cloud = np.asarray(points) + self.rng.normal(0, .002, (len(points), 3))
        image = np.full((self.c.camera_size[1], self.c.camera_size[0], 3), 112, np.uint8)
        if self.mission_fixture:
            image = self._mission_image(image)
        return Frame(self.time, cloud, velocity=self.velocity, image=image)

    def _project(self, world_points):
        points = np.array(world_points, dtype=float).reshape(-1, 3)
        points[:, :2] = (points[:, :2] - self.pose.xy()) @ rot2(self.pose.yaw)
        points[:, 2] -= self.c.base_height
        transform = np.asarray(self.c.body_from_camera)
        optical = (points - transform[:3, 3]) @ transform[:3, :3]
        pixel = optical @ np.asarray(self.c.camera_matrix).T
        return pixel[:, :2] / pixel[:, 2:3], optical[:, 2]

    def _mission_image(self, image):
        # A nearby local lab marker and one football, projected through the same
        # camera calibration as real perception. No identities/positions are
        # delivered to Autonomy. These are deliberately simple regression scenes.
        pixel, depth = self._project([[1.6, .35, self.c.ball_radius]])
        if depth[0] > .2 and 25 < pixel[0, 0] < image.shape[1] - 25 and 25 < pixel[0, 1] < image.shape[0] - 25:
            radius = self.c.camera_matrix[0][0] * self.c.ball_radius / depth[0]
            image = football_image(self.c, pixel[0], radius, .15)
        x, z, s = .95, self.c.base_height + .08, self.c.finish_marker_size / 2
        corners, depth = self._project([[x, s, z + s], [x, -s, z + s], [x, -s, z - s], [x, s, z - s]])
        if (depth > .1).all() and (corners[:, 0] > 1).all() and (corners[:, 0] < image.shape[1] - 1).all() and (corners[:, 1] > 1).all() and (corners[:, 1] < image.shape[0] - 1).all():
            marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), self.c.finish_marker_id, 100)
            source = np.array([[0, 0], [99, 0], [99, 99], [0, 99]], np.float32)
            homography = cv2.getPerspectiveTransform(source, corners.astype(np.float32))
            size = (image.shape[1], image.shape[0])
            warped = cv2.warpPerspective(marker, homography, size, borderValue=112)
            mask = cv2.warpPerspective(np.full((100, 100), 255, np.uint8), homography, size)
            image[mask > 0] = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)[mask > 0]
        return image

    def advance(self, command: Velocity, dt=.1):
        # Test-plant kinematics only. No leg controller or physical claims.
        self.pose.x, self.pose.y = self.pose.xy() + rot2(self.pose.yaw) @ command.array()[:2] * dt
        self.pose.yaw = wrap(self.pose.yaw + command.wz * dt)
        self.velocity = command
        self.time += dt
