"""Classical lidar odometry, conservative loop detection and a planar pose graph.

The planar backend targets floor-level navigation, not arbitrary 6-DoF climbing.
KISS-ICP is available for 3-D odometry; both expose the same quality gate.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from .config import Config
from .geometry import gravity_rotation, rot2, transform_xy, wrap
from .types import Frame, Pose
from .terrain import ground_surface


def register(source: np.ndarray, target: np.ndarray, guess: Pose, limit: float = .65):
    """Trimmed point-to-point SE(2) ICP, source -> target."""
    if len(source) < 12 or len(target) < 12:
        return guess, 0.0, float("inf")
    tree = cKDTree(target[:, :2])
    r, t = rot2(guess.yaw), guess.xy()
    for _ in range(24):
        current = source[:, :2] @ r.T + t
        distance, indices = tree.query(current)
        valid = distance < limit
        if valid.sum() < 12:
            return guess, 0.0, float("inf")
        threshold = max(.035, np.quantile(distance[valid], .85))
        valid &= distance <= threshold
        a, b = current[valid], target[indices[valid], :2]
        ca, cb = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
        dr = vt.T @ u.T
        if np.linalg.det(dr) < 0:
            vt[-1] *= -1
            dr = vt.T @ u.T
        dt = cb - dr @ ca
        r, t = dr @ r, dr @ t + dt
        if np.linalg.norm(dt) < 1e-4 and abs(np.arctan2(dr[1, 0], dr[0, 0])) < 1e-4:
            break
    distance, _ = tree.query(source[:, :2] @ r.T + t)
    inliers = distance < min(limit, .25)
    overlap = float(inliers.mean())
    rmse = float(np.sqrt(np.mean(distance[inliers] ** 2))) if inliers.any() else float("inf")
    pose = Pose(float(t[0]), float(t[1]), float(np.arctan2(r[1, 0], r[0, 0])))
    return pose, overlap, rmse


def relative(a: Pose, b: Pose) -> np.ndarray:
    xy = rot2(a.yaw).T @ (b.xy() - a.xy())
    return np.array([*xy, wrap(b.yaw - a.yaw)])


def line_normals(target):
    tree = cKDTree(target)
    _, neighbours = tree.query(target, k=min(7, len(target)))
    patches = target[neighbours]
    centered = patches-patches.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    line = (eigenvalues[:, 1] > .001)&(eigenvalues[:, 0] < .22*eigenvalues[:, 1])
    return tree, eigenvectors[:, :, 0], line


def register_lines(source, target, guess, limit=.65, yaw_hint=None):
    """Robust point-to-line ICP with a weak measured-motion prior.

    Along a featureless wall, point-to-point correspondences artificially pin
    the robot to the wall's sampling pattern. Line normals leave that direction
    unobserved, so measured joint/IMU motion supplies it until corners reappear.
    """
    if len(source) < 12 or len(target) < 12:
        return guess, 0., float("inf")
    target = target[:, :2]
    tree, normals, line = line_normals(target)
    if line.sum() < 12:
        return register(source, target, guess, limit)
    prior = np.array([guess.x, guess.y, guess.yaw])
    if yaw_hint is not None:
        prior[2] = yaw_hint
    value = prior.copy()
    regularization = np.array([.7, .7, 2.])
    if yaw_hint is not None:
        regularization[2] = 1./.025**2
    for _ in range(18):
        rotated = source[:, :2]@rot2(value[2]).T
        current = rotated+value[:2]
        distance, indices = tree.query(current)
        normal = normals[indices]
        residual = np.sum(normal*(current-target[indices]), axis=1)
        valid = (distance < limit)&line[indices]&(abs(residual) < .25)
        if valid.sum() < 12:
            return guess, 0., float("inf")
        n, r, q = normal[valid], residual[valid], rotated[valid]
        jacobian = np.column_stack([n, -n[:, 0]*q[:, 1]+n[:, 1]*q[:, 0]])
        weight = np.minimum(1., .04/np.maximum(abs(r), 1e-9))
        hessian = jacobian.T@(jacobian*weight[:, None])+np.diag(regularization)
        offset = value-prior; offset[2] = wrap(offset[2])
        gradient = jacobian.T@(weight*r)+regularization*offset
        delta = -np.linalg.solve(hessian, gradient)
        delta[:2] = np.clip(delta[:2], -.15, .15)
        delta[2] = np.clip(delta[2], -.15, .15)
        value += delta
        if np.linalg.norm(delta) < 1e-5:
            break
    distance, indices = tree.query(source[:, :2]@rot2(value[2]).T+value[:2])
    normal = normals[indices]
    residual = np.sum(normal*(source[:, :2]@rot2(value[2]).T+value[:2]-target[indices]), axis=1)
    inliers = (distance < .4)&(abs(residual) < .12)&line[indices]
    support = (distance < limit)&line[indices]
    # Judge residual agreement on usable line correspondences. Horizontal or
    # irregular surfaces are deliberately outside this metric, not failed wall
    # matches. Retain a coverage penalty so a tiny matching subset cannot make
    # an otherwise unrelated scan look reliable.
    coverage = min(1., float(support.mean())/.60)
    overlap = float(inliers.sum()/max(1, support.sum())*coverage) if inliers.sum() >= 12 else 0.
    rmse = float(np.sqrt(np.mean(residual[inliers]**2))) if inliers.any() else float("inf")
    return Pose(*value), overlap, rmse


@dataclass
class Keyframe:
    pose: Pose
    points: np.ndarray
    scan: np.ndarray


class PoseGraph:
    def __init__(self):
        self.nodes: list[Pose] = []
        self.edges: list[tuple[int, int, np.ndarray, float]] = []

    def optimize(self, heading_priors=None) -> list[Pose]:
        if len(self.nodes) < 3:
            return self.nodes
        anchor = np.array([self.nodes[0].x, self.nodes[0].y, self.nodes[0].yaw])
        initial = np.array([[p.x, p.y, p.yaw] for p in self.nodes[1:]]).ravel()

        def residual(x):
            values = np.vstack([anchor, x.reshape(-1, 3)])
            result = []
            for i, j, measurement, weight in self.edges:
                a, b = values[i], values[j]
                delta = rot2(a[2]).T @ (b[:2] - a[:2])
                error = np.array([*(delta - measurement[:2]), wrap(b[2] - a[2] - measurement[2])])
                result.extend(error * np.array([1., 1., 2.]) * weight)
            if heading_priors is not None:
                result.extend([wrap(v[2]-heading)/.025 for v, heading in zip(values[1:], heading_priors[1:])])
            return result

        result = least_squares(residual, initial, loss="soft_l1", f_scale=.1, max_nfev=60)
        if result.success and np.isfinite(result.x).all():
            self.nodes = [Pose(*v, quality=p.quality, sigma=p.sigma) for v, p in zip(
                np.vstack([anchor, result.x.reshape(-1, 3)]), self.nodes)]
        return self.nodes


class Localization:
    def __init__(self, config: Config):
        self.c = config
        self.pose = Pose()
        self.last_timestamp: float | None = None
        self.keyframes: list[Keyframe] = []
        self.graph = PoseGraph()
        self.revision = 0
        self.corrections: list[tuple[Pose, Pose]] = []
        self.kiss = None
        self.heading_origin = None
        self.keyframe_headings = []
        self.sensor_heading = None
        if config.localization_backend == "kiss":
            from kiss_icp.config import KISSConfig
            from kiss_icp.kiss_icp import KissICP
            settings = KISSConfig()
            settings.data.max_range = config.max_range
            settings.data.min_range = .25
            settings.data.deskew = False  # Enabled per frame only with real point times.
            settings.mapping.voxel_size = .12
            self.kiss = KissICP(settings)

    def structural_scan(self, points, pose=None):
        pose = self.pose if pose is None else pose
        limit = lambda xy: self.c.slope_limit(xy @ rot2(pose.yaw).T+pose.xy())
        samples = ground_surface(points + [0., 0., self.c.base_height], 0.,
                                 self.c.resolution, self.c.ground_max_gap, limit)
        ground = np.zeros(len(points), bool)
        if len(samples):
            distance, indices = cKDTree(samples[:, :2]).query(points[:, :2])
            ground = (distance < self.c.resolution) & (abs(points[:, 2] + self.c.base_height - samples[indices, 2]) < .04)
        scan = points[(~ground) & (points[:, 2] > -.22) & (points[:, 2] < .75)]
        if self.c.localization_metric == "point_to_line" and len(scan) >= 12:
            # Horizontal overhead planes have no stable XY correspondence for
            # planar odometry. Keep them in the returned cloud for clearance,
            # but do not let ceiling sampling drown out vertical wall matches.
            _, indices = cKDTree(scan).query(scan, k=min(10, len(scan)))
            patches = scan[indices]
            centered = patches-patches.mean(axis=1, keepdims=True)
            values, vectors = np.linalg.eigh(np.einsum("nki,nkj->nij", centered, centered))
            horizontal = ((scan[:, 2] > .16)&(abs(vectors[:, 2, 0]) > .90)
                          &(values[:, 1] > .0005)&(values[:, 0] < .20*values[:, 1]))
            scan = scan[~horizontal]
        if len(scan):
            _, indices = np.unique(np.floor(scan[:, :2]/.07).astype(int), axis=0, return_index=True)
            scan = scan[indices]
        return scan

    def update(self, frame: Frame) -> tuple[Pose, np.ndarray]:
        frame.validate()
        self.corrections = []
        points = frame.points @ gravity_rotation(frame.quaternion).T
        ranges = np.linalg.norm(points, axis=1)
        points = points[(ranges > .25) & (ranges < self.c.max_range)]
        scan = self.structural_scan(points)
        if self.last_timestamp is not None and frame.timestamp <= self.last_timestamp:
            raise ValueError("Sensor timestamps must increase strictly")
        dt = 0. if self.last_timestamp is None else frame.timestamp - self.last_timestamp
        self.last_timestamp = frame.timestamp
        if len(scan) < self.c.min_points or dt > self.c.max_frame_gap:
            self.pose.quality = 0.
            return self.pose, points
        spread = np.linalg.eigvalsh(np.cov(scan[:, :2].T))
        geometry = float(np.clip(spread[0] / max(.12, spread[-1] * .08), 0., 1.))
        if not self.keyframes:
            self.heading_origin = self._imu_yaw(frame.quaternion)
            self.sensor_heading = 0.
            self.pose = Pose(quality=.8 * geometry, sigma=.08)
            if self.kiss:
                self.kiss.register_frame(points, np.zeros(len(points)))
            self._keyframe(points)
            return self.pose, points
        old = self.pose
        displacement = rot2(old.yaw) @ (frame.velocity.array()[:2] * dt)
        guess = Pose(*(old.xy() + displacement), wrap(old.yaw + frame.velocity.wz * dt))
        if self.c.imu_heading_reference:
            self.sensor_heading = wrap(self._imu_yaw(frame.quaternion)-self.heading_origin)
            guess.yaw = self.sensor_heading
        target = np.vstack([transform_xy(k.scan, k.pose)
                            for k in self.keyframes[-12:]])
        if self.kiss is not None:
            # KISS expects timestamps matching the filtered cloud. Motion compensation
            # stays disabled here because gravity/range filtering changes indices.
            self.kiss.register_frame(points, np.zeros(len(points)))
            t = self.kiss.last_pose
            candidate = Pose(float(t[0, 3]), float(t[1, 3]), float(np.arctan2(t[1, 0], t[0, 0])))
            distances, _ = cKDTree(target).query(transform_xy(scan, candidate))
            inside = distances < .25
            overlap = float(inside.mean())
            rmse = float(np.sqrt(np.mean(distances[inside] ** 2))) if inside.any() else 1.
        else:
            registration = register_lines if self.c.localization_metric == "point_to_line" else register
            kwargs = {"yaw_hint": self.sensor_heading} if self.c.imu_heading_reference and registration is register_lines else {}
            candidate, overlap, rmse = registration(scan, target, guess, **kwargs)
        quality = geometry * overlap * np.exp(-rmse / .20)
        if np.linalg.norm(candidate.xy() - old.xy()) > max(.25, dt * 1.2) or abs(wrap(candidate.yaw - old.yaw)) > max(.4, dt * 2.):
            quality = 0.
        if quality < self.c.min_pose_quality:
            self.pose = Pose(old.x, old.y, old.yaw, float(quality), min(2., old.sigma + .05))
            return self.pose, points
        candidate.quality, candidate.sigma = float(quality), float(max(.03, rmse))
        self.pose = candidate
        last = self.keyframes[-1].pose
        if np.linalg.norm(last.xy() - candidate.xy()) > .45 or abs(wrap(last.yaw - candidate.yaw)) > .3:
            self._keyframe(points)
        return self.pose, points

    @staticmethod
    def _imu_yaw(quaternion):
        w, x, y, z = quaternion
        return float(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))

    def _keyframe(self, points: np.ndarray):
        pose = Pose(**vars(self.pose))
        self.keyframes.append(Keyframe(pose, points.copy(), self.structural_scan(points)))
        self.graph.nodes.append(pose)
        self.keyframe_headings.append(self.sensor_heading if self.c.imu_heading_reference else pose.yaw)
        n = len(self.keyframes) - 1
        if n:
            self.graph.edges.append((n - 1, n, relative(self.keyframes[n - 1].pose, pose), 1.))
        # The 3-D backend maintains its own odometry map; do not apply an SE(2)
        # correction to that map without a corresponding backend reset.
        if n < 18 or self.kiss is not None:
            return
        candidates = [(np.linalg.norm(k.pose.xy() - pose.xy()), i) for i, k in enumerate(self.keyframes[:-15])]
        distance, i = min(candidates)
        if distance > .75:
            return
        old = self.keyframes[i]
        source = self.structural_scan(points)[:, :2]
        target = old.scan[:, :2]
        delta = relative(old.pose, pose)
        if self.c.localization_metric == "point_to_line" and len(target) >= 12:
            _, normals, valid = line_normals(target)
            spectrum = np.linalg.eigvalsh(normals[valid].T@normals[valid])
            # Parallel walls do not constrain longitudinal loop displacement.
            # Such a loop must not fold an otherwise valid odometry trajectory.
            if valid.sum() < 12 or spectrum[0] < .05*max(spectrum[-1], 1.):
                return
        registration = register_lines if self.c.localization_metric == "point_to_line" else register
        fit, overlap, rmse = registration(source, target, Pose(*delta), limit=.35)
        if overlap < .85 or rmse > .06 or np.linalg.norm(fit.xy() - delta[:2]) > .4:
            return
        self.graph.edges.append((i, n, np.array([fit.x, fit.y, fit.yaw]), 2.))
        previous = [Pose(**vars(k.pose)) for k in self.keyframes]
        optimized = self.graph.optimize(self.keyframe_headings if self.c.imu_heading_reference else None)
        self.corrections = list(zip(previous, optimized))
        for key, updated in zip(self.keyframes, optimized):
            key.pose = updated
        self.pose = self.keyframes[-1].pose
        self.revision += 1
