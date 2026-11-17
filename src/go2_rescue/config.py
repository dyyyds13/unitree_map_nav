from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
import json
import math
import numpy as np


@dataclass
class Config:
    hardware_model: str = "unknown"
    network_interface: str = ""
    calibrated: bool = False
    localization_backend: str = "icp"
    localization_metric: str = "point_to_point"
    imu_heading_reference: bool = False  # Enable only with a stable orientation sensor.
    map_size: float = 40.0
    resolution: float = 0.10
    robot_radius: float = 0.32
    safety_margin: float = 0.12
    footprint_half_length: float = 0.0
    footprint_half_width: float = 0.0
    hazard_foot_half_length: float = 0.0
    hazard_foot_half_width: float = 0.0
    base_height: float = 0.35
    min_obstacle_height: float = 0.10
    body_clearance: float = 0.55
    max_step_height: float = 0.08
    max_slope_deg: float = 10.0
    terrain_slope_regions: list = field(default_factory=list)  # Explicit map_start terrain capabilities, no map clearing.
    ground_max_gap: float = 0.45
    water_detection: bool = False
    water_hsv_lower: list = field(default_factory=lambda: [80, 65, 35])
    water_hsv_upper: list = field(default_factory=lambda: [125, 255, 255])
    max_vx: float = 0.30
    max_reverse_vx: float = 0.08  # Independent conservative hardware default.
    passage_max_vx: float = 0.08  # Hardware default; simulator sets its own profile.
    max_vy: float = 0.12
    max_wz: float = 0.55
    acceleration: float = 0.45
    angular_acceleration: float = 1.2
    braking: float = 0.50
    command_timeout: float = 0.30
    max_sensor_age: float = 0.30
    max_frame_gap: float = 0.50
    max_range: float = 8.0
    min_points: int = 30
    min_pose_quality: float = 0.45
    mission_seconds: float = 180.0
    return_reserve: float = 35.0
    target_quota: int = 0  # Zero means unknown; positive only for declared test tasks.
    course_route: str = ""  # Explicit fixed-map prior, never simulator state.
    course_portal_resolution: float = 0.  # Optional finer local-course map after ramp/S/dead end.
    allow_margin_escape: bool = False  # Known simulated envelope only by default.
    search_mission: bool = False
    search_coverage_threshold: float = .90
    search_bounds: list = field(default_factory=list)  # Public map_start rectangle [xmin,xmax,ymin,ymax].
    search_detection_range: float = 3.5  # Distant ground-ray hypotheses are too uncertain to report.
    search_finish_approach: list = field(default_factory=list)  # Public-map priors, never victim coordinates.
    search_finish_gate_yaw: float | None = None  # Heading of a declared approach passage.
    search_finish_gate_index: int | None = None
    ball_radius: float = 0.11
    ball_confirmations: int = 3
    min_ball_confidence: float = 0.62
    camera_matrix: list = field(default_factory=lambda: [[260., 0., 160.], [0., 260., 120.], [0., 0., 1.]])
    distortion: list = field(default_factory=lambda: [0., 0., 0., 0., 0.])
    # OpenCV optical frame: right/down/forward -> body: forward/left/up.
    body_from_camera: list = field(default_factory=lambda: [[0., 0., 1., .24], [-1., 0., 0., 0.], [0., -1., 0., .08], [0., 0., 0., 1.]])
    body_from_lidar: list = field(default_factory=lambda: np.eye(4).tolist())
    camera_size: list = field(default_factory=lambda: [320, 240])
    finish_marker_id: int = 42
    finish_marker_size: float = 0.20
    finish_standoff: float = 0.65

    @classmethod
    def load(cls, path: str | Path | None = None) -> "Config":
        value = cls(**json.loads(Path(path).read_text())) if path else cls()
        value.validate()
        return value

    def validate(self) -> None:
        numbers = [v for v in asdict(self).values() if isinstance(v, (int, float)) and not isinstance(v, bool)]
        if not np.isfinite(numbers).all():
            raise ValueError("Configuration contains a non-finite number")
        positive = [self.map_size, self.resolution, self.robot_radius, self.base_height,
                    self.acceleration, self.angular_acceleration, self.braking, self.command_timeout,
                    self.max_sensor_age, self.max_frame_gap, self.max_range, self.max_vx,
                    self.max_wz, self.max_reverse_vx, self.passage_max_vx, self.mission_seconds, self.ball_radius,
                    self.search_detection_range, self.finish_marker_size]
        if min(positive) <= 0 or self.max_vy < 0 or self.safety_margin < 0:
            raise ValueError("Invalid positive geometry/timing/speed limits")
        if not 0 < self.return_reserve < self.mission_seconds:
            raise ValueError("Return reserve must be inside mission budget")
        if not isinstance(self.target_quota, int) or self.target_quota < 0:
            raise ValueError("target_quota must be a nonnegative declared count")
        if not isinstance(self.course_route, str):
            raise ValueError("course_route must be a path string")
        if not isinstance(self.search_mission, bool):
            raise ValueError("search_mission must be boolean")
        if not 0 < self.search_coverage_threshold <= 1:
            raise ValueError("search_coverage_threshold must be in (0,1]")
        bounds = np.asarray(self.search_bounds, dtype=float)
        if bounds.size and (bounds.shape != (4,) or not np.isfinite(bounds).all() or
                            not (bounds[0] < bounds[1] and bounds[2] < bounds[3])):
            raise ValueError("search_bounds must be finite [xmin,xmax,ymin,ymax]")
        approach = np.asarray(self.search_finish_approach, dtype=float)
        if approach.size and (approach.ndim != 2 or approach.shape[1] != 2 or not np.isfinite(approach).all()):
            raise ValueError("search_finish_approach must contain finite map-frame xy pairs")
        if self.search_finish_gate_yaw is not None and not np.isfinite(self.search_finish_gate_yaw):
            raise ValueError("search_finish_gate_yaw must be finite or null")
        if self.search_finish_gate_index is not None and (not isinstance(self.search_finish_gate_index,int) or
                not 0 <= self.search_finish_gate_index < len(self.search_finish_approach)):
            raise ValueError("search_finish_gate_index must identify an approach point")
        if self.course_portal_resolution != 0. and not (0 < self.course_portal_resolution <= self.resolution
                                                       and self.map_size/self.course_portal_resolution <= 1200):
            raise ValueError("course_portal_resolution must be zero or a supported finer map resolution")
        if not isinstance(self.imu_heading_reference, bool):
            raise ValueError("imu_heading_reference must be boolean")
        if not isinstance(self.allow_margin_escape, bool):
            raise ValueError("allow_margin_escape must be boolean")
        if not 0 < self.max_slope_deg < 45 or not 0 < self.max_step_height < self.body_clearance or not 0 < self.ground_max_gap <= .6:
            raise ValueError("Invalid terrain slope/step/interpolation limits")
        if not isinstance(self.terrain_slope_regions, list):
            raise ValueError("Local slope regions must be a list")
        for region in self.terrain_slope_regions:
            if not isinstance(region, dict):
                raise ValueError("Invalid local slope region")
            bounds = np.asarray(region.get("bounds", []), dtype=float)
            slope = region.get("max_slope_deg", 0.)
            if bounds.shape != (4,) or not np.isfinite(bounds).all() or not (bounds[0] < bounds[1] and bounds[2] < bounds[3]) or not np.isfinite(slope) or not 0 < slope < 45:
                raise ValueError("Invalid local slope region")
        if min(self.footprint_half_length, self.footprint_half_width) < 0 or bool(self.footprint_half_length) != bool(self.footprint_half_width):
            raise ValueError("Footprint dimensions must both be positive, or both zero for circular mode")
        if min(self.hazard_foot_half_length, self.hazard_foot_half_width) < 0 or bool(self.hazard_foot_half_length) != bool(self.hazard_foot_half_width):
            raise ValueError("Hazard contact envelope dimensions must both be set")
        low, high = np.asarray(self.water_hsv_lower), np.asarray(self.water_hsv_upper)
        if low.shape != (3,) or high.shape != (3,) or not np.isfinite(low).all() or not np.isfinite(high).all() or (low < 0).any() or (high > [179, 255, 255]).any() or (low > high).any():
            raise ValueError("Invalid water HSV cue")
        if self.localization_backend not in {"icp", "kiss"}:
            raise ValueError("localization_backend must be icp or kiss")
        if self.localization_metric not in {"point_to_point", "point_to_line"}:
            raise ValueError("localization_metric must be point_to_point or point_to_line")
        if self.map_size / self.resolution > 1200 or self.map_size / self.resolution < 20:
            raise ValueError("Map size/resolution exceeds supported bounds")
        k, t = np.asarray(self.camera_matrix), np.asarray(self.body_from_camera)
        if k.shape != (3, 3) or t.shape != (4, 4) or not np.isfinite(k).all() or not np.isfinite(t).all():
            raise ValueError("Invalid camera matrix/extrinsics")
        if min(k[0, 0], k[1, 1]) <= 0 or not np.allclose(t[3], [0, 0, 0, 1]):
            raise ValueError("Invalid camera focal length or homogeneous transform")
        if not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5) or not math.isclose(np.linalg.det(t[:3, :3]), 1., abs_tol=1e-5):
            raise ValueError("Camera extrinsics must contain a proper rotation")
        lidar = np.asarray(self.body_from_lidar)
        if lidar.shape != (4, 4) or not np.isfinite(lidar).all() or not np.allclose(lidar[3], [0, 0, 0, 1]):
            raise ValueError("Invalid lidar homogeneous transform")
        if not np.allclose(lidar[:3, :3].T @ lidar[:3, :3], np.eye(3), atol=1e-5) or not math.isclose(np.linalg.det(lidar[:3, :3]), 1., abs_tol=1e-5):
            raise ValueError("Lidar extrinsics must contain a proper rotation")
        if len(self.camera_size) != 2 or min(self.camera_size) <= 0:
            raise ValueError("Invalid calibrated image size")
        if not 0 <= self.min_pose_quality <= 1 or not 0 <= self.min_ball_confidence <= 1:
            raise ValueError("Confidence thresholds outside [0,1]")
        if self.ball_confirmations < 2 or self.min_points < 6:
            raise ValueError("Insufficient confirmation/point count")

    def slope_limit(self, xy):
        xy = np.asarray(xy)
        limits = np.full(xy.shape[:-1], self.max_slope_deg)
        for region in self.terrain_slope_regions:
            x0, x1, y0, y1 = region["bounds"]
            inside = (xy[..., 0] >= x0)&(xy[..., 0] <= x1)&(xy[..., 1] >= y0)&(xy[..., 1] <= y1)
            limits = np.where(inside, np.maximum(limits, region["max_slope_deg"]), limits)
        return limits

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")
