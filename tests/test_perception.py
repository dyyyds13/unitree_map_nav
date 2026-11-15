import cv2
import numpy as np
import pytest
from pathlib import Path

from go2_rescue.config import Config
from go2_rescue.perception import Perception, ransac_arc
from go2_rescue.synthetic import football_image
from go2_rescue.tracking import Tracker
from go2_rescue.types import BallObservation, Frame, Pose


def test_ransac_recovers_partial_arc_with_outliers():
    rng = np.random.default_rng(4)
    angles = np.linspace(.3, 3.8, 100)
    points = [40 + 22 * np.cos(angles), 60 + 22 * np.sin(angles)]
    points = np.column_stack(points) + rng.normal(0, .2, (100, 2))
    points = np.vstack([points, rng.uniform(15, 85, (30, 2))])
    center, radius = ransac_arc(points)
    assert np.linalg.norm(center - [40, 60]) < 1.
    assert abs(radius - 22) < 1.


@pytest.mark.parametrize("occlusion", [0., .2, .4, .6])
def test_geometry_detects_synthetic_black_white_ball_under_occlusion(occlusion):
    c = Config()
    detections = Perception(c).detect_balls(football_image(c, occlusion=occlusion))
    assert any(np.linalg.norm(np.array(d.center) - [180, 165]) < 5 for d in detections)


def test_uniform_circles_and_blank_images_are_not_footballs():
    c = Config()
    perception = Perception(c)
    image = np.full((240, 320, 3), 112, np.uint8)
    assert perception.detect_balls(image) == []
    cv2.circle(image, (160, 160), 22, (230, 230, 230), -1)
    assert perception.detect_balls(image) == []


def test_shadowed_white_patches_do_not_require_absolute_165_brightness():
    config = Config()
    shadowed = (football_image(config).astype(float) * .55).astype(np.uint8)
    assert shadowed.max() < 165
    assert Perception(config).detect_balls(shadowed)


def test_wall_sized_arc_is_not_reported_as_a_football():
    config = Config()
    perception = Perception(config)
    detection = BallObservation((120., 140.), 38., .9, .8)
    frame = Frame(0., np.empty((0, 3)))
    assert perception.locate_ball(detection, frame, Pose()).position is None


def test_ground_projection_uses_fitted_center_not_occluded_width():
    c = Config()
    p = Perception(c)
    frame = Frame(0., np.empty((0, 3)))
    a = BallObservation((160., 175.), 12., .9, .5)
    b = BallObservation((160., 175.), 25., .9, .9)
    p.locate_ball(a, frame, Pose(sigma=.05))
    p.locate_ball(b, frame, Pose(sigma=.05))
    assert a.position is not None and np.allclose(a.position, b.position)
    horizon = BallObservation((160., 110.), 20., .9, .5)
    assert p.locate_ball(horizon, frame, Pose()).position is None


def test_tracker_reports_once_requires_time_and_distinct_frames():
    c, pose = Config(), Pose(quality=1., sigma=.04)
    tracker = Tracker(c)
    events = []
    for i in range(12):
        detection = BallObservation((160, 160), 20, .92, .9, np.array([2. + i * .005, 1.]), .12)
        events.extend(tracker.update([detection], pose, i * .1))
    assert len(events) == 1 and events[0].kind == "target"
    tracker.update([], pose, 50.)
    events = []
    for i in range(8):
        detection = BallObservation((160, 160), 20, .92, .9, np.array([2., 1.]), .12)
        events.extend(tracker.update([detection], pose, 50 + i * .1))
    assert events == []


@pytest.mark.parametrize("quota,expected", [(0, 2), (1, 1), (2, 2)])
def test_declared_target_count_caps_reports_without_disabling_tracking(quota, expected):
    tracker = Tracker(Config(target_quota=quota))
    events = []
    for timestamp in [0., .2, .4, .6]:
        observations = [BallObservation((160, 160), 20, .95, .9, np.array(xy), .1)
                        for xy in [[1., 0.], [3., 1.]]]
        events.extend(tracker.update(observations, Pose(quality=1.), timestamp))
    assert len(events) == len(tracker.reported_positions) == expected
    assert len(tracker.tracks) == 2 and all(t.reported for t in tracker.tracks)


def test_weak_partial_arc_requires_viewpoint_change():
    tracker = Tracker(Config())
    events = []
    for i in range(8):
        detection = BallObservation((160, 160), 20, .7, .4, np.array([2., 0.]), .12)
        events.extend(tracker.update([detection], Pose(quality=1), i * .1))
    assert not events
    events = tracker.update([detection], Pose(y=.2, quality=1), .8)
    assert not events  # Translating past a rock cannot confirm a weak circle.
    detection.confidence = .9
    assert not tracker.update([detection], Pose(y=.2, quality=1), .9)
    events = tracker.update([detection], Pose(y=.2, quality=1), 1.)
    assert len(events) == 1


@pytest.mark.parametrize("name", ["rubble-arc", "wall-arc"])
def test_recorded_course_distractors_never_confirm_from_repeated_weak_arcs(name):
    image = cv2.imread(str(Path(__file__).parent / "fixtures" / f"{name}.png"))
    candidates = Perception(Config()).detect_balls(image)
    assert candidates and max(d.confidence for d in candidates) < .8
    tracker = Tracker(Config())
    for i in range(15):
        # Even optimistically assuming perfect, constant map position and a
        # translating observer must not turn the actual false arc into a report.
        for detection in candidates:
            detection.position, detection.sigma = np.array([2., 1.]), .12
        assert not tracker.update(candidates, Pose(y=.02*i, quality=1.), .1*i)


def test_visible_marker_pose_and_wrong_marker_rejection():
    c = Config()
    perception = Perception(c)
    image = np.full((240, 320, 3), 255, np.uint8)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 42, 52)
    image[94:146, 134:186] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    result = perception.finish(image, Frame(0., np.empty((0, 3))), Pose())
    assert result is not None
    marker_xy, goal = result
    assert np.isfinite(marker_xy).all() and .5 < np.linalg.norm(marker_xy) < 2.
    assert abs(np.linalg.norm(marker_xy - goal) - c.finish_standoff) < .01
    assert perception.detect_balls(image) == []
    c.finish_marker_id = 41
    assert perception.finish(image, Frame(0., np.empty((0, 3))), Pose()) is None
