"""Software ordering/safety regressions; not full physical course evidence."""
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from go2_rescue.config import Config
from go2_rescue.controller import Autonomy
from go2_rescue.course import OrderedCourse, STAGES
from go2_rescue.mapping import GridMap
from go2_rescue.planning import astar
from go2_rescue.planning import Explorer
from go2_rescue.synthetic import SensorWorld
from go2_rescue.types import Frame, Pose, Velocity


def test_margin_exit_completes_without_chasing_millimetre_subgoals():
    # Kinematic controller regression only, not physical/full-course evidence.
    c = Config(course_route="configs/official-course-route.json", map_size=6., resolution=.05,
               footprint_half_length=.35, footprint_half_width=.23, robot_radius=.23,
               safety_margin=.05, allow_margin_escape=True)
    autonomy = Autonomy(c)
    g = autonomy.grid
    g.seen[:] = True
    g.logodds[:] = -4.
    g.floor[:] = 0.
    ys = g.origin[1]+(np.arange(g.n)+.5)*c.resolution
    g.logodds[:, ys >= .80] = 4.
    pose, velocity = Pose(0., .55, 0., quality=1.), Velocity()
    seen = False
    for tick in range(40):
        observation = Frame(tick*.1, np.empty((0, 3)), np.array([1., 0., 0., 0.]), velocity)
        result = autonomy.ordered_step(observation, pose, np.empty((0, 3)), [], .1)
        seen |= result.reason == "ordered_margin_escape"
        if seen and result.reason != "ordered_margin_escape":
            assert tick*.1 < 3. and not autonomy.margin_escape_active
            assert pose.y < .52
            break
        assert abs(result.velocity.vx) <= .05 and abs(result.velocity.vy) <= .05
        pose.x += result.velocity.vx*.1
        pose.y += result.velocity.vy*.1
        pose.yaw += result.velocity.wz*.1
        velocity = result.velocity
    else:
        pytest.fail("Short safe exit stalled on interpolation subgoals")


def route_file(tmp_path):
    value = {"schema": "go2-ordered-course/1", "frame": "map_start",
             "stages": [{"name": name, "waypoints": [{"xy": [i+.5, 0.]}]} for i, name in enumerate(STAGES)]}
    path = tmp_path / "route.json"
    path.write_text(json.dumps(value))
    return path


def test_checkpoint_turn_does_not_sweep_past_the_required_heading(tmp_path):
    path = route_file(tmp_path)
    definition = json.loads(path.read_text())
    definition["stages"][0]["waypoints"] = [{"xy": [0., 0.], "yaw": .30, "strict": True}]
    path.write_text(json.dumps(definition))
    c = Config(course_route=str(path), map_size=3., resolution=.01,
               footprint_half_length=.35, footprint_half_width=.23, safety_margin=.05)
    autonomy = Autonomy(c)
    g = autonomy.grid
    g.seen[:] = True
    g.floor[:] = 0.
    g.logodds[:] = -4.
    ys = g.origin[1]+(np.arange(g.n)+.5)*c.resolution
    g.logodds[:, ys >= .40] = 4.
    # Required .30rad fits; the former .45rad swept endpoint hits the wall.
    assert autonomy.dwa.safe_turn(Pose(), .30, g) == Velocity()
    result = autonomy.ordered_step(frame(0.), Pose(), np.empty((0, 3)), [], .1)
    assert 0 < result.velocity.wz <= .20+1e-9
    assert result.reason == "ordered_align_checkpoint"


def frame(t, pitch=0., points=None):
    return Frame(t, np.empty((0, 3)) if points is None else points,
                 np.roll(Rotation.from_euler("y", pitch).as_quat(), 1))


def test_default_prior_contains_all_stages_no_target_coordinates():
    path = Path(__file__).resolve().parents[1] / "configs/official-course-route.json"
    course = OrderedCourse(path)
    assert [s["name"] for s in course.stages] == STAGES
    assert course.stage == "L1_ascent" and not course.route_complete
    assert course.definition["pipe_required"] == "tunnel2"
    assert "football_world_xyz" not in path.read_text()
    assert any(w.get("inspect_front") for s in course.stages for w in s["waypoints"])
    assert sum(w.get("reverse", False) for s in course.stages for w in s["waypoints"]) >= 2


def test_pipe2_stage_stops_if_configured_locomotion_cannot_cross_its_curb():
    autonomy = Autonomy(Config(course_route="configs/official-course-route.json"))
    autonomy.course.stage_index = STAGES.index("pipe")
    result = autonomy.ordered_step(frame(0.), Pose(), np.empty((0,3)), [], .1)
    assert result.state == "blocked"
    assert result.reason == "ordered_pipe2_requires_validated_step_capability"
    assert not result.velocity.array().any()
    assert autonomy.course.stage == "pipe" and not autonomy.course.route_complete


def test_local_entry_route_allows_ramp_but_guards_unvalidated_exit():
    autonomy = Autonomy(Config(course_route="configs/local-entry-ramp-route.json"))
    autonomy.course.stage_index = STAGES.index("pipe")
    entry = autonomy.ordered_step(frame(0.), Pose(), np.empty((0,3)), [], .1)
    assert entry.reason != "ordered_pipe2_requires_validated_step_capability"
    assert autonomy.c.slope_limit([7.2,5.42]) == 26.
    assert autonomy.c.slope_limit([0.,0.]) == autonomy.c.max_slope_deg
    autonomy.course.waypoint_index = len(autonomy.course.stages[5]["waypoints"])-1
    result = autonomy.ordered_step(frame(1.), Pose(), np.empty((0,3)), [], .1)
    assert result.state == "blocked" and not result.velocity.array().any()
    assert result.reason == "ordered_pipe2_exit_requires_validated_descent_capability"


def test_observed_straight_corridor_uses_full_body_support_without_disk_overinflation(tmp_path):
    path=route_file(tmp_path);definition=json.loads(path.read_text())
    definition['stages'][5]['waypoints']=[{'xy':[1.2,0.],'yaw':0.,'narrow_passage':True,'observed_corridor':True}]
    path.write_text(json.dumps(definition));course=OrderedCourse(path);course.stage_index=5
    c=Config(map_size=4.,resolution=.025,footprint_half_length=.35,footprint_half_width=.23,safety_margin=.05)
    g=GridMap(c);g.seen[:]=True;g.floor[:]=0.;g.logodds[:]=-4.
    ys=g.origin[1]+(np.arange(g.n)+.5)*c.resolution
    g.logodds[:,abs(ys)>=.275]=4.
    assert not g.traversable()[g.cell([0.,0.])]
    assert len(course.local_path(g,Pose()))>1
    # The nominal goal can sit just outside the measured corridor. Reaching
    # its safe arrival region may require only a10cm advance, not25cm.
    course.waypoint['xy']=[.15,-.15]
    short=course.local_path(g,Pose())
    assert len(short)>1 and np.linalg.norm(short[-1])<.25
    assert np.linalg.norm(short[-1]-course.target)<.20
    course.waypoint['xy']=[1.2,0.]
    # A real wall across the passage still blocks every observed advance.
    g.logodds[g.cell([.6,0.])[0],:]=4.
    g._free_cache=g._surface_safe=None
    assert len(course.local_path(g,Pose()))==0


def test_corridor_plans_from_scan_motion_instead_of_stance_velocity_alias(tmp_path):
    path=route_file(tmp_path);definition=json.loads(path.read_text())
    definition['stages'][5]['waypoints']=[{'xy':[1.,.2],'yaw':0.,'strict':True,
        'narrow_passage':True,'observed_corridor':True,'max_forward_mps':.22}]
    path.write_text(json.dumps(definition))
    a=Autonomy(Config(course_route=str(path),map_size=5.,resolution=.05,
        footprint_half_length=.35,footprint_half_width=.23,max_vy=.05,passage_max_vx=.5))
    a.course.stage_index=5;g=a.grid;g.seen[:]=True;g.floor[:]=0.;g.logodds[:]=-4.
    for t in [0.,.1,.2,.3,.4]:
        a.course.motion_window.append((t,np.zeros(2),np.array([0.,-.15,0.])))
    observation=frame(.4);observation.velocity=Velocity(0.,-.15,0.)
    result=a.ordered_step(observation,Pose(quality=1.),np.empty((0,3)),[],.1)
    assert result.velocity.vx>0. and result.velocity.vy>0.


def test_corridor_terrain_submap_keeps_unknowns_and_measured_obstacles_blocked(tmp_path):
    path=route_file(tmp_path);definition=json.loads(path.read_text())
    definition['stages'][5]['waypoints']=[{'xy':[1.2,0.],'yaw':0.,'strict':True,
        'narrow_passage':True,'observed_corridor':True}]
    path.write_text(json.dumps(definition))
    a=Autonomy(Config(course_route=str(path),map_size=6.,resolution=.05,base_height=.30))
    a.course.stage_index=5
    old=a.grid;old.floor[:]=.7;old.seen[:]=True;old.logodds[:]=-4.
    old.logodds[old.cell([2.5,0.])]=4.
    x,y=np.meshgrid(np.arange(-.6,1.11,.05),np.arange(-.6,.61,.05))
    points=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.30)])
    # A real low obstacle remains in the new observations and must still block.
    points=np.vstack([points,[.7,0.,-.1]])
    a.localization.update=lambda f:(Pose(quality=1.),points)
    result=a.step(frame(.6,points=points))
    assert a.corridor_global_grid is old and old.logodds[old.cell([2.5,0.])]==4.
    assert a.grid.floor[a.grid.cell([.4,0.])]<.08
    assert not a.grid.traversable()[a.grid.cell([2.5,0.])]  # Unknown locally, not invented free.
    assert not a.grid.traversable()[a.grid.cell([.7,0.])]  # Current real obstacle.
    assert any(e.kind=='corridor_terrain_submap_started' for e in result.events)


def test_future_finish_pose_cannot_skip_the_first_stage(tmp_path):
    course = OrderedCourse(route_file(tmp_path))
    events = course.update(frame(0.), Pose(6.5, 0.), 0.)
    assert course.stage == "L1_ascent" and course.waypoint_index == 0
    assert not any(e.kind == "course_stage_complete" for e in events)


def test_declared_broad_transit_gate_accepts_safe_side_pass_but_is_bounded(tmp_path):
    path = route_file(tmp_path)
    definition = json.loads(path.read_text())
    definition["stages"][2]["waypoints"][0]["gate_half_width"] = .6
    path.write_text(json.dumps(definition))
    for offset, should_pass in [(.5, True), (.65, False)]:
        course = OrderedCourse(path)
        course.stage_index = 2  # Isolated logic fixture, not a physical run.
        course.update(frame(0.), Pose(1.5, 0.), 0.)
        course.update(frame(.1), Pose(2.55, offset), 0.)
        assert (course.stage == "dead_end") == should_pass


def test_ramp_checkpoints_require_sensor_height_and_pitch_evidence(tmp_path):
    course = OrderedCourse(route_file(tmp_path))
    course.update(frame(0.), Pose(), 0.)
    course.update(frame(.1), Pose(.5), 0.)
    assert course.stage == "L1_ascent" and course.diagnostics()["awaiting_sensor_evidence"]
    events = course.update(frame(.2, -.15), Pose(.5), .35)
    assert [e.payload["stage"] for e in events if e.kind == "course_stage_complete"] == ["L1_ascent"]
    course.update(frame(.3), Pose(.5), .35)
    course.update(frame(.4), Pose(1.5), .35)
    assert course.stage == "L1_descent"
    course.update(frame(.5, .15), Pose(1.5), .02)
    assert course.stage == "S_curve"


def test_dead_end_requires_front_confirmation_hold_and_return(tmp_path):
    path = route_file(tmp_path)
    d = json.loads(path.read_text())
    d["stages"][3]["waypoints"] = [{"xy": [0., 0.], "yaw": 0., "inspect_front": True, "hold_seconds": .5},
                                      {"xy": [0., 1.], "yaw": 0., "reverse": True}]
    path.write_text(json.dumps(d))
    course = OrderedCourse(path)
    course.stage_index = 3  # Isolated unit fixture, not a mission execution.
    course.update(frame(0.), Pose(), 0.)
    assert course.waypoint_index == 0
    points = np.column_stack([np.full(8, .7), np.linspace(-.3, .3, 8), np.zeros(8)])
    for t in np.arange(.1, .91, .1):
        course.update(frame(float(t), points=points), Pose(), 0.)
    assert course.waypoint_index == 1 and course.stage == "dead_end"
    course.update(frame(1.), Pose(0., 1.), 0.)
    assert course.stage == "rubble"


@pytest.mark.parametrize("moving", [False, True])
def test_inspection_averages_gait_jitter_but_rejects_continuous_motion(tmp_path, moving):
    path = route_file(tmp_path)
    d = json.loads(path.read_text())
    d["stages"][3]["waypoints"] = [{"xy": [0., 0.], "yaw": 0., "inspect_front": True,
                                   "hold_seconds": .5}, {"xy": [0., 1.], "reverse": True}]
    path.write_text(json.dumps(d))
    course = OrderedCourse(path)
    course.stage_index = 3
    points = np.column_stack([np.full(8, .7), np.linspace(-.3, .3, 8), np.zeros(8)])
    events = []
    for i in range(12):
        observation = frame(i*.1, points=points)
        observation.velocity = Velocity(.09 if moving else (-1)**i*.10)
        events.extend(course.update(observation, Pose(), 0.))
    assert sum(e.kind == "dead_end_confirmed" for e in events) == 1
    assert (course.waypoint_index == 1) == (not moving)


@pytest.mark.parametrize("jitter,completes", [(.002, True), (.04, False)])
def test_inspection_hold_survives_small_jitter_but_rejects_large_motion(tmp_path, jitter, completes):
    path = route_file(tmp_path)
    d = json.loads(path.read_text())
    d["stages"][3]["waypoints"] = [{"xy": [0., 0.], "tolerance": .25, "yaw": 0.,
        "inspect_front": True, "hold_seconds": .5}, {"xy": [0., 1.], "reverse": True}]
    path.write_text(json.dumps(d))
    course = OrderedCourse(path)
    course.stage_index = 3
    points = np.column_stack([np.full(8, .7), np.linspace(-.3, .3, 8), np.zeros(8)])
    for i in range(10):
        x = .249 if i < 4 else .249+(-1)**i*jitter
        course.update(frame(i*.1, points=points), Pose(x, 0.), 0.)
    assert (course.waypoint_index == 1) == completes


def test_seeing_target_or_reaching_time_reserve_cannot_bypass_order(tmp_path):
    c = Config(map_size=10., course_route=str(route_file(tmp_path)), mission_seconds=2., return_reserve=1.)
    world, autonomy = SensorWorld(c, mission_fixture=True), Autonomy(c)
    autonomy.tracker.reported_positions.append(np.array([1., 0.]))
    autonomy.finish_goal = np.array([.3, 0.])
    result = autonomy.step(world.frame())
    assert result.state in {"ordered_route", "recovering"}
    assert autonomy.course.stage == "L1_ascent"
    assert not any(e.kind in {"target", "finish"} for e in result.events)


def test_unseen_checkpoint_does_not_clear_map_or_issue_an_unobserved_path(tmp_path):
    path = route_file(tmp_path)
    d = json.loads(path.read_text()); d["stages"][0]["waypoints"][0]["xy"] = [8., 0.]
    path.write_text(json.dumps(d))
    course = OrderedCourse(path)
    c = Config(map_size=10., robot_radius=.23, safety_margin=.05,
               footprint_half_length=.35, footprint_half_width=.23)
    grid = GridMap(c)
    x, y = np.meshgrid(np.arange(-1, 2, .075), np.arange(-1, 1, .075))
    grid.update(np.column_stack([x.ravel(), y.ravel(), np.full(x.size, -c.base_height)]), Pose())
    before = grid.logodds.copy()
    points = course.local_path(grid, Pose())
    assert len(points) and np.linalg.norm(points[-1]) < 1.35
    assert all(grid.traversable()[grid.cell(p)] for p in points)
    assert np.array_equal(before, grid.logodds)


def test_clearance_cost_can_choose_a_longer_safer_path():
    free = np.ones((5, 7), bool)
    costs = np.ones_like(free, float); costs[2, 1:6] = 8.
    path = astar(free, (2, 0), (2, 6), costs)
    assert any(x != 2 for x, y in path)
    assert all(x == 2 for x, y in astar(free, (2, 0), (2, 6)))
    with pytest.raises(ValueError):
        astar(free, (2, 0), (2, 6), np.zeros_like(costs))


def test_missing_or_reordered_stage_is_rejected(tmp_path):
    path = route_file(tmp_path)
    d = json.loads(path.read_text());d["stages"] = d["stages"][1:]
    path.write_text(json.dumps(d))
    with pytest.raises(ValueError):
        OrderedCourse(path)


def test_transit_gate_accepts_safe_side_detour_without_skipping_inspection(tmp_path):
    path = route_file(tmp_path)
    course = OrderedCourse(path)
    course.update(frame(0.), Pose(), 0.)
    events = course.update(frame(.1), Pose(.75, .2, 1.), 0.)
    assert course.waypoint_index == 1  # First point passed beside, with heading free.
    assert any(e.payload.get("method") == "passed_gate" for e in events)
    assert course.stage == "L1_ascent"  # Elevation evidence is still mandatory.


def test_margin_escape_restores_buffer_without_relaxing_physical_or_water_checks():
    c = Config(map_size=10., robot_radius=.23, safety_margin=.05,
               footprint_half_length=.35, footprint_half_width=.23, allow_margin_escape=True,
               hazard_foot_half_length=.28, hazard_foot_half_width=.22)
    grid = GridMap(c)
    x, y = np.meshgrid(np.arange(-2., 2., .075), np.arange(-2., 2., .075))
    grid.update(np.column_stack([x.ravel(), y.ravel(), np.full(x.size, -c.base_height)]), Pose())
    column = grid.cell([-.45, 0.])[0]
    grid.logodds[column, :] = 4.
    grid._free_cache = grid._surface_safe = None
    pose = Pose(-.025, 0.)
    route = Explorer().margin_escape(grid, pose)
    assert len(route) > 1
    x, y = np.meshgrid(np.linspace(-.35, .35, 11), np.linspace(-.23, .23, 9))
    body = np.column_stack([x.ravel(), y.ravel()])
    x, y = np.meshgrid(np.linspace(-.4, .4, 11), np.linspace(-.28, .28, 9))
    buffer = np.column_stack([x.ravel(), y.ravel()])
    assert grid.footprints_safe(route[:, :2], route[:, 2], body).all()
    assert grid.footprints_safe(route[-1:, :2], route[-1:, 2], buffer)[0]
    assert not len(Explorer().margin_escape(grid, Pose(-.15, 0.)))  # Physical overlap never allowed.
    hazard = np.array([[.05, -.08], [.2, -.08], [.2, .08], [.05, .08]])
    grid.add_hazard(hazard); grid.add_hazard(hazard)
    assert not len(Explorer().margin_escape(grid, pose))  # Water envelope is not shrunk.
    assert not Config().allow_margin_escape  # Unknown hardware keeps its original boundary.
