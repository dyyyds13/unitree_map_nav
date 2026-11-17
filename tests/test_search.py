"""Search semantics and geometry checks; reset poses below are scorer fixtures."""
from pathlib import Path

import numpy as np
import pytest

from go2_rescue.config import Config
from go2_rescue.controller import Autonomy
from go2_rescue.mapping import GridMap
from go2_rescue.search import CoverageSearch, ObservedGoal
from go2_rescue.types import BallObservation, Event, Frame, Pose, Velocity

ROOT = Path(__file__).resolve().parents[1]


def flat_map():
    config = Config(map_size=12., search_mission=True)
    grid = GridMap(config)
    grid.seen[:] = True
    grid.floor[:] = 0.
    grid.logodds[:] = -4.
    return config, grid


def test_coverage_uses_fixed_public_area_including_obstacles_not_known_map_size():
    config,grid = flat_map()
    config.search_bounds = [-5.,5.,-5.,5.]
    search = CoverageSearch(config)
    grid.seen[:] = False
    # Known cells outside the field cannot improve the fraction.
    grid.seen[:10,:] = True
    assert search.coverage(grid) == (0.,0.,100.)
    grid.seen[10:60,10:110] = True
    grid.floor[:] = np.nan
    grid.logodds[:] = 4.  # Observed obstacle is mapped too.
    assert search.coverage(grid) == pytest.approx((.5,50.,100.))
    grid.logodds[:] = 0. # Overhead-only/unclassified observations do not count.
    assert search.coverage(grid)[0] == 0.
    config.search_bounds = []
    assert search.coverage(grid)[0] is None


def test_coverage_threshold_latches_return_without_target_quota_or_panorama():
    config,grid = flat_map()
    config.search_bounds = [-5.,5.,-5.,5.]
    config.search_finish_approach = [[3.,0.]]
    a = Autonomy(config)
    a.grid,a.started = grid,0.
    a.explorer.choose = lambda *args,**kwargs: np.array([1.,0.])
    grid.seen[:] = False
    cells = np.array([(i,j) for i in range(10,110) for j in range(10,110)])
    grid.seen[tuple(cells[:8990].T)] = True
    frame = Frame(1.,np.empty((0,3)))
    target = Event('target',1.,{'position':[1.,1.]})
    result = a.search.command(a,frame,Pose(quality=1.),frame.points,[target],.1)
    assert not a.search.returning and target in result.events
    assert result.state != 'search_scan' and result.reason != 'camera_panorama'
    grid.seen[tuple(cells[:9000].T)] = True
    frame.timestamp = 2.
    result = a.search.command(a,frame,Pose(quality=1.),frame.points,[],.1)
    event = next(e for e in result.events if e.kind=='search_return')
    assert event.payload['reason'] == 'map_coverage_threshold'
    assert event.payload['map_coverage_fraction'] == pytest.approx(.9)
    np.testing.assert_array_equal(a.search.goal,[3.,0.])
    grid.seen[:] = False  # Later corrections cannot restart exploration.
    result = a.search.command(a,frame,Pose(quality=1.),frame.points,[],.1)
    assert a.search.returning and not any(e.kind=='search_return' for e in result.events)


def test_no_frontier_below_threshold_waits_without_spinning_or_claiming_coverage():
    config,grid = flat_map()
    config.search_bounds = [-5.,5.,-5.,5.]
    grid.seen[:] = False
    a = Autonomy(config)
    a.grid,a.started = grid,0.
    a.explorer.choose = lambda *args,**kwargs: None
    for now in [1.,15.]:
        frame = Frame(now,np.empty((0,3)))
        result = a.search.command(a,frame,Pose(quality=1.),frame.points,[],.1)
        assert not a.search.returning
        assert result.velocity == Velocity() and result.reason=='mapping_no_reachable_frontier'
    frame.timestamp = config.mission_seconds-config.return_reserve
    result = a.search.command(a,frame,Pose(quality=1.),frame.points,[],.1)
    assert any(e.kind=='search_return' and e.payload['reason']=='return_time_reserve'
               and not e.payload['coverage_threshold_reached'] for e in result.events)


@pytest.mark.parametrize('value',[0.,1.01,float('nan')])
def test_invalid_mapping_threshold_is_rejected(value):
    with pytest.raises(ValueError):
        Config(search_coverage_threshold=value).validate()


def test_search_rejects_far_ground_ray_hypotheses_but_keeps_near_balls():
    from go2_rescue.perception import Perception
    c = Config()
    def observation(distance):
        point = np.array([distance,0.,c.ball_radius-c.base_height])
        transform,k = np.array(c.body_from_camera),np.array(c.camera_matrix)
        optical = transform[:3,:3].T@(point-transform[:3,3])
        pixel = k@optical/optical[2]
        return BallObservation(tuple(pixel[:2]),k[0,0]*c.ball_radius/optical[2],.95,.9)
    frame,pose = Frame(1.,np.empty((0,3))),Pose(quality=1.,sigma=.03)
    assert Perception(c).locate_ball(observation(6.),frame,pose).position is not None
    c.search_mission = True
    assert Perception(c).locate_ball(observation(6.),frame,pose).position is None
    assert Perception(c).locate_ball(observation(2.),frame,pose).position is not None


def test_loop_correction_preserves_goal_timeout_while_transforming_goal():
    config,grid = flat_map()
    search = CoverageSearch(config)
    search.goal = np.array([1.,0.])
    search.goal_since = 3.
    search.correct_map(grid,[(Pose(),Pose(x=.5))])
    np.testing.assert_allclose(search.goal,[1.5,0.])
    assert search.goal_since == 3.


def test_loop_correction_does_not_move_public_finish_approach_priors():
    config,grid = flat_map()
    search = CoverageSearch(config)
    search.returning = True
    search.goal = np.array([1.,0.])
    search.correct_map(grid,[(Pose(),Pose(x=.5))])
    assert search.goal is None  # Next command reloads unchanged public prior.


def test_direction_prior_never_creates_support_across_unknown_gap():
    config, grid = flat_map()
    gap = grid.cell([1.,0.])[0]
    grid.seen[gap:gap+8,:] = False
    grid.floor[gap:gap+8,:] = np.nan
    path = ObservedGoal([3.,0.]).local_path(grid,Pose())
    assert not len(path) or path[:,0].max() < 1.
    assert not grid.seen[gap:gap+8,:].any()


def test_declared_exit_uses_full_oriented_body_without_disk_deadlock():
    config,grid = flat_map()
    config.map_size,config.resolution = 6.,.02
    config.footprint_half_length,config.footprint_half_width = .35,.23
    config.safety_margin = .05
    grid = GridMap(config)
    grid.seen[:] = True
    grid.logodds[:] = -4.
    columns = grid.origin[1]+(np.arange(grid.n)+.5)*config.resolution
    grid.floor[:,abs(columns)<.28] = 0.
    grid.logodds[:,abs(columns)>=.28] = 4.
    normal = ObservedGoal([1.2,0.])
    assert not len(normal.local_path(grid,Pose()))
    normal.waypoint = {'oriented_passage':True,'yaw':0.}
    assert len(normal.local_path(grid,Pose()))
    assert np.isnan(grid.floor[grid.cell([0.,1.])])


def test_finish_visibility_does_not_end_search_and_timeout_is_not_finish():
    config, grid = flat_map()
    a = Autonomy(config)
    a.grid, a.started = grid, 0.
    a.finish_goal, a.finish_marker = np.array([0.,0.]),np.array([.65,0.])
    a.finish_observations, a.last_finish = 20,1.
    frame = Frame(1.,np.empty((0,3)))
    result = a.search.command(a,frame,Pose(quality=1.),frame.points,[],.1)
    assert result.state != 'finished'
    assert not any(e.kind=='finish' for e in result.events)
    assert config.target_quota == 0


def test_search_releases_ordered_goal_ownership_after_dead_end():
    config, grid = flat_map()
    config.course_route = str(ROOT/'configs/local-entry-ramp-route.json')
    a = Autonomy(config)
    a.grid = grid
    a.course.stage_index = 4
    a.course.waypoint_index = 2
    a.course.completed = ['L1_ascent','L1_descent','S_curve','dead_end']
    # State-machine fixture, not physical stage completion evidence.
    a.course.update = lambda *args: []
    points = np.array([[x,y,-config.base_height] for x in np.arange(-1.,1.,.1) for y in np.arange(-1.,1.,.1)])
    a.localization.update = lambda frame: (Pose(quality=1.),points)
    result = a.step(Frame(.6,points))
    assert a.course is None
    assert any(e.kind=='search_started' and not e.payload['pipe_required'] for e in result.events)
    assert result.state != 'finished'


def test_visible_finish_cannot_override_the_declared_door_approach():
    config, grid = flat_map()
    config.search_finish_approach = [[.8,0.],[2.,0.]]
    config.search_finish_gate_yaw = 0.
    a = Autonomy(config)
    a.grid,a.started = grid,0.
    a.search.returning = True
    a.finish_goal,a.finish_marker = np.array([3.,1.]),np.array([3.65,1.])
    a.finish_observations,a.last_finish = 10,1.
    frame = Frame(1.,np.empty((0,3)))
    a.search.command(a,frame,Pose(quality=1.),frame.points,[],.1)
    np.testing.assert_array_equal(a.search.goal,[.8,0.])


def test_finish_requires_recent_marker_and_a_stable_stop():
    config, grid = flat_map()
    a = Autonomy(config)
    a.grid,a.started = grid,0.
    a.finish_goal,a.finish_marker = np.array([0.,0.]),np.array([.65,0.])
    a.search.returning = True
    a.finish_observations = 4
    pose = Pose(quality=1.)
    for now in np.arange(1.,2.5,.1):
        a.last_finish = now
        frame = Frame(now,np.empty((0,3)),velocity=Velocity(.3))
        result = a.search.command(a,frame,pose,frame.points,[],.1)
        assert result.state != 'finished'
    for now in np.arange(2.5,4.5,.1):
        a.last_finish = now-1. # Stop alone is insufficient.
        frame = Frame(now,np.empty((0,3)))
        result = a.search.command(a,frame,pose,frame.points,[],.1)
        assert result.state != 'finished'
    for now in np.arange(4.5,6.,.1):
        a.last_finish = now
        frame = Frame(now,np.empty((0,3)))
        result = a.search.command(a,frame,pose,frame.points,[],.1)
        if result.state == 'finished':
            break
    assert result.state == 'finished'
    assert result.events[-1].payload['official_signal_sent'] is False


def test_multiple_native_footballs_and_unique_scoring(tmp_path):
    mujoco = pytest.importorskip('mujoco')
    from go2_rescue.mujoco_scene import task_scene
    from go2_rescue.evaluation import SimulationEvaluator
    model = task_scene(ROOT,tmp_path,(.8,.8),'rubble-search',save_model=False)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    source = mujoco.MjModel.from_binary_path(str(ROOT/'scene/mujoco/competition-course.mjb'))
    evaluator = SimulationEvaluator(model,source,[0.,0.],0.)
    evaluator.search_mission = True
    assert len(evaluator.target_bodies) == 3
    assert not any(model.body(i).name=='local_football' for i in range(model.nbody))
    for body in evaluator.target_bodies:
        assert data.xpos[body,0] < 10. and data.xpos[body,1] < 5.
        geoms = set(np.flatnonzero(model.geom_bodyid==body))
        assert all(c.dist >= -.002 for c in data.contact if c.geom1 in geoms or c.geom2 in geoms)
        evaluator.update(data,[Event('target',0.,{'position':data.xpos[body,:2].tolist()})])
    evaluator.update(data,[Event('target',0.,{'position':data.xpos[evaluator.target_bodies[0],:2].tolist()})])
    report = evaluator.report(data,False,False)
    assert report['unique_targets_found']==3 and report['duplicate_target_reports']==1
    assert report['all_placed_targets_found'] and not report['local_task_pass']
    assert report['required_pipe'] is None and not report['official_signals_sent']


def test_simulation_keeps_explicit_fine_search_map_resolution(tmp_path):
    pytest.importorskip('mujoco')
    from go2_rescue.mujoco_sim import simulation_config
    config = Config(resolution=.05,search_mission=True)
    path = tmp_path/'config.json'
    config.save(path)
    assert simulation_config(path).resolution == .05
