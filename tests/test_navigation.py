import numpy as np
import pytest

from go2_rescue.config import Config
from go2_rescue.geometry import rot2
from go2_rescue.localization import Localization, PoseGraph, register, register_lines, relative
from go2_rescue.mapping import GridMap
from go2_rescue.planning import DWA, Explorer, astar
from go2_rescue.synthetic import SensorWorld
from go2_rescue.types import Frame, Pose, Velocity
from dataclasses import replace


def test_batched_ground_medians_preserve_lower_envelope_with_duplicate_rays():
    from go2_rescue.terrain import ground_surface
    rng = np.random.default_rng(24)
    points = []
    expected = []
    for i in range(8):
        for j in range(8):
            column = np.column_stack([i*.05+rng.uniform(.005, .04, 7),
                                      j*.05+rng.uniform(.005, .04, 7),
                                      np.r_[rng.uniform(-.008, .008, 5), .4, .7]])
            # Some cells contain one ray, some contain only the lower subset.
            column = column[:1+(i*8+j)%7]
            points.extend(column)
            lower = column[column[:, 2] <= column[:, 2].min()+.02]
            expected.append(np.median(lower, axis=0))
    rng.shuffle(points)
    actual = ground_surface(np.asarray(points), 0., .05, .3, 14.)
    np.testing.assert_array_equal(actual, np.asarray(expected))


def test_half_metre_per_second_cruise_still_checks_the_braking_tail():
    c=Config(map_size=12., max_vx=.5, footprint_half_length=.35,
             footprint_half_width=.23, robot_radius=.23, safety_margin=.05)
    grid=GridMap(c)
    grid.seen[:]=True;grid.floor[:]=0.;grid.logodds[:]=-4.
    planner=DWA(c)
    pose,velocity=Pose(quality=1.),Velocity()
    speeds=[]
    for _ in range(25):
        path=np.column_stack([np.arange(pose.x,4.,.05),np.zeros(len(np.arange(pose.x,4.,.05)))])
        velocity=planner.command(pose,velocity,path,np.empty((0,3)),grid,.1,desired_yaw=0.)
        pose.x+=velocity.vx*.1
        speeds.append(velocity.vx)
    assert min(speeds[-5:])>=.48 and max(speeds)<=.5
    # The wall is beyond today's body but inside a full-speed stopping path.
    wall_x=.85  # Obstacle input is body-frame, not map-frame.
    wall=np.column_stack([np.full(20,wall_x),np.linspace(-1.,1.,20),np.zeros(20)])
    path=np.array([pose.xy(),[pose.x+2.,pose.y]])
    assert planner.command(pose,velocity,path,wall,grid,.1,desired_yaw=0.)==Velocity()


def test_quarter_metre_reverse_retains_rear_braking_check():
    c = Config(map_size=12., max_reverse_vx=.25, footprint_half_length=.35,
               footprint_half_width=.23, robot_radius=.23, safety_margin=.05)
    grid = GridMap(c)
    grid.seen[:] = True
    grid.floor[:] = 0.
    grid.logodds[:] = -4.
    planner = DWA(c)
    pose, measured = Pose(), Velocity()
    for _ in range(20):
        path = np.array([pose.xy(), [-3., 0.]])
        measured = planner.command(pose, measured, path, np.empty((0, 3)), grid, desired_yaw=0.)
        pose.x += measured.vx*.1
    assert -.25 <= measured.vx < -.23
    wall = np.column_stack([np.full(15, -.55), np.linspace(-1., 1., 15), np.zeros(15)])
    assert planner.command(pose, measured, path, wall, grid, desired_yaw=0.) == Velocity()


def test_astar_routes_around_blockage_and_refuses_corner_cut():
    free = np.ones((15, 15), bool)
    free[7, :12] = False
    path = astar(free, (2, 2), (12, 2))
    assert path[0] == (2, 2) and path[-1] == (12, 2)
    assert all(free[p] for p in path)
    assert max(y for _, y in path) >= 12
    free = np.array([[True, False], [False, True]])
    assert astar(free, (0, 0), (1, 1)) == []
    assert astar(free, (-1, 0), (1, 1)) == []


def test_scan_matching_recovers_measured_motion_without_true_pose():
    rng = np.random.default_rng(7)
    target = rng.uniform(-2, 2, (500, 2))
    actual = Pose(.17, -.09, .07)
    source = (target - actual.xy()) @ rot2(actual.yaw)
    fitted, overlap, rmse = register(source, target, Pose(.12, -.05, .04))
    assert np.linalg.norm(fitted.xy() - actual.xy()) < .01
    assert abs(fitted.yaw - actual.yaw) < .01
    assert overlap > .98 and rmse < .01


def test_line_icp_preserves_measured_motion_along_featureless_corridor():
    y = np.linspace(-4., 4., 160)
    # Sensor resampling on infinite parallel walls has no longitudinal feature.
    walls = np.vstack([np.column_stack([np.full(len(y), x), y]) for x in [-.7, .7]])
    fitted, overlap, rmse = register_lines(walls, walls, Pose(0., .15, 0.))
    assert abs(fitted.y-.15) < .001
    assert abs(fitted.x) < .001 and overlap > .9 and rmse < .001


def test_line_icp_uses_corners_to_correct_both_translation_axes():
    t = np.linspace(-2., 2., 120)
    target = np.vstack([np.column_stack([t, np.full(len(t), 3.)]),
                        np.column_stack([np.full(len(t), 2.), t])])
    actual = Pose(.17, .09, .07)
    source = (target-actual.xy())@rot2(actual.yaw)
    fitted, overlap, rmse = register_lines(source, target, Pose(.12, .05, .04))
    assert np.linalg.norm(fitted.xy()-actual.xy()) < .01
    assert abs(fitted.yaw-actual.yaw) < .01 and overlap > .9 and rmse < .01


def test_stable_imu_reference_prevents_scan_fit_heading_drift():
    t = np.linspace(-2., 2., 120)
    target = np.vstack([np.column_stack([t, np.full(len(t), 3.)]),
                        np.column_stack([np.full(len(t), 2.), t])])
    # A biased scan map may otherwise rotate every accepted estimate. The
    # explicitly configured sensor reference must retain the measured heading.
    source = (target-[.1, .05])@rot2(.10)
    fitted, overlap, rmse = register_lines(source, target, Pose(.1, .05, 0.), yaw_hint=0.)
    assert abs(fitted.yaw) < .04


def test_pose_graph_preserves_configured_sensor_heading_priors():
    graph = PoseGraph()
    graph.nodes = [Pose(0., 0., 0.), Pose(1., 0., .1), Pose(2., 0., .2)]
    graph.edges = [(0, 1, np.array([1., 0., .1]), 1.),
                   (1, 2, np.array([1., 0., .1]), 1.)]
    graph.optimize([0., 0., 0.])
    assert max(abs(p.yaw) for p in graph.nodes) < .01


def test_slow_narrow_passage_retains_body_collision_and_water_checks():
    c = Config(map_size=6., resolution=.05, footprint_half_length=.35,
               footprint_half_width=.23, robot_radius=.23, safety_margin=.05,
               hazard_foot_half_length=.28, hazard_foot_half_width=.22)
    g = GridMap(c)
    g.seen[:] = True
    g.logodds[:] = -4.
    g.floor[:] = 0.
    x = g.origin[0]+(np.arange(g.n)+.5)*c.resolution
    g.logodds[(x < -.25)|(x >= .30), :] = 4.
    p = Pose(.025, 0., np.pi/2)
    path = np.array([p.xy(), [.025, 1.]])
    assert DWA(c).command(p, Velocity(), path, np.empty((0, 3)), g) == Velocity()
    from go2_rescue.controller import Autonomy
    narrow = Autonomy(c).passage_dwa
    command = narrow.command(p, Velocity(), path, np.empty((0, 3)), g, desired_yaw=np.pi/2)
    assert 0. < command.vx <= .08 and abs(command.vy) <= .05
    # Extra clearance may shrink; the physical body must never cross a wall.
    blocked = narrow.command(Pose(.16, 0., np.pi/2), Velocity(), path, np.empty((0, 3)), g)
    assert blocked == Velocity()
    g.hazard_votes[g.cell([.025, .1])] = 2
    g._has_hazards = True
    g._free_cache = g._surface_safe = None
    assert narrow.command(p, Velocity(), path, np.empty((0, 3)), g) == Velocity()


def test_se2_pose_goal_requires_heading_even_when_position_is_already_reached():
    c = Config(map_size=6., resolution=.1, footprint_half_length=.35,
               footprint_half_width=.23, safety_margin=.05)
    grid = GridMap(c)
    grid.seen[:] = True
    grid.logodds[:] = -4.
    grid.floor[:] = 0.
    route = Explorer().maneuver(grid, Pose(), np.array([0., 0.]), goal_yaw=np.pi/2)
    assert len(route) > 1
    assert np.linalg.norm(route[-1, :2]) < .12
    assert abs(route[-1, 2]-np.pi/2) < .14


def test_se2_goal_region_finds_safe_pose_when_its_center_is_blocked():
    c = Config(map_size=6., resolution=.1, footprint_half_length=.35,
               footprint_half_width=.23, safety_margin=.05)
    grid = GridMap(c)
    grid.seen[:] = True
    grid.logodds[:] = -4.
    grid.floor[:] = 0.
    x = grid.origin[0]+(np.arange(grid.n)+.5)*c.resolution
    grid.logodds[x >= .4, :] = 4.
    goal = np.array([.2, 0.])
    route = Explorer().maneuver(grid, Pose(-.2, 0.), goal, goal_yaw=np.pi/2, position_tolerance=.35)
    assert len(route) > 1
    assert np.linalg.norm(route[-1, :2]-goal) < .35
    assert abs(route[-1, 2]-np.pi/2) < .14
    xx, yy = np.meshgrid(np.linspace(-.4, .4, 9), np.linspace(-.28, .28, 7))
    offsets = np.column_stack([xx.ravel(), yy.ravel()])
    assert not grid.footprint_safe(goal, np.pi/2, offsets)
    assert grid.footprints_safe(route[:, :2], route[:, 2], offsets).all()


def test_horizontal_roof_is_excluded_from_planar_icp_but_retained_for_clearance():
    c = Config(localization_metric="point_to_line")
    localization = Localization(c)
    x, y = np.meshgrid(np.linspace(-.9, .9, 23), np.linspace(-1.5, 1.5, 31))
    roof = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, .27)])
    y, z = np.meshgrid(np.linspace(-1.5, 1.5, 45), np.linspace(-.2, .6, 13))
    walls = np.vstack([np.column_stack([np.full(y.size, xx), y.ravel(), z.ravel()]) for xx in [-1.1, 1.1]])
    points = np.vstack([roof, walls])
    structural = localization.structural_scan(points)
    assert np.sum(abs(structural[:, 2]-.27) < .001) == 0
    assert np.sum(abs(structural[:, 0]) > .99) >= c.min_points
    pose, mapping_points = localization.update(Frame(0., points))
    assert pose.quality >= c.min_pose_quality
    assert np.sum(abs(mapping_points[:, 2]-.27) < .001) == len(roof)


def test_line_quality_uses_valid_correspondences_and_rejects_tiny_overlap():
    t = np.linspace(-2., 2., 160)
    walls = np.vstack([np.column_stack([t, np.full(len(t), 3.)]),
                       np.column_stack([np.full(len(t), 2.), t])])
    x, y = np.meshgrid(np.linspace(-.5, .5, 24), np.linspace(-.5, .5, 24))
    target = np.vstack([walls, np.column_stack([x.ravel(), y.ravel()])])
    actual = Pose(.10, .05, .03)
    source = (target-actual.xy())@rot2(actual.yaw)
    fitted, overlap, rmse = register_lines(source, target, Pose(.08, .03, .02))
    assert overlap > .5 and rmse < .02
    assert np.linalg.norm(fitted.xy()-actual.xy()) < .02
    unrelated = np.vstack([walls[:40], np.full((400, 2), 100.)])
    _, overlap, _ = register_lines(unrelated, walls, Pose())
    assert overlap < .3


def test_pose_graph_loop_reduces_drift():
    graph = PoseGraph()
    graph.nodes = [Pose(0, 0, 0), Pose(1.1, 0, 0), Pose(1.2, 1.1, 0), Pose(.3, 1.2, 0), Pose(.4, .2, 0)]
    graph.edges = [(i, i + 1, relative(graph.nodes[i], graph.nodes[i + 1]), 1.) for i in range(4)]
    graph.edges.append((0, 4, np.zeros(3), 5.))
    before = np.linalg.norm(graph.nodes[-1].xy())
    graph.optimize()
    assert np.linalg.norm(graph.nodes[-1].xy()) < before / 3
    assert np.linalg.norm(graph.nodes[0].xy()) == 0.


def test_mapping_requires_floor_and_inflates_obstacles():
    c = Config(map_size=10.)
    grid = GridMap(c)
    # Ground support across a corridor plus a vertical obstacle in its centre.
    x, y = np.meshgrid(np.arange(-2, 2, .08), np.arange(-1, 1, .08))
    ground = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, -c.base_height)])
    obstacle = np.array([[1., 0., 0.], [1., .02, .2]])
    grid.update(np.vstack([ground, obstacle]), Pose(quality=1))
    free = grid.traversable()
    assert free[grid.cell([0, 0])]
    assert not free[grid.cell([1, 0])]
    assert not free[grid.cell([.7, 0])]
    assert not free[grid.cell([-3, 0])]
    # A new obstacle closes a previously open cell immediately.
    cell = grid.cell([-.8, 0])
    assert free[cell]
    grid.update(np.array([[-.8, 0., .05]]), Pose(quality=1))
    assert not grid.traversable()[cell]


def test_groundless_scans_never_establish_walkable_corridor():
    c = Config(map_size=10.)
    grid = GridMap(c)
    wall = np.array([[2., y, 0.] for y in np.linspace(-2, 2, 72)])
    grid.update(wall, Pose(quality=1))
    assert not grid.traversable()[grid.cell([1, 0])]


def test_continuous_ramp_is_ground_in_both_astar_and_dwa():
    c = Config(map_size=10.)
    grid = GridMap(c)
    x, y = np.meshgrid(np.arange(-1., 3., .075), np.arange(-1.1, 1.1, .075))
    height = np.maximum(x, 0) * np.tan(np.deg2rad(8.))
    points = np.column_stack([x.ravel(), y.ravel(), height.ravel() - c.base_height])
    pose = Pose(quality=1.)
    grid.update(points, pose)
    goal = grid.cell([2., 0.])
    assert grid.floor[goal] > .25  # Old fixed-height code would block this.
    path = astar(grid.traversable(), grid.cell([0., 0.]), goal)
    assert path
    obstacles = grid.obstacle_points(points, pose)
    assert len(obstacles) == 0
    command = DWA(c).command(pose, Velocity(), np.array([grid.world(p) for p in path]), obstacles, grid)
    assert command.vx > 0
    # Real obstacles on top of the ramp still block, at height above that ramp.
    grid.update(np.array([[1.5, 0., height.max() + .3 - c.base_height]]), pose)
    assert not grid.traversable()[grid.cell([1.5, 0.])]


def test_steep_surface_and_wide_ground_gap_remain_blocked():
    c = Config(map_size=10.)
    x, y = np.meshgrid(np.arange(-1., 3., .08), np.arange(-1., 1., .08))
    steep = np.column_stack([x.ravel(), y.ravel(),
                            np.maximum(x, 0).ravel() * np.tan(np.deg2rad(25.)) - c.base_height])
    grid = GridMap(c)
    grid.update(steep, Pose())
    assert not grid.traversable()[grid.cell([1.5, 0.])]
    # A large region without returns is not inferred as a traversable floor.
    keep = (x.ravel() < .4) | (x.ravel() > 1.3)
    grid = GridMap(c)
    grid.update(np.column_stack([x.ravel()[keep], y.ravel()[keep],
                                 np.full(keep.sum(), -c.base_height)]), Pose())
    assert not grid.traversable()[grid.cell([.85, 0.])]
    assert not astar(grid.traversable(), grid.cell([0., 0.]), grid.cell([2., 0.]))


def test_local_slope_capability_preserves_obstacles_and_outside_limits():
    x, y = np.meshgrid(np.arange(-1., 2.5, .06), np.arange(-.9, .9, .06))
    def scan(angle):
        return np.column_stack([x.ravel(), y.ravel(),
            np.clip(x, 0., 1.5).ravel()*np.tan(np.deg2rad(angle))-.30])
    c = Config(map_size=8., resolution=.05, base_height=.30,
        terrain_slope_regions=[{"bounds": [-.3, 2., -1., 1.], "max_slope_deg": 22.}])
    c.validate()
    assert c.slope_limit([1., 0.]) == 22.
    assert c.slope_limit([3., 0.]) == c.max_slope_deg
    g = GridMap(c);g.update(scan(18.7), Pose())
    assert g.traversable()[g.cell([.8, 0.])]
    assert len(g.obstacle_points(scan(18.7), Pose())) == 0
    # A physical obstacle remains blocked even in the special slope region.
    for _ in range(3):
        g.update(np.array([[.8, 0., .8*np.tan(np.deg2rad(18.7))-.30+.20]]), Pose())
    assert not g.traversable()[g.cell([.8, 0.])]
    for config, angle in [(c, 28.), (Config(map_size=8., resolution=.05, base_height=.30), 18.7)]:
        g = GridMap(config);g.update(scan(angle), Pose())
        assert not g.traversable()[g.cell([.8, 0.])]


def test_cached_ground_patch_never_extrapolates_or_bridges_holes():
    from go2_rescue.terrain import GroundPatch, interpolate_surface
    x,y = np.meshgrid(np.arange(0., 1., .1), np.arange(0., 1., .1))
    samples = np.column_stack([x.ravel(),y.ravel(),np.maximum(x-.4,0.).ravel()*.34])
    query = np.array([[.25,.35],[.65,.55],[-.1,.5],[1.1,.5]])
    actual = GroundPatch(samples,.3,22.).height(query)
    np.testing.assert_allclose(actual, interpolate_surface(samples,query,.3,22.), equal_nan=True)
    assert np.isnan(actual[2:]).all()
    gapped = samples[(samples[:,0]<.25)|(samples[:,0]>.65)]
    assert np.isnan(GroundPatch(gapped,.3,22.).height([.45,.45])[0])
    steep = samples.copy();steep[:,2] = steep[:,0]
    assert np.isnan(GroundPatch(steep,.3,22.).height([.45,.45])[0])


def test_later_overhead_returns_cannot_raise_an_observed_floor():
    c = Config(map_size=6., resolution=.05, base_height=.30,
        terrain_slope_regions=[{"bounds": [-2.,2.,-2.,2.], "max_slope_deg": 26.}])
    g = GridMap(c)
    x,y = np.meshgrid(np.arange(-.5,1.6,.05),np.arange(-.6,.61,.05))
    floor = np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.30)])
    g.update(floor,Pose())
    # A connected gentle canopy reaches an already measured flat floor cell.
    canopy = floor.copy();canopy[:,2] += np.maximum(x,0.).ravel()*.40
    for _ in range(5):
        g.update(canopy,Pose())
    assert g.floor[g.cell([1.3,0.])] < .08
    assert len(g.obstacle_points(canopy,Pose())) > 0


def test_forward_route_cap_has_candidates_when_joint_odometry_is_faster():
    c = Config(map_size=6.,max_vx=.5)
    g = GridMap(c);g.seen[:]=True;g.floor[:]=0.;g.logodds[:]=-4.
    command = DWA(c).command(Pose(),Velocity(.25),np.array([[0.,0.],[2.,0.]]),
        np.empty((0,3)),g,forward_limit=.12)
    assert 0 < command.vx <= .12


def test_headroom_recomputes_when_a_prior_high_floor_is_corrected():
    c = Config(map_size=6.,resolution=.05,base_height=.30)
    g = GridMap(c)
    x,y = np.meshgrid(np.arange(-.6,1.6,.05),np.arange(-.6,.61,.05))
    points = np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.30)])
    g.update(points,Pose())
    cell = g.cell([1.,0.])
    g.floor[cell]=.6;g.ceiling[cell]=.9;g.headroom[cell]=.3
    for _ in range(6):
        g.update(points,Pose())
    assert g.floor[cell] < .03
    assert g.headroom[cell] > .85
    assert g.traversable()[cell]


def test_headroom_is_relative_to_ramp_height_not_start_height():
    c = Config(map_size=10.)
    grid = GridMap(c)
    x, y = np.meshgrid(np.arange(-1., 3., .075), np.arange(-1., 1., .075))
    z = np.maximum(0., x) * np.tan(np.deg2rad(8.))
    grid.update(np.column_stack([x.ravel(), y.ravel(), z.ravel() - c.base_height]), Pose())
    cell = grid.cell([2., 0.])
    assert grid.traversable()[cell]
    roof_z = .70  # Above the old absolute .55m threshold, but too low over ramp.
    grid.update(np.array([[2., 0., roof_z - c.base_height]]), Pose())
    assert grid.headroom[cell] < c.body_clearance
    assert not grid.traversable()[cell]


def test_narrow_frontier_is_not_discarded_after_clearance_check():
    c = Config(map_size=10.)
    grid = GridMap(c)
    # Known flat supported patch; only two cells border unconfirmed support.
    start = grid.cell([0., 0.])
    grid.seen[45:70, 45:56] = True
    grid.logodds[45:70, 45:56] = -2.
    grid.floor[44:71, 44:57] = 0.
    grid.floor[68:70, 56] = np.nan
    goal = Explorer().choose(grid, Pose(), 0.)
    assert goal is not None
    assert astar(grid.traversable(), start, grid.cell(goal))


def test_oriented_footprint_fits_lane_but_cannot_turn_through_walls():
    c = Config(map_size=10., robot_radius=.23, safety_margin=.05,
               footprint_half_length=.35, footprint_half_width=.23)
    grid = GridMap(c)
    x, y = np.meshgrid(np.arange(-2., 2., .06), np.arange(-.4, .4, .06))
    ground = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, -c.base_height)])
    xx = np.arange(-2., 2., .06)
    walls = np.vstack([np.column_stack([xx, np.full(len(xx), yy), np.full(len(xx), .0)])
                       for yy in [-.44, .44]])
    grid.update(np.vstack([ground, walls]), Pose())
    offsets = np.array([[a, b] for a in np.linspace(-.4, .4, 9) for b in np.linspace(-.28, .28, 7)])
    assert grid.traversable()[grid.cell([0., 0.])]
    assert grid.footprint_safe(np.zeros(2), 0., offsets)
    assert not grid.footprint_safe(np.zeros(2), np.pi / 2, offsets)
    path = np.column_stack([np.linspace(0., 1.2, 20), np.zeros(20)])
    command = DWA(c).command(Pose(), Velocity(), path, walls, grid)
    assert command.vx > 0


def test_odometry_gates_empty_stale_and_degenerate_clouds():
    c = Config(map_size=10.)
    world, localizer = SensorWorld(c), Localization(c)
    pose, _ = localizer.update(world.frame())
    assert pose.quality >= c.min_pose_quality
    world.advance(Velocity(.2, .0, .05))
    pose, _ = localizer.update(world.frame())
    assert pose.quality >= c.min_pose_quality
    previous = pose.xy().copy()
    pose, _ = localizer.update(Frame(.2, np.empty((0, 3))))
    assert pose.quality == 0 and np.allclose(pose.xy(), previous)
    with pytest.raises(ValueError, match="increase"):
        localizer.update(Frame(.2, np.ones((50, 3))))
    line = np.column_stack([np.linspace(-2, 2, 80), np.ones(80), np.zeros(80)])
    pose, _ = Localization(c).update(Frame(0., line))
    assert pose.quality == 0


def test_explorer_finds_supported_frontier_and_dwa_obeys_bounds():
    c = Config(map_size=10.)
    world = SensorWorld(c)
    frame = world.frame()
    localizer = Localization(c)
    pose, points = localizer.update(frame)
    grid = GridMap(c)
    grid.update(points, pose)
    goal = Explorer().choose(grid, pose, 0.)
    # The safe approach stands inside the observed patch, before the raw
    # frontier; it can be closer than the old 65cm center-only frontier goal.
    assert goal is not None and np.linalg.norm(goal - pose.xy()) > .25
    path = astar(grid.traversable(), grid.cell(pose.xy()), grid.cell(goal))
    command = DWA(c).command(pose, Velocity(), np.array([grid.world(p) for p in path]),
                             points[points[:, 2] > -.22], grid)
    assert abs(command.vx) <= c.acceleration * .1 + 1e-9
    assert abs(command.wz) <= c.angular_acceleration * .1 + 1e-9
    assert DWA(c).command(pose, Velocity(), np.empty((0, 2)), points, grid) == Velocity()


def test_kiss_backend_installed_and_runs_on_sensor_cloud():
    pytest.importorskip("kiss_icp")
    c = Config(localization_backend="kiss", map_size=10.)
    world, localizer = SensorWorld(c), Localization(c)
    localizer.update(world.frame())
    world.advance(Velocity(.1, 0, 0))
    pose, _ = localizer.update(world.frame())
    assert np.isfinite([pose.x, pose.y, pose.yaw, pose.quality]).all()
    assert localizer.kiss is not None
