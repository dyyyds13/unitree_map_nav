"""Actual rigid-body/torque integration tests; optional MuJoCo extra required."""
import os
from pathlib import Path
import json

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("casadi")

from go2_rescue.mpc_locomotion import ConvexMPC
from go2_rescue.mujoco_scene import task_scene
from go2_rescue.mujoco_sim import SimSensors, simulation_config, smoke_command
from go2_rescue.perception import Perception
from go2_rescue.types import Pose, Velocity

ROOT = Path(__file__).resolve().parents[1]


def test_sim_speed_config_honors_file_and_explicit_override(tmp_path):
    from go2_rescue.config import Config
    c = Config(max_vx=.35, passage_max_vx=.25, max_reverse_vx=.12)
    path = tmp_path / "speed.json"
    c.save(path)
    loaded = simulation_config(path)
    assert loaded.max_vx == .35 and loaded.passage_max_vx == .25
    assert loaded.max_reverse_vx == .12
    override = simulation_config(path, max_speed=.5)
    assert override.max_vx == .5 and override.passage_max_vx == .5
    assert simulation_config().max_vx == .5
    assert simulation_config().max_reverse_vx == .25
    assert simulation_config(path, max_reverse_speed=.2).max_reverse_vx == .2
    # Unknown real hardware keeps its separate existing speed defaults.
    assert Config().max_vx == .3 and Config().passage_max_vx == .08
    assert Config().max_reverse_vx == .08


@pytest.mark.parametrize("speed", [0., -.5, float("nan")])
def test_invalid_sim_speed_is_rejected(speed):
    with pytest.raises(ValueError):
        simulation_config(max_speed=speed)
    with pytest.raises(ValueError):
        simulation_config(max_reverse_speed=speed)


def test_offline_pipe_audit_rejects_tunnel3_only_path():
    # Offline evaluator unit fixture, never a physical traversal claim.
    from go2_rescue.evaluation import SimulationEvaluator
    model, data = plant()
    evaluator = SimulationEvaluator(model,model,[.8,.8],0.)
    for y in np.linspace(5.3,7.5,30):
        data.qpos[:3] = [11.6,y,.33]
        mujoco.mj_forward(model,data)
        evaluator.update(data)
    report = evaluator.report(data,False,False)
    assert report["required_pipe"] == "tunnel2"
    assert not report["pipe_passage_verified"]


def test_offline_pipe_audit_accepts_supported_west_to_east_tube2_poses_only_as_a_stage():
    # Static scorer unit fixture. These reset poses are NOT a dynamics test.
    from go2_rescue.evaluation import SimulationEvaluator
    model, data = plant()
    evaluator = SimulationEvaluator(model,model,[.8,.8],0.)
    assert len(evaluator.foot_geoms)==4  # Foot spheres belong to calf bodies.
    for x,z in [(8.2,.33),(8.95,.50),(9.6,.50),(10.4,.50),(11.1,.33)]:
        data.qpos[:3] = [x,6.22,z]
        mujoco.mj_forward(model,data)
        evaluator.update(data)
    report=evaluator.report(data,False,False)
    assert report["pipe2_floor_contact"] and report["pipe_passage_verified"]
    assert not report["ordered_traversal_verified"] and not report["local_task_pass"]


def plant():
    model = mujoco.MjModel.from_binary_path(str(ROOT / "scene/mujoco/competition-course.mjb"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[:3] = [.8, .8, .3]
    mujoco.mj_forward(model, data)
    return model, data


def test_local_entry_ramp_moves_only_the_declared_source_component(tmp_path):
    import hashlib
    source = ROOT / "scene/mujoco/competition-course.mjb"
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    model = task_scene(ROOT, tmp_path, (.8, .8), pipe_ramp="entry", save_model=False)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    heights = []
    for x in [7.85, 8.05, 8.2, 8.4, 8.58, 8.65, 10.9]:
        gid = np.array([-1], np.int32)
        distance = mujoco.mj_ray(model, data, np.array([x, 6.22, .45]), np.array([0., 0., -1.]),
                                 None, True, -1, gid)
        heights.append(.45-distance)
    assert abs(heights[0]) < 1e-6 and abs(heights[-1]) < 1e-6
    assert np.all(np.diff(heights[:6]) > 0.) and abs(heights[5]-.22) < 1e-6
    provenance = json.loads((tmp_path/"scene_provenance.json").read_text())
    assert provenance["scene_kind"] == "local_entry_ramp_plus_task_objects"
    assert len(provenance["geometry_changes"]) == 1
    assert provenance["max_geometry_vertex_error_m"] < 1e-6
    assert hashlib.sha256(source.read_bytes()).hexdigest() == before
    assert not (tmp_path/"runtime-scene.mjb").exists()


def test_front_camera_inset_uses_sensor_pixels_and_fits_viewport():
    from go2_rescue.mujoco_sim import camera_inset
    front = np.full((240, 320, 3), [20, 40, 90], np.uint8)
    viewport = mujoco.MjrRect(0, 0, 800, 600)
    rect, image = camera_inset(front, viewport)
    assert (rect.width, rect.height) == (320, 240)
    assert rect.left+rect.width <= viewport.width and rect.bottom+rect.height <= viewport.height
    np.testing.assert_array_equal(image[100, 100], [90, 40, 20])
    np.testing.assert_array_equal(front[100, 100], [20, 40, 90])
    rect, image = camera_inset(front, mujoco.MjrRect(0, 0, 200, 160))
    assert image.shape == (rect.height, rect.width, 3)
    assert rect.left+rect.width <= 200 and rect.bottom+rect.height <= 160


def test_actual_mpc_torques_walk_official_model_without_root_state_writes():
    model, data = plant()
    control = ConvexMPC(model, data, ROOT)
    initial = data.qpos[:3].copy()
    min_height = initial[2]
    for tick in range(1500):  # 3 physical seconds, only mj_step moves the robot.
        if tick % 3 == 0:
            command = Velocity(.15) if data.time >= 1. else Velocity()
            data.ctrl[:] = control.update(float(data.time), command)
        mujoco.mj_step(model, data)
        min_height = min(min_height, data.qpos[2])
    assert data.qpos[0] - initial[0] > .20
    assert min_height > .20
    assert control.solves > 100
    assert np.max(abs(data.ctrl) - control.limits) <= 1e-9
    assert not np.any(data.xfrc_applied) and not np.any(data.qfrc_applied)


def test_actual_mpc_half_metre_cruise_and_stop_on_official_clear_strip():
    from go2_rescue.evaluation import SimulationEvaluator
    model,data=plant()
    data.qpos[:3]=[6.4,2.65,.30]  # Isolated controller stage, initial reset only.
    mujoco.mj_forward(model,data)
    control=ConvexMPC(model,data,ROOT)
    sensors=SimSensors(model,data,simulation_config())
    evaluator=SimulationEvaluator(model,model,[6.4,2.65],0.)
    command=Velocity()
    samples=[]
    try:
        for tick in range(4000):
            if tick>=300 and tick%50==0:
                control.observe_terrain(sensors.lidar())
                command=smoke_command(float(data.time),"speed-smoke",.5)
            if tick%3==0:
                data.ctrl[:]=control.update(float(data.time),command)
            mujoco.mj_step(model,data)
            evaluator.update(data)
            if tick%50==0:
                samples.append(np.r_[data.time,data.qpos[:3]])
                assert data.qpos[2]>.20
                assert abs(data.qpos[1]-2.65)<.10
                assert np.max(abs(data.ctrl)-control.limits)<=1e-9
        samples=np.array(samples)
        cruise=samples[(samples[:,0]>2.5)&(samples[:,0]<4.5)]
        measured_speed=np.polyfit(cruise[:,0],cruise[:,1],1)[0]
        assert .45<measured_speed<.65
        stopped=samples[samples[:,0]>7.]
        assert np.linalg.norm(stopped[-1,1:3]-stopped[0,1:3])<.08
        assert not evaluator.incidents and not evaluator.water_episodes and not evaluator.boundary_episodes
        assert not np.any(data.xfrc_applied) and not np.any(data.qfrc_applied)
    finally:
        sensors.close()


def test_mpc_estimator_ignores_plant_global_translation_and_linear_twist():
    model, data = plant()
    first = ConvexMPC(model, data, ROOT)
    torque = first.update(0., Velocity())
    # Global root coordinates/linear velocities differ, but measured joints and
    # orientation are identical. Control must be translation invariant.
    data.qpos[:2] += [4., 3.]
    data.qvel[:3] = [10., -10., 0.]
    mujoco.mj_forward(model, data)
    second = ConvexMPC(model, data, ROOT)
    changed = second.update(0., Velocity())
    assert np.allclose(first.go2.position, second.go2.position, atol=1e-9)
    assert np.allclose(torque, changed, atol=1e-7)


def test_local_slope_friction_cone_uses_measured_surface_normal():
    model,data = plant()
    control = ConvexMPC(model,data,ROOT)
    # The unchanged flat case has the upstream world-Z friction pyramid.
    surface_solver = control.surface_solver_type(control.go2,control.traj)
    flat = np.asarray(surface_solver.surface_friction())[:4,12*control.traj.N:12*control.traj.N+3]
    np.testing.assert_allclose(flat,[[1.,0.,-.4],[-1.,0.,-.4],[0.,1.,-.4],[0.,-1.,-.4]])
    control.terrain_slope_limit_deg = 26.
    control.terrain_plane = np.array([.34,0.,0.])
    cone = np.asarray(surface_solver.surface_friction())[:4,12*control.traj.N:12*control.traj.N+3]
    assert (cone@np.array([0.,0.,100.]) <= 0.).all()  # Static vertical support is possible.
    assert (cone@np.array([15.,0.,100.]) > 0.).any()  # Too much uphill tangent force.
    assert (cone@np.array([-34.,0.,100.]) < 0.).all()  # Pure measured normal.


def test_mpc_reverses_under_torque_control_for_dead_end_retreat():
    model, data = plant()
    controller = ConvexMPC(model, data, ROOT)
    initial = data.qpos[:2].copy()
    for tick in range(1500):
        if tick % 3 == 0:
            data.ctrl[:] = controller.update(float(data.time), Velocity(-.06) if data.time > .6 else Velocity())
        mujoco.mj_step(model, data)
        assert data.qpos[2] > .20
    assert data.qpos[0]-initial[0] < -.08
    assert not np.any(data.xfrc_applied) and not np.any(data.qfrc_applied)


def test_mpc_quarter_metre_reverse_and_stop_on_clear_ground():
    from go2_rescue.evaluation import SimulationEvaluator
    model, data = plant()
    data.qpos[:3] = [6.4, 2.65, .30]  # Isolated controller stage, reset only.
    mujoco.mj_forward(model, data)
    control = ConvexMPC(model, data, ROOT)
    evaluator = SimulationEvaluator(model, model, [6.4, 2.65], 0.)
    samples = []
    for tick in range(2500):
        if tick % 3 == 0:
            command = Velocity(-.25) if .6 < data.time < 3. else Velocity()
            data.ctrl[:] = control.update(float(data.time), command)
        mujoco.mj_step(model, data)
        evaluator.update(data)
        assert data.qpos[2] > .20
        if tick % 50 == 0:
            samples.append([data.time, *data.qpos[:3]])
    samples = np.array(samples)
    cruise = samples[(samples[:, 0] > 1.5)&(samples[:, 0] < 2.8)]
    assert -.40 < np.polyfit(cruise[:, 0], cruise[:, 1], 1)[0] < -.18
    assert np.linalg.norm(samples[-1, 1:3]-samples[-10, 1:3]) < .08
    assert not evaluator.incidents and not evaluator.water_episodes and not evaluator.boundary_episodes


def test_ordered_prior_waypoint_postures_do_not_overlap_official_walls_or_piles():
    # Offline geometric audit only. No stepping, scene editing or route-generation
    # truth is exposed to Autonomy. Endpoints alone do not prove physical travel.
    from scipy.spatial.transform import Rotation
    model, data = plant()
    robot = set(np.flatnonzero(model.body_rootid[model.geom_bodyid] == model.body("base_link").id))
    model.geom_group[list(robot)] = 5
    groups = np.array([1, 1, 1, 1, 1, 0], dtype=np.uint8)
    definition = json.loads((ROOT / "configs/official-course-route.json").read_text())
    for stage in definition["stages"]:
        for waypoint in stage["waypoints"]:
            xy = np.asarray(waypoint["xy"])+definition["assumed_start_xy"]
            gid = np.array([-1], np.int32)
            distance = mujoco.mj_ray(model, data, np.r_[xy, .5], np.array([0., 0., -1.]), groups, True, -1, gid)
            floor = .5-distance if distance >= 0 else 0.
            data.qpos[:3] = [*xy, floor+.33]
            data.qpos[3:7] = np.roll(Rotation.from_euler("z", waypoint.get("yaw", 0.)).as_quat(), 1)
            mujoco.mj_forward(model, data)
            for contact in data.contact:
                a, b = int(contact.geom1), int(contact.geom2)
                if (a in robot) == (b in robot):
                    continue
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, b if a in robot else a) or ""
                forbidden = any(t in name for t in ["wall", "boundary", "pile", "boulder", "partition", "roof", "puddle"])
                assert not (forbidden and contact.dist < 0.), (stage["name"], waypoint, name)


def test_lidar_adapted_mpc_advances_up_official_ramp():
    model, data = plant()
    # Focused controller regression: reset once before the ramp and command
    # forward motion. This test does not claim autonomous approach/full course.
    data.qpos[:3] = [1.1, 4., .30]
    data.qpos[3:7] = [np.sqrt(.5), 0., 0., np.sqrt(.5)]
    mujoco.mj_forward(model, data)
    control = ConvexMPC(model, data, ROOT)
    sensors = SimSensors(model, data, simulation_config())
    try:
        for tick in range(4300):
            if tick % 50 == 0:
                control.observe_terrain(sensors.lidar())
            if tick % 3 == 0:
                data.ctrl[:] = control.update(float(data.time), Velocity(.14) if data.time >= .6 else Velocity())
            mujoco.mj_step(model, data)
            assert data.qpos[2] > .20
        # The old fixed-height/fixed-touchdown controller stalls near y=4.8.
        assert data.qpos[1] > 4.88
        assert data.qpos[2] > .39
        assert control.terrain_updates > 60
        assert np.rad2deg(np.arctan(np.linalg.norm(control.terrain_plane[:2]))) > 7.
        assert not np.any(data.xfrc_applied) and not np.any(data.qfrc_applied)
    finally:
        sensors.close()


def test_derived_task_preserves_geometry_and_camera_sees_physical_objects(tmp_path):
    model = task_scene(ROOT, tmp_path, (.8, .8), layout="near-start")
    provenance = json.loads((tmp_path / "scene_provenance.json").read_text())
    assert provenance["max_geometry_vertex_error_m"] < 1e-6
    assert provenance["imported_geoms"] > 700
    assert model.nu == 12 and model.nq == 26  # Actual dynamic football free joint.
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[:3] = [.8, .8, .33]
    mujoco.mj_forward(model, data)
    config = simulation_config(task_fixture=True, task_layout="near-start")
    sensors = SimSensors(model, data, config)
    try:
        frame = sensors.observe(Velocity())
    finally:
        sensors.close()
    frame.validate()
    assert frame.image.shape == (240, 320, 3)
    assert 30 < len(frame.points) <= 648
    perception = Perception(config)
    assert 42 in perception.aruco.detectMarkers(frame.image)[1]
    balls = [perception.locate_ball(d, frame, Pose(quality=1., sigma=.03))
             for d in perception.detect_balls(frame.image)]
    located = [d for d in balls if d.position is not None]
    assert len(located) == 1
    assert np.linalg.norm(located[0].position - [1.6, .25]) < .15
    assert perception.finish(frame.image, frame, Pose(quality=1.)) is not None


def test_sensor_raycast_hides_robot_without_altering_collision_or_render_groups():
    model, data = plant()
    before = model.geom_group.copy()
    sensors = SimSensors(model, data, simulation_config())
    try:
        points = sensors.lidar()
        assert len(points) > 50
        assert np.isfinite(points).all()
    finally:
        sensors.close()
    assert np.array_equal(before, model.geom_group)


@pytest.mark.parametrize("fast_render", [False, True])
def test_default_task_is_in_far_finish_area_not_visible_from_start(tmp_path, fast_render):
    model = task_scene(ROOT, tmp_path, (.8, .8))
    provenance = json.loads((tmp_path / "scene_provenance.json").read_text())
    assert provenance["task_layout"] == "finish-area"
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[:3] = [.8, .8, .33]
    mujoco.mj_forward(model, data)
    ball = data.body("local_football").xpos.copy()
    marker = data.geom("local_finish_marker").xpos.copy()
    assert np.allclose(ball, [11.35, 9.05, .11])
    assert np.allclose(marker, [11., 8.8, .43])
    assert np.linalg.norm(ball[:2] - data.qpos[:2]) > 10.
    ball_geom = model.geom("local_football_geom").id
    assert all(c.dist >= -.002 for c in data.contact
               if ball_geom in (c.geom1, c.geom2))
    config = simulation_config(task_fixture=True)
    assert config.mission_seconds > 20.
    sensors = SimSensors(model, data, config, fast_render=fast_render)
    perception = Perception(config)
    try:
        initial = sensors.observe(Velocity())
        assert perception.finish(initial.image, initial, Pose(quality=1.)) is None
        # Sensor calibration check inside the finish enclosure, not a mission
        # run or a shortcut used by navigation/robot control.
        data.qpos[:3] = [9.75, 8.8, .33]
        mujoco.mj_forward(model, data)
        nearby = sensors.observe(Velocity())
        assert perception.finish(nearby.image, nearby, Pose(quality=1.)) is not None
        located = [perception.locate_ball(d, nearby, Pose(quality=1., sigma=.03))
                   for d in perception.detect_balls(nearby.image)]
        assert any(d.position is not None and np.linalg.norm(d.position - [1.6, .25]) < .15
                   for d in located)
    finally:
        sensors.close()


@pytest.mark.parametrize("spawn,scope", [([.8, .8], "original_start"), ([6.5, 5.], "custom_start_stage_test")])
def test_simulation_time_limit_is_not_reported_as_completed_mission(tmp_path, capsys, spawn, scope):
    from types import SimpleNamespace
    from go2_rescue.mujoco_sim import run_simulation

    args = SimpleNamespace(output=str(tmp_path), config=None, seconds=.8,
                           spawn=spawn, control="autonomy", task_fixture=True,
                           viewer=False, realtime=False, record=False, seed=0, course_policy="explore")
    summary = run_simulation(args)
    assert summary["status"] == "time_limit"
    assert summary["stop_reason"] == "simulation_time_limit"
    assert not summary["mission_complete"] and not summary["fell"]
    assert summary["modified_course_geometry"] and summary["pipe_ramp"] == "entry"
    assert summary["run_scope"] == scope and not summary["full_course_pass"]
    assert np.allclose(summary["diagnostics_only"]["initial_world_position"][:2], spawn)
    assert not any(e["kind"] == "finish" for e in summary["events"])
    records = [json.loads(line) for line in (tmp_path / "runtime.jsonl").read_text().splitlines()]
    assert records[0]["kind"] == "startup"
    assert records[0]["initial_world_xy"] == spawn and records[0]["run_scope"] == scope
    assert any(r["kind"] == "event" and r["event"]["kind"] == "start" for r in records)
    assert records[-1]["stop_reason"] == "simulation_time_limit"
    assert "STOP: simulation_time_limit" in capsys.readouterr().out
    assert summary["recording"] == dict(video=False, raw=False, physics=False, snapshots=False)
    for name in ["simulation.mp4", "physics.npz", "preview.png", "live-preview.png", "map.png", "map.npz", "recording", "runtime-scene.mjb"]:
        assert not (tmp_path / name).exists(), name


def test_simulation_media_is_explicit_opt_in(tmp_path, capsys):
    from types import SimpleNamespace
    from go2_rescue.mujoco_sim import run_simulation
    args = SimpleNamespace(output=str(tmp_path), config=None, seconds=.8,
                           spawn=[.8, .8], control="smoke", task_fixture=False,
                           viewer=False, realtime=False, record=True, video=True,
                           save_physics=True, snapshots=True, seed=0)
    summary = run_simulation(args)
    assert all(summary["recording"].values())
    for name in ["simulation.mp4", "physics.npz", "preview.png", "map.npz", "recording"]:
        assert (tmp_path/name).exists(), name
    with np.load(tmp_path/"physics.npz") as physics:
        assert len(physics["states"]) >= 8


def test_slow_viewer_does_not_refresh_every_physics_tick(tmp_path, monkeypatch):
    from types import SimpleNamespace
    import mujoco.viewer
    import go2_rescue.mujoco_sim as sim

    # Model a renderer slower than its wall-clock refresh interval, without
    # sleeping or requiring a desktop. Physics must still advance between
    # display updates instead of synchronizing every 2 ms integration tick.
    wall = [0.]
    def clock():
        wall[0] += .01
        return wall[0]

    def sync():
        wall[0] += .2

    viewer = SimpleNamespace(
        opt=SimpleNamespace(geomgroup=np.ones(6)),
        user_scn=mujoco.MjvScene(),
        cam=SimpleNamespace(lookat=np.zeros(3)),
        viewport=SimpleNamespace(width=640, height=480),
        is_running=lambda: True, set_texts=lambda *args: None, sync=sync)
    monkeypatch.setattr(mujoco.viewer, "launch_passive", lambda *args: viewer)
    monkeypatch.setattr(sim, "time", SimpleNamespace(monotonic=clock, sleep=lambda _: None))
    monkeypatch.setattr(sim, "finish_viewer", lambda *args: None)
    args = SimpleNamespace(output=str(tmp_path), config=None, seconds=.2,
                           spawn=[.8, .8], control="smoke", task_fixture=False,
                           viewer=True, viewer_fps=15, realtime=False, record=False, seed=0)
    summary = sim.run_simulation(args)
    assert summary["simulation_seconds"] == pytest.approx(.2)
    assert 1 <= summary["compute"]["viewer_syncs"] <= 3
    assert summary["timing_p95_seconds"]["viewer_sync"] >= .2
