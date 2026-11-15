"""Real MuJoCo dynamics + open-source MPC + the existing autonomy pipeline."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
from collections import deque

import cv2
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .config import Config
from .controller import Autonomy
from .mpc_locomotion import ConvexMPC
from .recording import Recorder
from .types import Frame, Step, Velocity


class SimSensors:
    """Sensor renderer/raycaster; plant truth is internal to sensor generation.

    The public output is Frame, with joint-kinematic measured velocity. It does
    not expose plant qpos, geom IDs, ball coordinates or a precomputed map.
    """
    def __init__(self, model, data, config: Config, seed=0):
        self.model, self.data, self.config = model, data, config
        self.base = model.body("base_link").id
        self.rng = np.random.default_rng(seed)
        self.robot_geoms = np.flatnonzero(model.body_rootid[model.geom_bodyid] == self.base)
        azimuth = np.linspace(-np.pi, np.pi, 72, endpoint=False)
        elevation = np.deg2rad(np.linspace(-60, 15, 9))
        self.directions = np.array([[np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)]
                                    for e in elevation for a in azimuth])
        self.renderer = mujoco.Renderer(model, height=240, width=320)
        from OpenGL.GL import glGetString, GL_RENDERER
        self.graphics_device = (glGetString(GL_RENDERER) or b"unknown").decode(errors="replace")
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 1  # Official course collision surfaces are group 3.
        self.camera = mujoco.MjvCamera()
        self.model.vis.global_.fovy = 60.

    def lidar(self):
        model, data = self.model, self.data
        rotation = data.xmat[self.base].reshape(3, 3)
        offset = np.array([0., 0., .10])
        origin = data.xpos[self.base] + rotation @ offset
        directions = self.directions @ rotation.T
        ids = np.full(len(directions), -1, dtype=np.int32)
        distances = np.full(len(directions), -1., dtype=float)
        previous = model.geom_group[self.robot_geoms].copy()
        try:
            # Exclude the complete robot, not only the base-link geom. Restore
            # immediately; rendering and collision masks are unchanged.
            model.geom_group[self.robot_geoms] = 5
            groups = np.array([1, 1, 1, 1, 1, 0], dtype=np.uint8)
            mujoco.mj_multiRay(model, data, origin, directions.ravel(), groups, True, -1,
                              ids, distances, None, len(directions), self.config.max_range)
        finally:
            model.geom_group[self.robot_geoms] = previous
        valid = (distances > .08) & (distances < self.config.max_range)
        ranges = distances[valid] + self.rng.normal(0, .002, valid.sum())
        return offset + self.directions[valid] * ranges[:, None]

    def image(self):
        rotation = self.data.xmat[self.base].reshape(3, 3)
        transform = np.asarray(self.config.body_from_camera)
        origin = self.data.xpos[self.base] + rotation @ transform[:3, 3]
        optical = rotation @ transform[:3, :3]
        self.renderer.update_scene(self.data, camera=self.camera, scene_option=self.option)
        for camera in self.renderer.scene.camera:
            camera.pos[:] = origin
            camera.forward[:] = optical[:, 2]
            camera.up[:] = -optical[:, 1]
        return cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)

    def observe(self, measured: Velocity):
        return Frame(float(self.data.time), self.lidar(),
                     self.data.sensor("imu_quat").data.copy(), measured, self.image())

    def close(self):
        self.renderer.close()


def simulation_config(path=None, task_fixture=False, task_layout="finish-area", max_speed=None, max_reverse_speed=None):
    c = Config.load(path)
    c.base_height = .33
    c.camera_size = [320, 240]
    focal = 120. / np.tan(np.deg2rad(30.))
    c.camera_matrix = [[focal, 0., 160.], [0., focal, 120.], [0., 0., 1.]]
    c.distortion = [0., 0., 0., 0., 0.]
    c.body_from_camera = [[0., 0., 1., .24], [-1., 0., 0., 0.], [0., -1., 0., .10], [0., 0., 0., 1.]]
    # The MJB contains no camera. Mount this documented simulated front camera
    # 15deg downward so the ground ahead remains visible while climbing.
    camera_transform = np.asarray(c.body_from_camera)
    camera_transform[:3, :3] = Rotation.from_euler("y", 15., degrees=True).as_matrix() @ camera_transform[:3, :3]
    c.body_from_camera = camera_transform.tolist()
    if path is None:
        c.max_vx, c.max_vy, c.max_wz = .50, .08, .35
        c.passage_max_vx = .50
        c.max_reverse_vx = .25
    if max_speed is not None:
        c.max_vx = c.passage_max_vx = max_speed
    if max_reverse_speed is not None:
        c.max_reverse_vx = max_reverse_speed
    c.resolution = .10
    # Simulator Go2 dimensions are known. Use the body's oriented envelope in
    # DWA; a circumscribed disk plus the old 12cm margin sealed the start lane.
    # Unknown real hardware keeps its original conservative circular defaults.
    c.robot_radius, c.safety_margin = .23, .05
    c.footprint_half_length, c.footprint_half_width = .35, .23
    c.hazard_foot_half_length, c.hazard_foot_half_width = .28, .22
    c.body_clearance = .48  # Known simulated standing envelope plus head margin.
    # Official ramp measurements are about 12 degrees. This simulator-only
    # bound is backed by the sensor-driven MPC ascent regression; real hardware
    # keeps its calibrated/default terrain limit.
    c.max_slope_deg = 14. if path is None else c.max_slope_deg
    if path is None:
        c.water_detection = True
    if task_fixture and path is None:
        c.target_quota = 1  # This explicitly constructed local fixture has one target.
        if task_layout == "near-start":
            c.mission_seconds, c.return_reserve = 20., 12.
        else:
            c.mission_seconds, c.return_reserve = 600., 60.
    c.validate()
    return c


def smoke_command(t, profile="smoke", max_speed=.5):
    if profile == "speed-smoke":
        # Acceleration and braking are time profiles, independent of plant
        # truth. This is a controller test, not autonomous navigation.
        speed = max(0., min(max_speed, .45*(t-1.), .45*(6.-t)))
        return Velocity(speed)
    if profile == "ramp-smoke":
        return Velocity(.14) if .6 <= t < 18. else Velocity()
    if t < 1.:
        return Velocity()
    if t < 4.:
        return Velocity(.15)
    if t < 6.:
        return Velocity(wz=.25)
    return Velocity()


class RuntimeLog:
    """Flush diagnostics to both the terminal and an inspectable JSONL file."""
    def __init__(self, path):
        self.file = Path(path).open("w")

    def emit(self, kind, timestamp, message, **details):
        record = {"kind": kind, "simulation_seconds": timestamp, "message": message, **details}
        self.file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self.file.flush()
        print(f"[go2 sim={timestamp:6.2f}s] {message}", flush=True)

    def close(self):
        self.file.close()


def close_viewer(viewer):
    """Keep model/image storage alive until the passive render thread exits.

    MuJoCo3.17 Handle.close() only requests exit. Its pinned Handle owns a
    weakref to Simulate, which expires after render_loop/destroy complete.
    Waiting here avoids GLFW teardown racing that daemon at Python exit.
    """
    viewer.clear_images()
    viewer.close()
    reference = getattr(viewer, "_sim", None)
    deadline = time.monotonic()+3.
    while reference is not None and reference() is not None and time.monotonic() < deadline:
        time.sleep(.01)


def finish_viewer(viewer, keep_open):
    """Review a frozen final frame without stepping physics or sending commands."""
    if viewer is None:
        return
    try:
        if keep_open and viewer.is_running():
            print("[go2] 仿真已停止，保留最终画面。关闭窗口或按 Ctrl+C 退出；不会继续物理步进。", flush=True)
            while viewer.is_running():
                viewer.sync()
                time.sleep(.05)
    except KeyboardInterrupt:
        pass
    finally:
        close_viewer(viewer)


def camera_inset(front_bgr, viewport):
    """Reuse the sensor image as a live RGB overlay; no rendering/recording."""
    width = min(480, max(320, viewport.width//3), max(1, viewport.width-32), max(1, int((viewport.height-64)*4/3)))
    height = max(1, width*3//4)
    rgb = cv2.cvtColor(front_bgr, cv2.COLOR_BGR2RGB)
    if rgb.shape[:2] != (height, width):
        rgb = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_AREA)
    cv2.rectangle(rgb, (0, 0), (width-1, min(21, height-1)), (15, 15, 15), -1)
    cv2.putText(rgb, "ROBOT FRONT CAMERA | 10 Hz", (6, 15), cv2.FONT_HERSHEY_SIMPLEX,
                .38, (245, 245, 245), 1, cv2.LINE_AA)
    return mujoco.MjrRect(viewport.left+max(0, viewport.width-width-16), viewport.bottom+16, width, height), rgb


def run_simulation(args):
    # Go2 uses tiny 3x3/18x18 systems. BLAS worker-pool dispatch costs more
    # than these solves and competes with the viewer. Restore limits on exit.
    from threadpoolctl import threadpool_limits
    with threadpool_limits(limits=1, user_api="blas"):
        return _run_simulation(args)


def _run_simulation(args):
    root = Path(__file__).resolve().parents[2]
    pipe_ramp = getattr(args, "pipe_ramp", None) or ("entry" if args.task_fixture else "official")
    if pipe_ramp == "entry" and not args.task_fixture:
        raise ValueError("--pipe-ramp entry requires --task-fixture (a labelled local modified scene)")
    output = Path(args.output)
    if output.exists() and any(output.iterdir()):
        raise ValueError("Simulation output must be a new/empty directory")
    if not np.isfinite(args.seconds) or args.seconds <= 0 or args.seconds > 600:
        raise ValueError("--seconds must be in (0,600]")
    if not np.isfinite(args.spawn).all():
        raise ValueError("--spawn must contain finite initial coordinates")
    initial_yaw = getattr(args, "spawn_yaw", 0.)
    if not np.isfinite(initial_yaw):
        raise ValueError("--spawn-yaw must be finite")
    video = getattr(args, "video", False)
    save_physics = getattr(args, "save_physics", False)
    snapshots = getattr(args, "snapshots", False)
    viewer_fps = getattr(args, "viewer_fps", 60.)
    if not np.isfinite(viewer_fps) or not 1 <= viewer_fps <= 120:
        raise ValueError("--viewer-fps must be in [1,120]")
    output.mkdir(parents=True, exist_ok=True)
    print(f"[go2] 加载场景和 MPC；运行上限 {args.seconds:g} 仿真秒；输出 {output.resolve()}", flush=True)
    task_layout = getattr(args, "task_layout", "finish-area")
    config = simulation_config(args.config, args.task_fixture, task_layout, getattr(args, "max_speed", None),
                               getattr(args, "max_reverse_speed", None))
    course_policy = getattr(args, "course_policy", "auto")
    if course_policy == "auto":
        course_policy = "explore" if task_layout == "near-start" or args.control != "autonomy" else "ordered"
    if course_policy == "ordered" and args.control == "autonomy":
        if not np.allclose(args.spawn, [.8, .8], atol=1e-6, rtol=0.) or abs(initial_yaw) > 1e-6:
            raise ValueError("Ordered full course must start at (0.8,0.8), yaw0; stage tests require --course-policy explore")
        default_route = "configs/local-entry-ramp-route.json" if pipe_ramp == "entry" else "configs/official-course-route.json"
        config.course_route = getattr(args, "course_route", None) or config.course_route or default_route
        if args.config is None:
            config.localization_metric = "point_to_line"
            config.imu_heading_reference = True
            config.allow_margin_escape = True
            # The U-exit is only about 0.62m wide. A 0.10m cell can consume
            # most of its remaining body clearance on the two boundary cells.
            config.course_portal_resolution = .05
            config.map_size = 30.
    else:
        config.course_route = ""
    if args.task_fixture:
        from .mujoco_scene import task_scene
        model = task_scene(root, output, tuple(args.spawn), task_layout,
                           save_model=save_physics or snapshots, pipe_ramp=pipe_ramp)
        scene_kind = "local_entry_ramp_plus_task_objects" if pipe_ramp == "entry" else "official_geometry_plus_local_task_objects"
    else:
        model = mujoco.MjModel.from_binary_path(str(root / "scene/mujoco/competition-course.mjb"))
        scene_kind = "official_mjb"
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[:3] = [*args.spawn, .30]  # Initial reset only; never set root pose in the stepping loop.
    data.qpos[3:7] = np.roll(Rotation.from_euler("z", initial_yaw).as_quat(), 1)
    mujoco.mj_forward(model, data)
    controller = ConvexMPC(model, data, root)
    sensors = SimSensors(model, data, config, args.seed)
    autonomy = Autonomy(config)
    config.save(output / "config.json")
    if autonomy.course is not None:
        (output / "ordered-course.json").write_text(json.dumps(autonomy.course.definition, indent=2, ensure_ascii=False)+"\n")
    recorder = Recorder(output / "recording") if args.record else None
    overview = mujoco.Renderer(model, height=480, width=640) if video else None
    overview_camera = mujoco.MjvCamera()
    overview_camera.distance, overview_camera.azimuth, overview_camera.elevation = 3.8, 140., -30.
    writer = cv2.VideoWriter(str(output / "simulation.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10., (960, 480)) if video else None
    if writer is not None and not writer.isOpened():
        raise RuntimeError("Could not open MuJoCo video encoder")
    viewer = None
    if args.viewer:
        from mujoco import viewer as mjviewer
        viewer = mjviewer.launch_passive(model, data)
        viewer.opt.geomgroup[3] = 1
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 4., 140., -30.
    initial = data.qpos[:3].copy()
    from .evaluation import SimulationEvaluator
    official_model = mujoco.MjModel.from_binary_path(str(root / "scene/mujoco/competition-course.mjb"))
    evaluator = SimulationEvaluator(model, official_model, initial[:2], initial_yaw)
    last_step = Step(Velocity(), controller_pose(), "warming_up", [])
    observations, all_events, history, command_history = 0, [], [], []
    control_times, nav_times, sensor_times, physics_times = [deque(maxlen=10000) for _ in range(4)]
    command, next_nav, next_video, next_physics = Velocity(), .6, 0., 0.
    next_viewer, front, viewer_syncs = 0., None, 0
    overlay_key, camera_updates = None, 0
    ticks, fell = 0, False
    start = time.monotonic()
    status, error, stop_reason = "time_limit", None, "simulation_time_limit"
    runtime = RuntimeLog(output / "runtime.jsonl")
    original_start = np.allclose(args.spawn, [.8, .8], atol=1e-6, rtol=0.)
    run_scope = "original_start" if original_start else "custom_start_stage_test"
    start_label = f"{'ORIGINAL' if original_start else 'STAGE'} START ({args.spawn[0]:g}, {args.spawn[1]:g})"
    scene_label = "LOCAL: RAMP AT ENTRY" if pipe_ramp == "entry" else "SOURCE COURSE"
    runtime.emit("startup", 0., f"{start_label} yaw={initial_yaw:g}; policy={course_policy}; scene={scene_kind} control={args.control}; limit={args.seconds:g}s",
                 scene=scene_kind, control=args.control, time_limit=args.seconds,
                 run_scope=run_scope, initial_world_xy=list(args.spawn), initial_yaw=initial_yaw, course_policy=course_policy)
    runtime.emit("limits", 0., f"前进上限={config.max_vx:g}m/s；倒退上限={config.max_reverse_vx:g}m/s；窄口前进上限={min(config.max_vx, config.passage_max_vx):g}m/s；实际指令仍由避障和制动检查决定",
                 max_forward_mps=config.max_vx, max_reverse_mps=config.max_reverse_vx,
                 max_passage_forward_mps=min(config.max_vx, config.passage_max_vx))
    runtime.emit("resources", 0., f"物理/MPC/导航=CPU；OpenGL={sensors.graphics_device}；视频={video} 原始录制={args.record} 物理归档={save_physics} 快照={snapshots}；窗口刷新上限={viewer_fps:g}Hz",
                 video=video, raw_recording=args.record, physics_archive=save_physics, snapshots=snapshots,
                 graphics_device=sensors.graphics_device)
    previous_state, next_progress, next_snapshot = None, 0., 20.
    try:
        with (output / "trajectory.jsonl").open("w") as log:
            while data.time < args.seconds - 1e-9:
                t = float(data.time)
                if viewer is not None and not viewer.is_running():
                    status = "viewer_closed"
                    stop_reason = "viewer_closed_by_user"
                    break
                if t + 1e-9 >= next_nav:
                    begin = time.monotonic()
                    frame = sensors.observe(controller.measured_velocity())
                    controller.terrain_slope_limit_deg = max(15., float(config.slope_limit(last_step.pose.xy())))
                    controller.observe_terrain(frame.points)
                    sensor_times.append(time.monotonic()-begin)
                    front = frame.image
                    begin = time.monotonic()
                    if args.control == "autonomy":
                        last_step = autonomy.step(frame)
                        controller.observe_forward_motion(last_step.pose, frame.timestamp)
                        command = last_step.velocity
                    else:
                        command = smoke_command(t, args.control, config.max_vx)
                        last_step = Step(command, controller_pose(), f"mpc_{args.control}_profile", [], "scripted MPC test, not autonomous navigation")
                    nav_times.append(time.monotonic() - begin)
                    if recorder:
                        recorder.write(frame)
                    all_events.extend(asdict(e) for e in last_step.events)
                    evaluator.update(data, last_step.events)
                    if last_step.state != previous_state or t >= next_progress:
                        slope = float(np.rad2deg(np.arctan(np.linalg.norm(controller.terrain_plane[:2]))))
                        runtime.emit("progress", t,
                                     f"state={last_step.state} reason={last_step.reason or '-'} "
                                     f"cmd=({command.vx:.2f},{command.vy:.2f},{command.wz:.2f}) "
                                     f"pose_quality={last_step.pose.quality:.2f} terrain_slope={slope:.1f}deg",
                                     state=last_step.state, reason=last_step.reason,
                                     command=asdict(command), pose_quality=last_step.pose.quality,
                                     observed_support_slope_deg=slope)
                        previous_state, next_progress = last_step.state, t + 2.
                    for event in last_step.events:
                        runtime.emit("event", t, f"event={event.kind} " + json.dumps(event.payload, ensure_ascii=False),
                                     event=asdict(event))
                    observations += 1
                    from .cli import step_record
                    record = step_record(last_step, t, nav_times[-1])
                    # Read-only diagnostics, kept outside Step/Autonomy inputs.
                    record["diagnostics_only_world_position"] = data.qpos[:3].tolist()
                    log.write(json.dumps(record, allow_nan=False) + "\n")
                    if snapshots and t >= next_snapshot:
                        from .cli import save_map
                        save_map(autonomy, output)
                        (output / "progress.json").write_text(json.dumps({"simulation_seconds": t,
                            "state": last_step.state, "diagnostics": last_step.diagnostics,
                            "evaluation": evaluator.report(data, False, fell)}, indent=2)+"\n")
                        if "canvas" in locals():
                            cv2.imwrite(str(output / "live-preview.png"), canvas)
                        next_snapshot += 20.
                    next_nav += .1
                if ticks % 3 == 0:
                    begin = time.monotonic()
                    data.ctrl[:] = controller.update(t, command)
                    control_times.append(time.monotonic() - begin)
                # This is the sole state transition during a run. No qpos/qvel
                # overwrite, root forces, mocap following, or kinematic plant.
                begin = time.monotonic()
                mujoco.mj_step(model, data)
                physics_times.append(time.monotonic()-begin)
                ticks += 1
                if ticks % 5 == 0 or args.control == "speed-smoke":
                    evaluator.update(data)
                if save_physics and data.time + 1e-9 >= next_physics:
                    history.append(np.r_[data.time, data.qpos.copy()])
                    command_history.append(command.array())
                    next_physics += .1
                if writer is not None and data.time + 1e-9 >= next_video:
                    base = data.xpos[model.body("base_link").id]
                    overview_camera.lookat[:] = base
                    overview.update_scene(data, camera=overview_camera, scene_option=sensors.option)
                    canvas = np.zeros((480, 960, 3), np.uint8)
                    canvas[:, :640] = cv2.cvtColor(overview.render(), cv2.COLOR_RGB2BGR)
                    if front is not None:
                        canvas[:240, 640:] = front
                    lines = ["MuJoCo 3.17 | Convex MPC", start_label, f"scene: {scene_kind[:29]}",
                             f"t={data.time:.1f}s {last_step.state}",
                             f"cmd {command.vx:.2f} {command.vy:.2f} {command.wz:.2f}",
                             f"localization {last_step.pose.quality:.2f}",
                             "events: " + ",".join(e["kind"] for e in all_events[-4:]),
                             last_step.reason[:40]]
                    if autonomy.course is not None:
                        lines[2] = f"stage: {autonomy.course.stage} wp {autonomy.course.waypoint_index}"
                    for i, text in enumerate(lines):
                        cv2.putText(canvas, text, (650, 265 + 27 * i), cv2.FONT_HERSHEY_SIMPLEX, .40, (220, 230, 220), 1, cv2.LINE_AA)
                    writer.write(canvas)
                    next_video += .1
                now = time.monotonic()
                if viewer is not None and now >= next_viewer:
                    viewport = viewer.viewport
                    if viewport is None or not viewer.is_running():
                        status, stop_reason = "viewer_closed", "viewer_closed_by_user"
                        break
                    viewer.cam.lookat[:] = data.xpos[model.body("base_link").id]
                    phase = autonomy.course.stage if autonomy.course is not None else "exploration"
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                      "Scene\nStart\nStage\nState\nSim time",
                                      f"{scene_label}\n{start_label}\n{phase}\n{last_step.state}\n{data.time:.1f}s"))
                    if front is not None and not getattr(args, "hide_camera", False):
                        key = (observations, viewport.width, viewport.height)
                        if key != overlay_key and viewport.width > 0 and viewport.height > 64:
                            viewer.set_images(camera_inset(front, viewport))
                            overlay_key, camera_updates = key, camera_updates+1
                    viewer.sync()
                    viewer_syncs += 1
                    next_viewer = now+1./viewer_fps
                rpy = Rotation.from_quat(np.roll(data.qpos[3:7], -1)).as_euler("xyz")
                if data.qpos[2] < .13 or max(abs(rpy[:2])) > .8 or not np.isfinite(data.qpos).all():
                    fell, status = True, "fell"
                    stop_reason = "fall_or_invalid_physics_state"
                    break
                if last_step.state in {"finished", "timed_out"}:
                    status = "completed" if last_step.state == "finished" else "timed_out"
                    stop_reason = "mission_finished" if last_step.state == "finished" else "mission_budget_exhausted"
                    break
                if args.realtime:
                    delay = float(data.time) - (time.monotonic() - start)
                    if delay > 0:
                        time.sleep(min(.01, delay))
    except KeyboardInterrupt:
        status, stop_reason = "interrupted", "keyboard_interrupt"
    except Exception as exc:
        status, error = "failed", f"{type(exc).__name__}: {exc}"
        stop_reason = "exception"
        raise
    finally:
        # Navigation runs after a physics/render tick. Include its terminal
        # event in the video even when it falls between two 10 Hz render ticks.
        if "canvas" in locals():
            canvas[240:, 640:] = 0
            terminal_lines = ["MuJoCo 3.17 | Convex MPC", start_label, f"t={data.time:.1f}s {last_step.state}",
                              "events: " + ",".join(e["kind"] for e in all_events[-4:]),
                              f"cmd {command.vx:.2f} {command.vy:.2f} {command.wz:.2f}",
                              last_step.reason[:37]]
            if autonomy.course is not None:
                terminal_lines.append(f"stage: {autonomy.course.stage} wp {autonomy.course.waypoint_index}")
            for i, text in enumerate(terminal_lines):
                cv2.putText(canvas, text, (650, 270 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, .40, (220, 230, 220), 1, cv2.LINE_AA)
            for _ in range(10):
                writer.write(canvas)
        if writer is not None:
            writer.release()
        if viewer and status in {"failed", "interrupted", "viewer_closed"}:
            close_viewer(viewer)
        if overview is not None:
            overview.close()
        sensors.close()
        if recorder:
            recorder.close()
        if snapshots and "canvas" in locals():
            cv2.imwrite(str(output / "preview.png"), canvas)
        if save_physics:
            np.savez_compressed(output / "physics.npz", states=np.asarray(history), commands=np.asarray(command_history))
        if snapshots:
            from .cli import save_map
            save_map(autonomy, output)
        summary = {"status": status, "stop_reason": stop_reason,
                   "run_scope": run_scope, "initial_yaw": initial_yaw,
                   "course_policy": course_policy, "ordered_course": None if autonomy.course is None else autonomy.course.diagnostics(),
                   "task_layout": task_layout if args.task_fixture else None,
                   "mission_complete": last_step.state == "finished", "time_limit_seconds": args.seconds,
                   "error": error, "scene": scene_kind, "control": args.control,
                   "pipe_ramp": pipe_ramp, "modified_course_geometry": pipe_ramp != "official",
                   "physics": "MuJoCo rigid-body dynamics; torque actuators; mj_step only",
                   "controller": "go2-convex-mpc@1c63c6a762779887ab0431fd60db681dede6cb32",
                   "state_estimation": "joint/IMU contact-scheduled kinematics, no plant root pose input",
                   "mujoco": mujoco.__version__, "simulation_seconds": float(data.time),
                   "wall_seconds": time.monotonic() - start, "fell": fell,
                   "mpc_solves": controller.solves, "events": all_events, "mission_state": last_step.state,
                   "mpc_terrain_updates": controller.terrain_updates,
                   "observations": observations, "nav_p95_seconds": float(np.quantile(nav_times, .95)) if nav_times else None,
                   "recording": {"video": video, "raw": args.record, "physics": save_physics, "snapshots": snapshots},
                   "compute": {"physics": "CPU", "mpc": "CPU", "autonomy": "CPU", "blas_threads": 1,
                               "graphics_device": sensors.graphics_device, "viewer_syncs": viewer_syncs,
                               "front_camera_updates": camera_updates},
                   "timing_p95_seconds": {name: float(np.quantile(values, .95)) if values else None for name, values in
                       [("sensors_and_terrain", sensor_times), ("autonomy", nav_times), ("mpc_update", control_times), ("physics_step", physics_times)]},
                   "final_map_resolution_m": config.resolution,
                   "mpc_qp_p95_seconds": float(np.quantile(controller.solve_times, .95)) if controller.solve_times else None,
                   "diagnostics_only": {"initial_world_position": initial.tolist(), "final_world_position": data.qpos[:3].tolist(),
                                        "displacement_m": float(np.linalg.norm(data.qpos[:2] - initial[:2]))},
                   "evaluation": evaluator.report(data, last_step.state == "finished", fell),
                   "official_scene_sha256": hashlib.sha256((root / "scene/mujoco/competition-course.mjb").read_bytes()).hexdigest(),
                   "not_a_competition_score": True}
        summary["full_course_pass"] = bool(pipe_ramp == "official" and original_start and args.control == "autonomy"
                                            and autonomy.course is not None and autonomy.course.route_complete
                                            and summary["evaluation"]["ordered_traversal_verified"]
                                            and summary["evaluation"]["local_task_pass"])
        (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        runtime.emit("stop", float(data.time),
                     f"STOP: {stop_reason}; mission={last_step.state}; fell={fell}; "
                     f"events={[e['kind'] for e in all_events]}; results={output.resolve()}",
                     status=status, stop_reason=stop_reason, mission_complete=last_step.state == "finished", error=error)
        if stop_reason == "mission_finished":
            print("[go2] 任务正常完成：已发出 finish，结束自主控制。", flush=True)
        elif stop_reason == "simulation_time_limit":
            print("[go2] 已到 --seconds 仿真时间上限；这不代表任务完成，请查看 mission_state。", flush=True)
        runtime.close()
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if viewer is not None and viewer.is_running():
        phase = autonomy.course.stage if autonomy.course is not None else "exploration"
        viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                          "Scene\nStart\nStage\nStopped", f"{scene_label}\n{start_label}\n{phase}\n{stop_reason}"))
        viewer.sync()
    finish_viewer(viewer, not getattr(args, "close_viewer_on_stop", False)
                  and status not in {"failed", "interrupted", "viewer_closed"})
    return summary


def controller_pose():
    from .types import Pose
    return Pose()
