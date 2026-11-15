"""Render a finished run's actual physics trajectory, for diagnostics only."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import cv2
import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    summary = json.loads((args.run / "summary.json").read_text())
    states = np.load(args.run / "physics.npz")["states"]
    if not len(states):
        parser.error("Run contains no physical states")
    scene = args.run / "runtime-scene.mjb"
    if not scene.exists():
        scene = root / "scene/mujoco/competition-course.mjb"
    model = mujoco.MjModel.from_binary_path(str(scene))
    data = mujoco.MjData(model)
    data.qpos[:] = states[-1, 1:1+model.nq]
    mujoco.mj_forward(model, data)
    width, height = 1000, 900
    model.vis.global_.fovy = 45.
    model.vis.global_.ipd = 0.
    model.vis.global_.offwidth, model.vis.global_.offheight = width, height
    camera, option = mujoco.MjvCamera(), mujoco.MjvOption()
    camera.lookat[:] = [6., 5., 0.]
    camera.distance, camera.azimuth, camera.elevation = 17., 90., -90.
    option.geomgroup[3] = 1
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera=camera, scene_option=option)
        overhead = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
        eye = renderer.scene.camera[0]
        origin, forward, up = eye.pos.copy(), eye.forward.copy(), eye.up.copy()
    finally:
        renderer.close()
    right = np.cross(forward, up)
    focal = height/(2*np.tan(np.deg2rad(45.)/2))

    def project(xyz):
        delta = np.asarray(xyz)-origin
        depth = delta@forward
        return np.column_stack([width/2+focal*(delta@right)/depth,
                                height/2-focal*(delta@up)/depth]).round().astype(np.int32)

    # Ground truth appears only in this offline illustration, never in the
    # autonomy input, route selection, MPC or live command generation.
    points = project(states[:, 1:4])
    cv2.polylines(overhead, [points.reshape(-1, 1, 2)], False, (245, 175, 40), 3, cv2.LINE_AA)
    for point, label, color in [(points[0], "S", (50, 230, 80)), (points[-1], "E", (60, 60, 250))]:
        cv2.circle(overhead, tuple(point), 9, color, -1, cv2.LINE_AA)
        cv2.putText(overhead, label, tuple(point+[12, -8]), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
    canvas = np.full((height, width+380, 3), 22, np.uint8)
    canvas[:, :width] = overhead
    evaluation = summary.get("evaluation", {})
    initial = summary["diagnostics_only"]["initial_world_position"]
    stage = np.linalg.norm(np.asarray(initial[:2])-[.8, .8]) > .1
    lines = ["Go2 competition diagnostics", "Actual MuJoCo physics path", "",
             "STAGE TEST" if stage else "ORIGINAL START",
             f"Reset: {initial[0]:.2f}, {initial[1]:.2f}",
             f"State: {summary['mission_state']}", f"Stop: {summary['stop_reason']}",
             f"Simulation: {summary['simulation_seconds']:.1f}s",
             f"Mission finished: {summary['mission_complete']}",
             f"Local check passed: {evaluation.get('local_task_pass', False)}", "",
             f"Full ordered pass: {summary.get('full_course_pass', False)}",
             f"Stages completed: {len(evaluation.get('ordered_stages_completed', []))}/7", "",
             f"Fall: {summary['fell']}",
             f"Obstacle contacts: {evaluation.get('obstacle_contact_episodes', '?')}",
             f"Water entries: {evaluation.get('water_entry_episodes', '?')}",
             f"Out of bounds: {evaluation.get('out_of_bounds_episodes', '?')}", "",
             "First physical terrain contacts:"]
    lines.extend(f"  {key}: {value:.1f}s" for key, value in evaluation.get("first_terrain_contact", {}).items())
    errors = evaluation.get("target_position_errors_m", [])
    lines += ["", f"Target reports: {len(errors)}", *[f"  Error {i+1}: {error:.3f}m" for i, error in enumerate(errors[:8])],
              "", "S = start, E = last physical state", "Blue = actual trajectory", "",
              "Local ball/marker fixture", "Not official scoring", "Atlas/truth used only for display"]
    if stage:
        lines += ["Not a full-course success result"]
    spacing = min(25, (height-50)//max(1, len(lines)))
    for index, line in enumerate(lines):
        cv2.putText(canvas, line, (width+12, 30+spacing*index), cv2.FONT_HERSHEY_SIMPLEX, .45,
                    (225, 230, 225), 1, cv2.LINE_AA)
    output = args.output or args.run / "course-report.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), canvas):
        raise RuntimeError("Failed to write course report")
    print(output.resolve())


if __name__ == "__main__":
    main()
