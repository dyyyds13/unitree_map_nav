"""Display the explicit nominal route on the unchanged public course (offline)."""
import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import cv2
import mujoco
import numpy as np


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", type=Path, default=root / "configs/official-course-route.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    definition = json.loads(args.route.read_text())
    source = root / "scene/mujoco/competition-course.mjb"
    model = mujoco.MjModel.from_binary_path(str(source))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[:3] = [.8, .8, .30]
    mujoco.mj_forward(model, data)
    model.vis.global_.offwidth, model.vis.global_.offheight = 1000, 900
    model.vis.global_.ipd, model.vis.global_.fovy = 0., 45.
    camera, option = mujoco.MjvCamera(), mujoco.MjvOption()
    camera.lookat[:] = [6., 5., 0.]
    camera.distance, camera.azimuth, camera.elevation = 17., 90., -90.
    option.geomgroup[3] = 1
    renderer = mujoco.Renderer(model, height=900, width=1000)
    try:
        renderer.update_scene(data, camera=camera, scene_option=option)
        canvas = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
        eye = renderer.scene.camera[0]
        origin, forward, up = eye.pos.copy(), eye.forward.copy(), eye.up.copy()
    finally:
        renderer.close()
    right = np.cross(forward, up)
    focal = 900/(2*np.tan(np.deg2rad(45.)/2))

    def project(xy):
        xyz = np.column_stack([xy, np.full(len(xy), .05)])
        delta = xyz-origin; depth = delta@forward
        return np.column_stack([500+focal*(delta@right)/depth, 450-focal*(delta@up)/depth]).round().astype(int)

    palette = [(255, 170, 40), (255, 245, 60), (0, 170, 255), (90, 80, 255),
               (240, 100, 230), (80, 230, 90), (240, 245, 245)]
    start = np.asarray(definition["assumed_start_xy"])
    previous = start
    legend = ["PLANNED ROUTE - not a completed run", "Fixed public map, ordered V1", ""]
    for stage_index, stage in enumerate(definition["stages"]):
        xy = np.array([w["xy"] for w in stage["waypoints"]])+start
        pixels = project(np.vstack([previous, xy]));color = palette[stage_index]
        for a, b in zip(pixels[:-1], pixels[1:]):
            cv2.arrowedLine(canvas, tuple(a), tuple(b), color, 2, cv2.LINE_AA, tipLength=.16)
        for p in pixels[1:]:
            cv2.circle(canvas, tuple(p), 3, color, -1, cv2.LINE_AA)
        mid = pixels[len(pixels)//2]
        cv2.putText(canvas, str(stage_index+1), tuple(mid+[7, -7]), cv2.FONT_HERSHEY_SIMPLEX, .7, color, 2)
        legend.append(f"{stage_index+1}. {stage['name']}")
        previous = xy[-1]
    panel = np.full((900, 380, 3), 22, np.uint8)
    legend += ["", "Start: (0.8, 0.8), yaw0", "Dead end: enter, inspect, back out", "Rubble: avoid rocks, pass region",
               "Required pipe: circular tunnel2", "22cm curb: controller experimental", "", "Online pose comes from sensors",
               "Ball coordinates are not in route", "No skipped stages / no teleport", "",
               "Official collision geometry unchanged", "Source SHA256:", hashlib.sha256(source.read_bytes()).hexdigest()[:32],
               "", "Arrows are navigation priors only", "Actual completion needs physics logs"]
    for i, line in enumerate(legend):
        color = palette[i-3] if 3 <= i < 10 else (230, 230, 230)
        cv2.putText(panel, line, (12, 30+28*i), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), np.hstack([canvas, panel]))
    print(args.output.resolve())


if __name__ == "__main__":
    main()
