"""Offline physical-record inspection with a transparent upper pipe shell.

Only the renderer's draw list changes. Original model, recorded states and
collision geometry stay unchanged. This does not run/control the simulator.
"""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import cv2
import mujoco
import numpy as np

from go2_rescue.mujoco_sim import SimSensors, simulation_config


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("run",type=Path)
    args=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    output=args.run/"inspection.mp4"
    if output.exists():
        p.error("Inspection video already exists")
    states=np.load(args.run/"physics.npz")["states"]
    summary=json.loads((args.run/"summary.json").read_text())
    model=mujoco.MjModel.from_binary_path(str(root/"scene/mujoco/competition-course.mjb"))
    data=mujoco.MjData(model)
    roof={model.geom(f"tunnel2_segment_{i:02d}").id for i in range(12)}
    sensors=SimSensors(model,data,simulation_config())
    renderer=mujoco.Renderer(model,height=480,width=640)
    camera=mujoco.MjvCamera()
    camera.distance,camera.azimuth,camera.elevation=3.5,90.,-40.
    writer=cv2.VideoWriter(str(output),cv2.VideoWriter_fourcc(*"mp4v"),10.,(960,480))
    if not writer.isOpened():
        raise RuntimeError("Cannot open inspection video encoder")
    try:
        for row in states:
            data.qpos[:]=row[1:]
            mujoco.mj_forward(model,data)  # OFFLINE recorded-state display only.
            camera.lookat[:]=data.qpos[:3]
            renderer.update_scene(data,camera=camera,scene_option=sensors.option)
            for geom in renderer.scene.geoms[:renderer.scene.ngeom]:
                if geom.objtype==mujoco.mjtObj.mjOBJ_GEOM and geom.objid in roof:
                    geom.rgba[3]=.12  # Render-only transparency; never model edits.
            canvas=np.zeros((480,960,3),np.uint8)
            canvas[:,:640]=cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR)
            canvas[:240,640:]=sensors.image()  # Solid roof in the physical camera view.
            cv2.putText(canvas,"STAGE TEST | recorded physics replay",(10,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
            cv2.putText(canvas,"Upper shell transparent FOR DISPLAY ONLY",(10,50),cv2.FONT_HERSHEY_SIMPLEX,.50,(0,255,255),1)
            for i,line in enumerate([f"t={row[0]:.1f}s","No collision model edits","Solid front camera above",f"Stage pass: {summary['stage_pass']}","Full course: False","Not autonomous navigation"]):
                cv2.putText(canvas,line,(650,275+28*i),cv2.FONT_HERSHEY_SIMPLEX,.43,(230,230,230),1)
            writer.write(canvas)
            if abs(row[0]-26.)<.051:
                cv2.imwrite(str(args.run/"inspection.png"),canvas)
    finally:
        writer.release()
        renderer.close()
        sensors.close()
    print(output.resolve())


if __name__=="__main__":
    main()
