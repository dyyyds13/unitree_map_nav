"""Explicit partial-stage torque test, never an original-start mission result.

The scripted velocity is a controller diagnostic, not autonomous navigation.
Plant truth is used ONLY for rendering and read-only evaluation/stop checks.
"""
import argparse
import json
import os
from pathlib import Path
import time

os.environ.setdefault("MUJOCO_GL", "egl")
import cv2
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from go2_rescue.mpc_locomotion import ConvexMPC
from go2_rescue.evaluation import SimulationEvaluator
from go2_rescue.mujoco_sim import SimSensors, simulation_config
from go2_rescue.types import Velocity


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True)
    p.add_argument("--seconds", type=float, default=60.)
    p.add_argument("--speed", type=float, default=.045)
    p.add_argument("--spawn-x", type=float, default=8.10)
    args = p.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    model = mujoco.MjModel.from_binary_path(str(root/"scene/mujoco/competition-course.mjb"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[:3] = [args.spawn_x, 6.22, .30]  # Explicit initial partial-stage reset only.
    data.qpos[3:7] = [1., 0., 0., 0.]
    mujoco.mj_forward(model, data)
    controller = ConvexMPC(model, data, root)
    evaluator = SimulationEvaluator(model, model, [args.spawn_x,6.22],0.)
    controller.enable_curb()
    sensors = SimSensors(model, data, simulation_config())
    renderer = mujoco.Renderer(model, height=480, width=640)
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = 3.0, 180., -45.
    video = cv2.VideoWriter(str(output/"simulation.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10., (960,480))
    if not video.isOpened():
        raise RuntimeError("Cannot open video encoder")
    history, diagnostics, contacts = [], [], set()
    entered, exited, fell, stop = False, False, False, "time_limit"
    stop_since = None
    begin = time.monotonic()
    try:
        for tick in range(int(args.seconds/model.opt.timestep)):
            t = float(data.time)
            if tick%50 == 0:
                controller.observe_terrain(sensors.lidar())
            if tick%3 == 0:
                # Feedback uses measured IMU and joint/stance kinematics only.
                settled = controller.curb.entry_complete
                if controller.go2.position[0]>3.0 and stop_since is None:
                    stop_since=t
                command = Velocity(args.speed if t>1. and stop_since is None else 0.,
                                   float(np.clip(-.35*controller.go2.position[1],-.035,.035)) if settled else 0.,
                                   float(np.clip(-.6*controller.go2.rpy[2],-.12,.12)) if settled else 0.)
                data.ctrl[:] = controller.update(t, command)
            mujoco.mj_step(model, data)
            evaluator.update(data)  # Every physics tick; not navigation input.
            if tick%50 == 0:
                pose = data.qpos[:3].copy()
                rpy = Rotation.from_quat(np.roll(data.qpos[3:7], -1)).as_euler("xyz")
                entered |= bool(8.9<pose[0]<10.3 and abs(pose[1]-6.22)<.20 and pose[2]>.40)
                exited |= bool(entered and pose[0]>11.1 and abs(pose[1]-6.22)<.25 and pose[2]<.40)
                for contact in data.contact:
                    for gid in [contact.geom1, contact.geom2]:
                        name = mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,int(gid)) or ""
                        if "tunnel" in name:
                            contacts.add(name)
                record = {"time":t, "world_position":pose.tolist(),"rpy":rpy.tolist(),
                          "kinematic_position":controller.go2.position.tolist(),
                          "curb_confirmations":controller.curb.confirmations,"edge":controller.curb.edge,
                          "entry_complete":controller.curb.entry_complete,
                          "lower":controller.curb.lower,"upper":controller.curb.upper}
                diagnostics.append(record)
                if tick%1000 == 0:
                    print(json.dumps(record),flush=True)
                    (output/"progress.json").write_text(json.dumps(record,indent=2))
                camera.lookat[:] = pose
                renderer.update_scene(data,camera=camera,scene_option=sensors.option)
                frame=np.zeros((480,960,3),np.uint8)
                frame[:,:640]=cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR)
                frame[:240,640:]=sensors.image()
                cv2.putText(frame,f"STAGE TEST ONLY | Pipe2 MPC t={t:.1f}s",(10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
                cv2.putText(frame,f"curb samples={controller.curb.confirmations} entered={entered}",(10,50),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                for i,line in enumerate(["Official map unchanged","Torque MPC | no root writes","Front camera above",f"t={t:.1f}s",f"entered: {entered}",f"exited: {exited}","Not a full-course run"]):
                    cv2.putText(frame,line,(650,270+27*i),cv2.FONT_HERSHEY_SIMPLEX,.45,(230,230,230),1)
                video.write(frame)
                cv2.imwrite(str(output/"preview.png"),frame)
                history.append(np.r_[data.time,data.qpos.copy()])
                if pose[2]<.13 or max(abs(rpy[:2]))>.8:
                    fell,stop=True,"fall"
                    break
                if stop_since is not None and t-stop_since>6.:
                    stop="pipe2_exit_and_stop_verified" if exited and max(abs(rpy[:2]))<.2 and .20<pose[2]<.40 else "stop_not_verified"
                    break
    except KeyboardInterrupt:
        stop="interrupted"
    finally:
        video.release();renderer.close();sensors.close()
        np.savez_compressed(output/"physics.npz",states=np.array(history))
        (output/"diagnostics.json").write_text(json.dumps(diagnostics,indent=2))
        result={"scope":"partial-stage scripted controller test; NOT autonomous or full-course",
                "stop_reason":stop,"pipe2_entered":entered,"pipe2_exited":exited,"fell":fell,
                "seconds":float(data.time),"wall_seconds":time.monotonic()-begin,
                "final_position":data.qpos[:3].tolist(),"terrain_contacts":sorted(contacts),
                "root_overwrites_during_stepping":False,"applied_root_forces":False}
        result["evaluation"]=evaluator.report(data,False,fell)
        result["stage_pass"]=bool(stop=="pipe2_exit_and_stop_verified" and not fell
                                  and evaluator.pipe_passage_verified and not evaluator.incidents
                                  and not evaluator.water_episodes and not evaluator.boundary_episodes)
        result["full_course_pass"]=False
        (output/"summary.json").write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)


if __name__=="__main__":
    main()
