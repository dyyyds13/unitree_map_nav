"""Autonomous local entry-ramp PARTIAL STAGE, no video or truth control."""
from pathlib import Path
import argparse,json,time
from dataclasses import asdict
from collections import deque
import os
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from threadpoolctl import threadpool_limits
from go2_rescue.mujoco_scene import task_scene
from go2_rescue.mujoco_sim import simulation_config,SimSensors
from go2_rescue.mpc_locomotion import ConvexMPC
from go2_rescue.controller import Autonomy
from go2_rescue.evaluation import SimulationEvaluator
from go2_rescue.types import Velocity

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True,type=Path);p.add_argument('--seconds',type=float,default=65.)
 p.add_argument('--spawn',nargs=2,type=float,default=[7.25,6.22],metavar=('X','Y'));p.add_argument('--seed',type=int,default=0)
 p.add_argument('--route-y-bias',type=float,default=0.,help='Synthetic error in route prior, not robot truth')
 a=p.parse_args();out=a.output
 if not np.isfinite(a.seconds) or not 0<a.seconds<=120 or not np.isfinite(a.spawn).all() or not np.isfinite(a.route_y_bias):p.error('Invalid duration/spawn')
 if out.exists() and any(out.iterdir()):p.error('Use new/empty output')
 out.mkdir(parents=True,exist_ok=True);root=Path(__file__).resolve().parents[1]
 c=simulation_config();c.map_size=12.;c.resolution=.05;c.localization_metric='point_to_line';c.imu_heading_reference=True
 route=json.loads((root/'configs/local-entry-ramp-route.json').read_text())
 # Stage reset has a different start-relative frame from the full course.
 offset=np.array(a.spawn)-.8
 for stage in route['stages']:
  for w in stage['waypoints']:w['xy']=(np.array(w['xy'])-offset+[0.,a.route_y_bias]).tolist()
 for r in route['terrain_slope_regions']:
  b=np.array(r['bounds']);r['bounds']=(b-offset[[0,0,1,1]]).tolist()
 rp=out.resolve()/'stage-route.json';rp.write_text(json.dumps(route,indent=2));c.course_route=str(rp)
 model=task_scene(root,out,tuple(a.spawn),pipe_ramp='entry',save_model=False)
 data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,0)
 data.qpos[:3]=[*a.spawn,.30] # Initial reset only, no pose writes during stepping.
 mujoco.mj_forward(model,data)
 mpc=ConvexMPC(model,data,root);sensors=SimSensors(model,data,c,a.seed);autonomy=Autonomy(c);autonomy.course.stage_index=5
 official=mujoco.MjModel.from_binary_path(str(root/'scene/mujoco/competition-course.mjb'))
 evaluator=SimulationEvaluator(model,official,a.spawn,0.)
 command=Velocity();next_nav=.6;ticks=0;entered=False;fell=False;stable_since=None;last=None;begin=time.monotonic();reason='time_limit'
 stop_history=deque(maxlen=501);stop_speed=float('inf')
 try:
  with (out/'trajectory.jsonl').open('w') as log:
   while data.time<a.seconds:
    t=float(data.time)
    if t+1e-9>=next_nav:
     f=sensors.observe(mpc.measured_velocity());last=autonomy.step(f)
     mpc.terrain_slope_limit_deg=max(15.,float(c.slope_limit(last.pose.xy())))
     mpc.observe_forward_motion(last.pose,t)
     mpc.observe_terrain(f.points);command=last.velocity
     log.write(json.dumps({'timestamp':t,'state':last.state,'reason':last.reason,'pose':asdict(last.pose),'velocity':asdict(command),
       'course':autonomy.course.diagnostics(),'mpc_plane':mpc.terrain_plane.tolist(),
       'mpc_position':mpc.go2.position.tolist(),'mpc_rpy':mpc.go2.rpy.tolist(),
       'diagnostics_only_world_position':data.qpos[:3].tolist()})+'\n');log.flush()
     if ticks%500<50:print(round(t,1),last.reason,'wp',autonomy.course.waypoint_index,'cmd',command,'xy',last.pose.xy(),flush=True)
     next_nav+=.1
    if ticks%3==0:data.ctrl[:]=mpc.update(t,command)
    mujoco.mj_step(model,data);ticks+=1;evaluator.update(data)
    # Truth below is read-only scoring/termination, never a control input.
    xy=data.qpos[:2];entered |= bool(8.9<xy[0]<10.3 and abs(xy[1]-6.22)<.2 and data.qpos[2]>.40)
    stop_history.append((float(data.time),xy.copy()))
    if len(stop_history)>1:stop_speed=float(np.linalg.norm(xy-stop_history[0][1])/(data.time-stop_history[0][0]))
    rpy=Rotation.from_quat(np.roll(data.qpos[3:7],-1)).as_euler('xyz')
    if data.qpos[2]<.13 or abs(rpy[:2]).max()>np.deg2rad(45):fell=True;reason='fell';break
    if last is not None and last.reason=='ordered_pipe2_exit_requires_validated_descent_capability' and entered:
     stable_since=t if stable_since is None else stable_since
     if t-stable_since>2. and stop_speed<.03:reason='entry_complete_exit_guard_stopped';break
 except KeyboardInterrupt:reason='interrupted'
 finally:
  sensors.close();e=evaluator.report(data,False,fell)
  passed=bool(reason=='entry_complete_exit_guard_stopped' and not fell and not e['obstacle_contact_episodes'] and not e['water_entry_episodes'] and not e['out_of_bounds_episodes'])
  s={'scope':'custom-reset autonomous PARTIAL STAGE, local modified map, not full course','spawn':a.spawn,'seed':a.seed,'stop_reason':reason,'stage_pass':passed,'full_course_pass':False,'entered':entered,'fell':fell,'simulation_seconds':float(data.time),'wall_seconds':time.monotonic()-begin,'evaluation':e,'final_world_position_diagnostics':data.qpos[:3].tolist(),'diagnostics_only_stop_speed_mps':stop_speed}
  (out/'summary.json').write_text(json.dumps(s,indent=2));c.save(out/'config.json');print(json.dumps(s,indent=2))
 return 0 if passed else 1
if __name__=='__main__':
 with threadpool_limits(limits=1,user_api='blas'):raise SystemExit(main())
