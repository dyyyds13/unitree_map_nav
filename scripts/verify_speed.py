"""Read-only speed/stop check for a scripted MuJoCo speed-smoke recording."""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    args=parser.parse_args()
    summary=json.loads((args.run/"summary.json").read_text())
    config=json.loads((args.run/"config.json").read_text())
    if summary["control"]!="speed-smoke":
        parser.error("This check is for scripted speed-smoke only, not an autonomous mission")
    with np.load(args.run/"physics.npz") as archive:
        states,commands=archive["states"],archive["commands"]
    if len(states)<3 or len(commands)!=len(states):
        parser.error("Insufficient or mismatched physical records")
    time=states[:,0]
    xy=states[:,1:3]
    yaw=summary["initial_yaw"]
    forward=np.array([np.cos(yaw),np.sin(yaw)])
    side=np.array([-forward[1],forward[0]])
    dt=np.diff(time)
    projected=np.diff(xy,axis=0)@forward/dt
    requested=config["max_vx"]
    cruise=(commands[:-1,0]>=.95*requested)&(time[:-1]>1.5)
    mean=float(np.average(projected[cruise],weights=dt[cruise])) if cruise.any() else None
    stopped=(time>=time[-1]-1.)&(np.linalg.norm(commands,axis=1)<.02)
    stop_speed=None
    if stopped.sum()>=5:
        stop_speed=float(np.linalg.norm([np.polyfit(time[stopped],xy[stopped,i],1)[0] for i in range(2)]))
    lateral=float(np.max(abs((xy-xy[0])@side)))
    evaluation=summary["evaluation"]
    passed=bool(time[-1]>=7. and mean is not None and .8*requested<=mean<=1.3*requested
                and stop_speed is not None and stop_speed<.08 and lateral<.12
                and not summary["fell"] and not evaluation["obstacle_contact_episodes"]
                and not evaluation["water_entry_episodes"] and not evaluation["out_of_bounds_episodes"])
    result={"scope":"scripted controller speed stage, NOT autonomous/full-course/hardware validation",
            "requested_forward_mps":requested,"actual_cruise_mean_mps":mean,
            "actual_cruise_p95_mps":float(np.quantile(projected[cruise],.95)) if cruise.any() else None,
            "stopped_net_speed_last_second_mps":stop_speed,"max_lateral_deviation_m":lateral,
            "speed_stage_pass":passed,"full_course_pass":False,
            "obstacle_contact_episodes":evaluation["obstacle_contact_episodes"],"fell":summary["fell"]}
    (args.run/"speed-check.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))
    return 0 if passed else 1


if __name__=="__main__":
    raise SystemExit(main())
