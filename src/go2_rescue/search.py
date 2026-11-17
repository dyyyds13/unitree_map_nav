"""Map coverage exploration with incidental vision; no simulator dependencies."""
from collections import deque

import numpy as np
from scipy.spatial import cKDTree

from .course import OrderedCourse
from .geometry import rot2, wrap
from .types import Event, Step, Velocity


class ObservedGoal:
    """Reuse the supported-ground local path builder for a direction prior."""
    def __init__(self, target, gate_yaw=None):
        self.target = np.asarray(target, float)
        self.waypoint = {} if gate_yaw is None else {'observed_corridor':True,'yaw':gate_yaw}

    local_path = OrderedCourse.local_path


class CoverageSearch:
    def __init__(self, config):
        self.c = config
        self.motion = deque(maxlen=5)
        self.goal = None
        self.goal_since = 0.
        self.returning = False
        self.finish_index = 0
        self.hold_since = None
        self._region_key = None
        self._region = None

    def coverage(self, grid):
        if not self.c.search_bounds:
            return None, 0., 0.
        bounds = self.c.search_bounds
        key = (grid.n, grid.c.resolution, tuple(grid.origin), tuple(bounds))
        if key != self._region_key:
            x0,x1,y0,y1 = bounds
            centers = grid.origin[:,None]+(np.arange(grid.n)+.5)*grid.c.resolution
            self._region = ((centers[0]>=x0)&(centers[0]<x1))[:,None] & ((centers[1]>=y0)&(centers[1]<y1))[None,:]
            self._region_key = key
        # Empty/overhead-only rays do not classify ground occupancy. Both
        # observed obstacle and observed free cells count as mapped; neither
        # implies a safe standing pose or a camera view of a victim.
        known = grid.seen & ((abs(grid.logodds)>.1)|np.isfinite(grid.floor))
        mapped = float((known & self._region).sum()*grid.c.resolution**2)
        x0,x1,y0,y1 = bounds
        total = float((x1-x0)*(y1-y0))
        return min(1.,mapped/total),mapped,total

    def diagnostics(self, grid):
        fraction,mapped,total = self.coverage(grid)
        return {"returning": self.returning,
                "map_coverage_fraction": fraction,
                "map_coverage_threshold": self.c.search_coverage_threshold,
                "mapped_area_m2": mapped, "exploration_area_m2": total,
                "coverage_threshold_reached": fraction is not None and fraction>=self.c.search_coverage_threshold,
                "observed_floor_area_m2": float((grid.seen & np.isfinite(grid.floor)).sum()*grid.c.resolution**2),
                "coverage_scope": "estimated occupancy within configured public bounds; not victim visibility",
                "official_signals_enabled": False}

    def correct_map(self, grid, corrections):
        """Transform the current goal without restarting its timeout."""
        if not corrections:
            return
        tree = cKDTree([old.xy() for old,new in corrections])
        def corrected(xy):
            xy = np.asarray(xy).reshape(-1,2)
            result = xy.copy()
            if len(xy):
                nearest = tree.query(xy)[1]
                for index in np.unique(nearest):
                    old,new = corrections[int(index)]
                    select = nearest == index
                    result[select] = (xy[select]-old.xy())@rot2(new.yaw-old.yaw).T+new.xy()
            return result
        if self.returning:
            self.goal = None  # Public approach priors remain in map_start.
        elif self.goal is not None:
            self.goal = corrected([self.goal])[0]
        self.hold_since = None
        self.motion.clear()

    def command(self, a, frame, pose, points, events, dt):
        now = frame.timestamp
        self.motion.append((now,pose.xy().copy(),frame.velocity.array().copy()))
        fraction,mapped,total = self.coverage(a.grid)
        reached = fraction is not None and fraction>=self.c.search_coverage_threshold
        reserve = now-a.started >= self.c.mission_seconds-self.c.return_reserve
        if not self.returning and (reached or reserve):
            self.returning = True
            self.goal = None
            # Exploration recovery must not retain goal ownership on return.
            a.recovery_path,a.recovery_index = np.empty((0,3)),0
            a.margin_escape_active = False
            events.append(Event('search_return',now,{
                'reason':'map_coverage_threshold' if reached else 'return_time_reserve',
                'map_coverage_fraction':fraction,'coverage_threshold_reached':reached,
                'mapped_area_m2':mapped,'exploration_area_m2':total}))

        if a.margin_escape_active:
            half = np.array([self.c.footprint_half_length,self.c.footprint_half_width])+self.c.safety_margin
            x,y = np.meshgrid(np.linspace(-half[0],half[0],11),np.linspace(-half[1],half[1],9))
            if a.grid.footprint_safe(pose.xy(),pose.yaw,np.column_stack([x.ravel(),y.ravel()])):
                a.recovery_path,a.recovery_index = np.empty((0,3)),0
                a.margin_escape_active = False
        if not a.margin_escape_active and not len(a.recovery_path):
            route = a.explorer.margin_escape(a.grid,pose)
            if len(route):
                a.recovery_path,a.recovery_index = route,1
                a.recovery_until,a.margin_escape_active = now+20.,True

        # Keep safe recovery manoeuvres until complete; do not choose a new
        # frontier every frame while the locomotion controller is turning.
        if a.recovery_index < len(a.recovery_path):
            while a.recovery_index < len(a.recovery_path):
                waypoint = a.recovery_path[a.recovery_index]
                if np.linalg.norm(waypoint[:2]-pose.xy()) > .12 or abs(wrap(waypoint[2]-pose.yaw)) > .18:
                    break
                a.recovery_index += 1
            if a.recovery_index < len(a.recovery_path) and now < a.recovery_until:
                waypoint = a.recovery_path[a.recovery_index]
                planner = a.escape_dwa if a.margin_escape_active else a.dwa
                if a.margin_escape_active:
                    waypoint = a.recovery_path[-1]
                command = planner.command(pose,frame.velocity,np.array([pose.xy(),waypoint[:2]]),
                    a.grid.obstacle_points(points,pose),a.grid,dt,desired_yaw=float(waypoint[2]),reverse_limit=.15)
                return Step(command,pose,'recovering',events,'search_observed_recovery')
            a.recovery_path,a.recovery_index = np.empty((0,3)),0
            a.margin_escape_active = False
            self.goal = None

        approach_complete = self.finish_index >= len(self.c.search_finish_approach)
        if self.returning and approach_complete and a.finish_goal is None:
            return Step(a.dwa.safe_turn(pose,.25,a.grid),pose,'to_finish',events,'search_reacquire_finish_marker')
        at_visual_finish = a.finish_goal is not None and np.linalg.norm(a.finish_goal-pose.xy()) < .22
        if self.returning and a.finish_goal is not None and (approach_complete or at_visual_finish):
            self.goal = a.finish_goal.copy()
            if np.linalg.norm(self.goal-pose.xy()) < .22:
                error = wrap(np.arctan2(*(a.finish_marker-pose.xy())[::-1])-pose.yaw)
                if abs(error) > .12:
                    return Step(a.dwa.safe_turn(pose,float(np.clip(error/1.5,-.25,.25)),a.grid),
                                pose,'to_finish',events,'search_face_finish')
                motion = np.array([m[2] for m in self.motion])
                displacement = np.array([m[1]-pose.xy() for m in self.motion])
                stable = len(motion)>=3 and np.linalg.norm(motion.mean(axis=0)) < .06 and np.linalg.norm(displacement,axis=1).max()<.025
                visible = now-a.last_finish < .25 and a.finish_observations >= 3
                self.hold_since = (now if self.hold_since is None else self.hold_since) if stable and visible else None
                if self.hold_since is not None and now-self.hold_since >= 1.:
                    events.append(Event('finish',now,{'method':'local_aruco_standoff','official_signal_sent':False}))
                    a.state = 'finished'
                    return Step(Velocity(),pose,'finished',events,'search_finish_confirmed')
                return Step(Velocity(),pose,'to_finish',events,'search_finish_hold')
        self.hold_since = None

        arrival_tolerance = .09 if self.returning else .22
        if self.goal is not None and np.linalg.norm(self.goal-pose.xy()) < arrival_tolerance:
            a.explorer.arrived(a.grid,self.goal,now)
            center = a.grid.cell(self.goal)
            radius = int(np.ceil(.5/a.grid.c.resolution))
            for dx in range(-radius,radius+1):
                for dy in range(-radius,radius+1):
                    if dx*dx+dy*dy <= radius*radius:
                        a.explorer.blacklist[center[0]+dx,center[1]+dy] = now+120.
            self.goal = None
            if self.returning:
                self.finish_index += 1

        if self.goal is None:
            if self.returning:
                approach = self.c.search_finish_approach
                if not approach:
                    return Step(Velocity(),pose,'blocked',events,'finish_not_observed_no_finish_prior')
                self.goal = np.asarray(approach[min(self.finish_index,len(approach)-1)],float)
            else:
                self.goal = a.explorer.choose(a.grid,pose,now,bounds=self.c.search_bounds)
                if self.goal is None:
                    return Step(Velocity(),pose,'blocked',events,'mapping_no_reachable_frontier')
            self.goal_since = now

        a.goal = self.goal.copy()
        gate_index = (len(self.c.search_finish_approach)-1 if self.c.search_finish_gate_index is None
                      else self.c.search_finish_gate_index)
        gate_yaw = self.c.search_finish_gate_yaw if (self.returning and
            self.finish_index == gate_index) else None
        if gate_yaw is not None and abs(wrap(gate_yaw-pose.yaw)) > .15:
            turn = a.dwa.safe_turn(pose,float(np.clip(wrap(gate_yaw-pose.yaw)/1.5,-.3,.3)),a.grid)
            if turn == Velocity() and now-a.last_progress > 3.:
                route = a.explorer.maneuver(a.grid,pose,pose.xy(),goal_yaw=gate_yaw,position_tolerance=.35)
                a.last_progress = now
                if len(route)>1:
                    a.recovery_path,a.recovery_index,a.recovery_until = route,1,now+30.
            return Step(turn,pose,'to_finish',events,'search_align_finish_door')
        path = ObservedGoal(self.goal,gate_yaw).local_path(a.grid,pose)
        a.current_path = path
        measured = frame.velocity
        if len(self.motion)>=3:
            duration = now-self.motion[0][0]
            if duration >= .19:
                velocity = (pose.xy()-self.motion[0][1])/duration
                if np.linalg.norm(velocity)<.6:
                    measured = Velocity(*(rot2(pose.yaw).T@velocity),frame.velocity.wz)
        if len(path):
            planner = a.dwa if gate_yaw is None else a.passage_dwa
            command = planner.command(pose,measured,path,a.grid.obstacle_points(points,pose),
                a.grid,dt,face_path=True,desired_yaw=gate_yaw,
                reverse_limit=.15,forward_limit=.30 if gate_yaw is None else .2)
        else:
            command = Velocity()
        slow = np.linalg.norm(command.array()) < .03
        a.slow_since = (now if a.slow_since is None else a.slow_since) if slow else None
        if (a.slow_since is not None and now-a.slow_since > 3.) or now-a.last_progress > 8.:
            if a.begin_recovery(pose,now):
                return Step(Velocity(),pose,'recovering',events,'search_plan_recovery')
            a.explorer.reject(a.grid,self.goal,now)
            self.goal = None
            a.last_progress = now
        elif now-self.goal_since > 25. and not self.returning:
            a.explorer.reject(a.grid,self.goal,now)
            self.goal = None
        a.state = 'to_finish' if self.returning else 'searching'
        return Step(command,pose,a.state,events,'search_follow_observed_path' if len(path) else 'search_no_observed_path')
