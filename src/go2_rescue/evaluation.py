"""Read-only simulator scoring diagnostics. Never imported by Autonomy/MPC."""
from __future__ import annotations

import mujoco
import numpy as np


class SimulationEvaluator:
    def __init__(self, model, source_model, initial_xy, initial_yaw):
        self.model = model
        self.robot = model.body("base_link").id
        self.robot_geoms = set(np.flatnonzero(model.body_rootid[model.geom_bodyid] == self.robot).tolist())
        self.names = {}
        for i in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
            if name.startswith("course_geom_"):
                name = mujoco.mj_id2name(source_model, mujoco.mjtObj.mjOBJ_GEOM, int(name.rsplit("_", 1)[1])) or name
            self.names[i] = name
        self.water = [i for i, name in self.names.items() if "puddle" in name]
        self.feet = [model.body(f"{leg}_foot").id for leg in ["FL", "FR", "RL", "RR"]]
        # Official model foot spheres FL/FR/RL/RR are attached to the calf
        # bodies; the fixed *_foot frames themselves contain no geom.
        self.foot_geoms = {i for i in self.robot_geoms if self.names[i] in {"FL","FR","RL","RR"}}
        self.initial = np.asarray(initial_xy)
        cs, sn = np.cos(initial_yaw), np.sin(initial_yaw)
        self.map_rotation = np.array([[cs, -sn], [sn, cs]])
        self.previous_contacts = set()
        self.incidents = []
        self.first_terrain_contact = {}
        self.water_episodes = self.boundary_episodes = 0
        self.was_wet = self.was_outside = False
        self.target_errors = []
        self.max_base_height = 0.
        self.map_anchor_timestamp = None
        self.ordered_stages = []
        self.course_waypoint_errors = []
        self.dead_end_entered = self.dead_end_exited = self.rubble_area_visited = False
        self.pipe_inside_x = []
        self.pipe2_floor_contact = False
        self.pipe2_entered_from_west = False
        self.pipe_passage_verified = False

    def update(self, data, events=()):
        now = float(data.time)
        contacts = set()
        contact_details = {}
        for contact in data.contact:
            a, b = int(contact.geom1), int(contact.geom2)
            if (a in self.robot_geoms) == (b in self.robot_geoms):
                continue
            robot, obstacle = (a, b) if a in self.robot_geoms else (b, a)
            name = self.names[obstacle]
            for terrain in ["ascent", "platform", "descent", "tunnel"]:
                if terrain in name:
                    self.first_terrain_contact.setdefault(terrain, now)
            forbidden = any(token in name for token in ["boundary", "partition", "dead_end", "wall", "pile", "boulder"])
            foot = robot in self.foot_geoms
            if "tunnel2_internal_circular_segment_flat_insert" in name and foot:
                self.pipe2_floor_contact = True
            # Roof/shell/body collisions are failures too. Scheduled footholds
            # on the insert and exit threshold are permitted foot support.
            if "tunnel" in name:
                permitted_support = foot and (
                    "internal_circular_segment_flat_insert" in name or "sloped_continuous_threshold" in name
                    or "tunnel2_segment_" in name and contact.pos[2] <= .245)
                forbidden |= not permitted_support
            if forbidden and contact.dist < .001:
                contacts.add(name)
                contact_details[name] = {"robot_geom": self.names[robot],
                                         "robot_body": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, int(self.model.geom_bodyid[robot])),
                                         "contact_world_xyz": contact.pos.tolist(),
                                         "distance_m": float(contact.dist)}
        for name in contacts - self.previous_contacts:
            if len(self.incidents) < 1000:
                self.incidents.append({"timestamp": now, "obstacle": name, **contact_details[name]})
        self.previous_contacts = contacts
        wet = False
        for geom in self.water:
            local = (data.xpos[self.feet] - data.geom_xpos[geom]) @ data.geom_xmat[geom].reshape(3, 3)
            size = self.model.geom_size[geom]
            wet |= bool(((abs(local[:, 0]) < size[0] + .02) & (abs(local[:, 1]) < size[1] + .02) & (abs(local[:, 2]) < .08)).any())
        if wet and not self.was_wet:
            self.water_episodes += 1
        self.was_wet = wet
        base = data.xpos[self.robot]
        outside = bool((base[:2] < 0.).any() or (base[:2] > [12., 10.]).any())
        if outside and not self.was_outside:
            self.boundary_episodes += 1
        self.was_outside = outside
        self.max_base_height = max(self.max_base_height, float(base[2]))
        # Independent fixed-course audit; this state is never sent to autonomy.
        self.dead_end_entered |= bool(3.7 < base[0] < 4.8 and 3.6 < base[1] < 5.2)
        self.dead_end_exited |= bool(self.dead_end_entered and base[1] > 5.8)
        self.rubble_area_visited |= bool(self.dead_end_exited and 5.6 < base[0] < 10.5 and 2.25 < base[1] < 4.7)
        # User explicitly requires circular tunnel2 west-to-east. The old
        # tunnel3 passage must never satisfy this independent physical audit.
        if 8.0 < base[0] < 8.5 and abs(base[1]-6.22) < .25:
            self.pipe2_entered_from_west = True
        if self.pipe2_entered_from_west and 8.85 < base[0] < 10.45 and abs(base[1]-6.22) < .24 and .40 < base[2] < .68:
            self.pipe_inside_x.append(float(base[0]))
        if (self.pipe_inside_x and self.pipe2_floor_contact and 10.95 < base[0] < 11.5
                and abs(base[1]-6.22) < .28 and base[2] < .40
                and max(self.pipe_inside_x)-min(self.pipe_inside_x) > 1.15):
            self.pipe_passage_verified = True
        for event in events:
            if event.kind == "start":
                # Localization creates its origin at the first accepted scan,
                # after controller warmup. Score in that frame, not the earlier
                # reset pose. This transform stays exclusively in diagnostics.
                self.initial = data.xpos[self.robot, :2].copy()
                rotation = data.xmat[self.robot].reshape(3, 3)
                yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
                cs, sn = np.cos(yaw), np.sin(yaw)
                self.map_rotation = np.array([[cs, -sn], [sn, cs]])
                self.map_anchor_timestamp = now
            if event.kind == "course_waypoint":
                expected = self.initial+self.map_rotation@np.asarray(event.payload["xy"])
                self.course_waypoint_errors.append(float(np.linalg.norm(base[:2]-expected)))
            if event.kind == "course_stage_complete":
                self.ordered_stages.append(event.payload["stage"])
            if event.kind == "target":
                try:
                    actual = data.body("local_football").xpos[:2]
                except KeyError:
                    continue
                report = self.initial + self.map_rotation @ np.asarray(event.payload["position"])
                self.target_errors.append(float(np.linalg.norm(report - actual)))

    def report(self, data, mission_complete, fell):
        xy = data.xpos[self.robot, :2]
        inside_finish = bool(9.2 < xy[0] < 12. and 7.1 < xy[1] < 10.)
        from .course import STAGES
        terrain = self.first_terrain_contact
        terrain_order = all(k in terrain for k in ["ascent", "platform", "descent"]) and terrain["ascent"] < terrain["platform"] < terrain["descent"]
        ordered_verified = bool(self.ordered_stages == STAGES and terrain_order and self.dead_end_exited
                                and self.rubble_area_visited and self.pipe_passage_verified)
        return {"scope": "local scene diagnostics; not official scoring",
                "ordered_stages_completed": self.ordered_stages.copy(), "ordered_traversal_verified": ordered_verified,
                "dead_end_entered": bool(self.dead_end_entered), "dead_end_exited": bool(self.dead_end_exited),
                "rubble_area_visited": bool(self.rubble_area_visited), "pipe_passage_verified": self.pipe_passage_verified,
                "required_pipe": "tunnel2", "pipe2_floor_contact": self.pipe2_floor_contact,
                "course_waypoint_errors_m": self.course_waypoint_errors.copy(),
                "obstacle_contact_episodes": len(self.incidents), "contacts": self.incidents,
                "water_entry_episodes": self.water_episodes, "out_of_bounds_episodes": self.boundary_episodes,
                "first_terrain_contact": self.first_terrain_contact, "max_base_height_m": self.max_base_height,
                "target_position_errors_m": self.target_errors, "inside_finish_enclosure": inside_finish,
                "map_anchor_timestamp": self.map_anchor_timestamp, "map_anchor_world_xy": self.initial.tolist(),
                "target_error_p90_m": float(np.quantile(self.target_errors, .9)) if self.target_errors else None,
                "local_task_pass": bool(mission_complete and self.target_errors and not fell and inside_finish
                                        and max(self.target_errors) <= .5
                                        and not self.incidents and not self.water_episodes and not self.boundary_episodes)}
