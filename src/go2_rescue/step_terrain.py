"""Sensor-only single curb model and swing trajectory for MPC development.

Opt-in capability, not a replacement for obstacle collision checks. Coordinates
are the joint/IMU estimator's local frame; no simulator geometry is read.
"""
from __future__ import annotations

import numpy as np


def raised_swing(start, end, duration, clearance=.055):
    """Lift before translation: a high foot must clear the curb's front face."""
    start, end = np.array(start, float), np.array(end, float)
    apex = max(start[2], end[2]) + clearance
    knots = [start, np.r_[start[:2], apex], np.r_[end[:2], apex], end]
    times = np.array([0., .35, .75, 1.]) * duration

    def evaluate(time):
        index = min(2, int(np.searchsorted(times[1:], time, side="right")))
        dt = times[index+1]-times[index]
        s = np.clip((time-times[index])/dt, 0., 1.)
        q = 10*s**3-15*s**4+6*s**5
        dq = (30*s**2-60*s**3+30*s**4)/dt
        ddq = (60*s-180*s**2+120*s**3)/dt**2
        delta = knots[index+1]-knots[index]
        return knots[index]+delta*q, delta*dq, delta*ddq

    return evaluate


class CurbObservation:
    def __init__(self, origin, direction, lower_height):
        self.origin = np.array(origin, float)
        self.direction = np.array(direction, float)
        self.lower = float(lower_height)
        self.upper = self.edge = None
        self.confirmations = 0
        self.entry_complete = False

    def observe(self, points, body_xy):
        along = (points[:, :2]-self.origin)@self.direction
        lateral = (points[:, :2]-body_xy)@np.array([-self.direction[1], self.direction[0]])
        forward = (points[:, :2]-body_xy)@self.direction
        usable = (abs(lateral)<.27)&(forward>.12)&(forward<1.2)
        samples = points[usable]
        distances = along[usable]
        if len(samples)<8:
            return
        heights = samples[:, 2]
        candidates = heights[(heights>self.lower+.12)&(heights<self.lower+.27)]
        if len(candidates)<5:
            return
        bins = np.round(candidates/.01).astype(int)
        values, counts = np.unique(bins, return_counts=True)
        level = values[np.argmax(counts)]*.01
        plateau = abs(heights-level)<.008
        if plateau.sum()<5 or np.ptp(lateral[usable][plateau])<.20:
            return
        upper = float(np.median(heights[plateau]))
        face = (heights>self.lower+.035)&(heights<upper-.025)
        if face.sum()<4:
            return
        edge = float(np.median(distances[face]))
        if self.edge is not None and (abs(edge-self.edge)>.06 or abs(upper-self.upper)>.04):
            return
        self.edge = edge if self.edge is None else .8*self.edge+.2*edge
        self.upper = upper if self.upper is None else .8*self.upper+.2*upper
        self.confirmations += 1

    @property
    def ready(self):
        return self.confirmations>=3

    def height(self, xy):
        distance = (np.asarray(xy)-self.origin)@self.direction
        return self.upper if self.ready and distance>=self.edge else self.lower

    def body_surface(self, xy):
        if not self.ready:
            return self.lower, np.zeros(2)
        distance = (np.asarray(xy)-self.origin)@self.direction-self.edge
        fraction = np.clip((distance+.36)/.64, 0., 1.)
        return self.lower+(self.upper-self.lower)*fraction, self.direction*(self.upper-self.lower)/.64 if 0<fraction<1 else np.zeros(2)
