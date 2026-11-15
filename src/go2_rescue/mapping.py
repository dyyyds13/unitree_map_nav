from __future__ import annotations

import numpy as np
import cv2
from scipy.ndimage import binary_dilation

from .config import Config
from .geometry import bresenham, transform_xy
from .types import Pose
from .terrain import ground_surface, interpolate_surface


class GridMap:
    """Observed free/occupied space plus floor/ceiling evidence.

    Empty rays never establish safe floor support. A cell needs ground returns
    before it becomes traversable; this conservatively rejects unobserved drops.
    """

    def __init__(self, config: Config):
        self.c = config
        self.n = int(np.ceil(config.map_size / config.resolution))
        self.origin = np.array([-config.map_size / 2, -config.map_size / 2])
        self.clear()

    def clear(self):
        shape = (self.n, self.n)
        self.logodds = np.zeros(shape, dtype=np.float32)
        self.seen = np.zeros(shape, dtype=bool)
        self.floor = np.full(shape, np.nan, dtype=np.float32)
        self.ceiling = np.full(shape, np.inf, dtype=np.float32)
        self.headroom = np.full(shape, np.inf, dtype=np.float32)
        self.hazard_votes = np.zeros(shape, dtype=np.uint8)
        self._has_hazards = False
        self.ground_height = 0.
        self._free_cache = None
        self._surface_safe = None

    def cell(self, xy) -> tuple[int, int]:
        if not np.isfinite(np.asarray(xy)[:2]).all():
            raise ValueError("Non-finite map coordinate")
        ij = np.floor((np.asarray(xy)[:2] - self.origin) / self.c.resolution).astype(int)
        return int(ij[0]), int(ij[1])

    def world(self, cell) -> np.ndarray:
        return self.origin + (np.asarray(cell) + .5) * self.c.resolution

    def inside(self, cell) -> bool:
        return 0 <= cell[0] < self.n and 0 <= cell[1] < self.n

    def update(self, points: np.ndarray, pose: Pose):
        start = self.cell(pose.xy())
        if not self.inside(start):
            return
        xy = transform_xy(points, pose)
        raw_heights = points[:, 2] + self.c.base_height
        cells = np.floor((xy - self.origin) / self.c.resolution).astype(int)
        inside = (cells >= 0).all(axis=1) & (cells < self.n).all(axis=1)
        # Align successive local elevation observations to overlapping measured
        # terrain, rather than pinning every scan's floor to world z=0.
        indices = np.flatnonzero(inside)
        if len(indices):
            previous = self.floor[tuple(cells[indices].T)]
            offsets = previous - raw_heights[indices]
            consistent = np.isfinite(offsets) & (abs(offsets - self.ground_height) < .08)
            if consistent.sum() >= 5:
                self.ground_height = float(np.median(offsets[consistent]))
        heights = raw_heights + self.ground_height
        samples = ground_surface(np.column_stack([xy[inside], heights[inside]]),
                                 self.ground_height, self.c.resolution,
                                 self.c.ground_max_gap, self.c.slope_limit)
        if len(samples):
            lo = np.maximum(0, np.floor((samples[:, :2].min(axis=0) - self.origin) / self.c.resolution).astype(int))
            hi = np.minimum(self.n - 1, np.floor((samples[:, :2].max(axis=0) - self.origin) / self.c.resolution).astype(int))
            ii, jj = np.mgrid[lo[0]:hi[0]+1, lo[1]:hi[1]+1]
            query_cells = np.column_stack([ii.ravel(), jj.ravel()])
            query_xy = self.origin + (query_cells + .5) * self.c.resolution
            z = interpolate_surface(samples, query_xy, self.c.ground_max_gap, self.c.slope_limit)
            valid = np.isfinite(z)
            observed_cells = np.floor((samples[:, :2] - self.origin) / self.c.resolution).astype(int)
            floor_cells = np.vstack([query_cells[valid], observed_cells])
            floor_z = np.r_[z[valid], samples[:, 2]]
            local_limits = self.c.slope_limit(self.origin+(floor_cells+.5)*self.c.resolution)
            for cell, height, local_limit in zip(floor_cells, floor_z, local_limits):
                ij = tuple(cell)
                old = self.floor[ij]
                # An occluded floor does not move up to a later roof return.
                # Keep established lower-envelope support when a connected
                # curved wall/roof is mistakenly included in ground samples.
                # The higher hit is still classified for obstacles/headroom.
                if local_limit > self.c.max_slope_deg and np.isfinite(old) and height > old+self.c.max_step_height:
                    continue
                self.floor[ij] = height if not np.isfinite(old) else .5 * old + .5 * height
                # A return previously classified without floor support may
                # later be confirmed as the ramp itself, not an overhead roof.
                if self.ceiling[ij] <= height + .025:
                    self.ceiling[ij] = self.headroom[ij] = np.inf
                else:
                    self.headroom[ij] = self.ceiling[ij]-self.floor[ij]
        self._free_cache = None
        self._surface_safe = None
        floor_at_hit = np.full(len(points), np.nan)
        if len(indices):
            floor_at_hit[indices] = self.floor[tuple(cells[indices].T)]
        relative_height = heights - np.where(np.isfinite(floor_at_hit), floor_at_hit, self.ground_height)
        # Distinct 3D rays may traverse one cell: free updates must not erase a
        # same-scan obstacle endpoint. Apply free evidence before hit evidence.
        free, hits = set(), set()
        for p, height, relative in zip(xy, heights, relative_height):
            end = self.cell(p)
            if not self.inside(end):
                continue
            obstacle = self.c.min_obstacle_height < relative < self.c.body_clearance
            ray = list(bresenham(start, end))
            # Only robot-height returns establish horizontal visibility. Ground
            # rays are also useful, but overhead rays cannot clear low obstacles.
            if relative < self.c.body_clearance:
                free.update(ray[:-1])
                if not obstacle:
                    free.add(end)
            if obstacle:
                hits.add(end)
            self.seen[end] = True
            if relative > max(self.c.min_obstacle_height, self.c.base_height-.05):
                # Clearance is relative to a measured support surface. A far
                # unclassified ramp must not leave a permanent fictitious roof.
                # Low obstacles belong in the reversible occupancy layer.
                # Treating a 10cm rock/ramp edge as a roof left an irreversible
                # low ceiling after later rays had correctly cleared the cell.
                if np.isfinite(self.floor[end]):
                    self.ceiling[end] = min(self.ceiling[end], height)
                    self.headroom[end] = self.ceiling[end]-self.floor[end]
        for cell in free - hits:
            self.logodds[cell] = max(-4., self.logodds[cell] - .45)
            self.seen[cell] = True
        for cell in hits:
            self.logodds[cell] = min(4., self.logodds[cell] + 1.25)
        # The standing footprint is direct evidence of local support.
        radius = int(np.ceil(self.c.robot_radius / self.c.resolution))
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                ij = (start[0] + dx, start[1] + dy)
                if self.inside(ij) and np.hypot(dx, dy) * self.c.resolution <= self.c.robot_radius:
                    self.seen[ij] = True
                    if ij not in hits:
                        self.logodds[ij] = min(-1., self.logodds[ij])
                        if not np.isfinite(self.floor[ij]):
                            self.floor[ij] = self.ground_height

    def obstacle_points(self, points, pose):
        """Use the same local elevation model in DWA as in the global map."""
        xy = transform_xy(points, pose)
        cells = np.floor((xy - self.origin) / self.c.resolution).astype(int)
        inside = (cells >= 0).all(axis=1) & (cells < self.n).all(axis=1)
        floor = np.full(len(points), self.ground_height)
        ix = np.flatnonzero(inside)
        observed = self.floor[tuple(cells[ix].T)]
        floor[ix] = np.where(np.isfinite(observed), observed, self.ground_height)
        height = points[:, 2] + self.c.base_height + self.ground_height - floor
        return points[(height > self.c.min_obstacle_height) & (height < self.c.body_clearance)]

    def add_hazard(self, polygon):
        cells = np.array([self.cell(p) for p in polygon])
        mask = np.zeros_like(self.hazard_votes)
        cv2.fillPoly(mask, [cells[:, ::-1].astype(np.int32)], 1)
        # Account for monocular boundary projection and sparse lidar sampling.
        margin = max(1, int(np.ceil(.08 / self.c.resolution)))
        mask = cv2.dilate(mask, np.ones((2*margin+1, 2*margin+1), np.uint8))
        self.hazard_votes = np.minimum(10, self.hazard_votes + mask)
        self._has_hazards = bool((self.hazard_votes >= 2).any())
        self._free_cache = self._surface_safe = None

    def traversable(self) -> np.ndarray:
        if self._free_cache is not None:
            return self._free_cache
        occupied = self.logodds > .5
        hazards = self.hazard_votes >= 2
        # Fill small sampling gaps only next to measured floor; never invent a
        # broad supported surface from a wall-only scan.
        support = binary_dilation(np.isfinite(self.floor), iterations=1)
        # Fit the tangent around each cell without blending a neighbouring wall
        # or raised rock into the flat floor beside it. Otherwise slope masking
        # followed by robot inflation effectively counts the wall twice.
        # Unobserved map padding has no height gradients. Work on the measured
        # bounding window plus the full two-cell stencil, preserving the same
        # NaN boundaries while avoiding millions of empty-cell operations.
        rows, columns = np.nonzero(np.isfinite(self.floor))
        if len(rows):
            window = (slice(max(0, rows.min()-2), min(self.n, rows.max()+3)),
                      slice(max(0, columns.min()-2), min(self.n, columns.max()+3)))
        else:
            window = (slice(0, 1), slice(0, 1))
        floor = self.floor[window]
        ii, jj = np.mgrid[window[0], window[1]]
        xy = self.origin+(np.stack([ii, jj], axis=-1)+.5)*self.c.resolution
        slope_limits = self.c.slope_limit(xy)
        cxx = np.zeros_like(floor, dtype=float)
        cyy, cxy, cxz, cyz = [cxx.copy() for _ in range(4)]
        step_edge = np.zeros_like(floor, dtype=bool)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if not (dx or dy):
                    continue
                neighbour = np.roll(floor, (-dx, -dy), axis=(0, 1))
                if dx > 0:
                    neighbour[-dx:, :] = np.nan
                elif dx < 0:
                    neighbour[:-dx, :] = np.nan
                if dy > 0:
                    neighbour[:, -dy:] = np.nan
                elif dy < 0:
                    neighbour[:, :-dy] = np.nan
                delta = neighbour - floor
                distance = np.hypot(dx, dy) * self.c.resolution
                local_limit = np.minimum(slope_limits, np.roll(slope_limits, (-dx, -dy), axis=(0, 1)))
                continuous = np.isfinite(delta) & (abs(delta) <= np.tan(np.deg2rad(local_limit)) * distance + .008)
                x, y = dx * self.c.resolution, dy * self.c.resolution
                cxx += continuous * x*x
                cyy += continuous * y*y
                cxy += continuous * x*y
                dz = np.where(continuous, delta, 0.)
                cxz += dz * x
                cyz += dz * y
                if max(abs(dx), abs(dy)) == 1:
                    step_edge |= np.isfinite(delta) & (delta < -self.c.max_step_height)
        determinant = cxx * cyy - cxy * cxy
        valid = determinant > 1e-8
        gx = np.divide(cxz * cyy - cyz * cxy, determinant, out=np.zeros_like(cxx), where=valid)
        gy = np.divide(cyz * cxx - cxz * cxy, determinant, out=np.zeros_like(cxx), where=valid)
        steep = np.zeros_like(self.seen)
        steep[window] = (np.hypot(gx, gy) > np.tan(np.deg2rad(slope_limits))) | step_edge
        insufficient_headroom = self.headroom < self.c.body_clearance
        body_radius = min(self.c.footprint_half_length, self.c.footprint_half_width) if self.c.footprint_half_length else self.c.robot_radius
        clearance_radius = body_radius + self.c.safety_margin
        radius = int(np.ceil(clearance_radius / self.c.resolution))
        xx, yy = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        disk = (xx*xx + yy*yy) * self.c.resolution**2 <= clearance_radius**2
        inflated = binary_dilation(occupied | steep | insufficient_headroom, structure=disk)
        foot_radius = min(self.c.hazard_foot_half_length, self.c.hazard_foot_half_width) if self.c.hazard_foot_half_length else clearance_radius
        fr = int(np.ceil(foot_radius/self.c.resolution))
        fx, fy = np.ogrid[-fr:fr+1, -fr:fr+1]
        foot_disk = (fx*fx+fy*fy)*self.c.resolution**2 <= foot_radius**2
        inflated_hazards = binary_dilation(hazards, structure=foot_disk)
        self._surface_safe = self.seen & (self.logodds < 0.) & support & ~(occupied | steep | insufficient_headroom)
        allowed = self.seen & (self.logodds < 0.) & support & ~inflated & ~inflated_hazards
        allowed[[0, -1], :] = False
        allowed[:, [0, -1]] = False
        self._free_cache = allowed
        return allowed

    def footprint_safe(self, xy, yaw, offsets):
        """Check the complete oriented footprint, including ground support."""
        from .geometry import rot2
        if self._surface_safe is None:
            self.traversable()
        points = offsets @ rot2(yaw).T + xy
        cells = np.floor((points - self.origin) / self.c.resolution).astype(int)
        if (cells < 0).any() or (cells >= self.n).any():
            return False
        if not self._surface_safe[tuple(cells.T)].all():
            return False
        return bool(self.hazard_contacts_safe(np.asarray(xy)[None], np.array([yaw]), offsets)[0])

    def hazard_contacts_safe(self, xy, yaw, body_offsets):
        if not self._has_hazards:
            return np.ones(len(xy), bool)
        offsets = body_offsets
        if self.c.hazard_foot_half_length:
            xx, yy = np.meshgrid(np.linspace(-self.c.hazard_foot_half_length, self.c.hazard_foot_half_length, 7),
                                 np.linspace(-self.c.hazard_foot_half_width, self.c.hazard_foot_half_width, 7))
            offsets = np.column_stack([xx.ravel(), yy.ravel()])
        cs, sn = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        x = xy[:, 0, None]+cs*offsets[None, :, 0]-sn*offsets[None, :, 1]
        y = xy[:, 1, None]+sn*offsets[None, :, 0]+cs*offsets[None, :, 1]
        i = np.floor((x-self.origin[0])/self.c.resolution).astype(int)
        j = np.floor((y-self.origin[1])/self.c.resolution).astype(int)
        inside = ((i >= 0)&(i < self.n)&(j >= 0)&(j < self.n)).all(axis=1)
        return inside & (self.hazard_votes[np.clip(i, 0, self.n-1), np.clip(j, 0, self.n-1)] < 2).all(axis=1)

    def footprints_safe(self, xy, yaw, offsets):
        if self._surface_safe is None:
            self.traversable()
        cs, sn = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        x = xy[:, 0, None] + cs * offsets[None, :, 0] - sn * offsets[None, :, 1]
        y = xy[:, 1, None] + sn * offsets[None, :, 0] + cs * offsets[None, :, 1]
        i = np.floor((x - self.origin[0]) / self.c.resolution).astype(int)
        j = np.floor((y - self.origin[1]) / self.c.resolution).astype(int)
        inside = ((i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)).all(axis=1)
        return inside & self._surface_safe[np.clip(i, 0, self.n-1), np.clip(j, 0, self.n-1)].all(axis=1) & self.hazard_contacts_safe(xy, yaw, offsets)

    def save(self, path):
        np.savez_compressed(path, logodds=self.logodds, seen=self.seen, floor=self.floor,
                            ceiling=self.ceiling, headroom=self.headroom,
                            hazard_votes=self.hazard_votes,
                            origin=self.origin, resolution=self.c.resolution)
