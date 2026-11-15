"""Connected, bounded interpolation of measured ground returns (no scene truth)."""
from __future__ import annotations

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree, Delaunay, QhullError


def ground_surface(xyz, reference_height, resolution, max_gap, max_slope_deg):
    """Return lower-envelope samples connected to the current support height.

    Height alone cannot distinguish a ramp from a wall. Neighbouring returns
    must connect with a bounded gradient and bounded horizontal sampling gap.
    Duplicate XY columns retain the lowest sample; their higher hits are still
    tested separately for obstacles/headroom by GridMap.
    """
    if not len(xyz):
        return np.empty((0, 3))
    cells = np.floor(xyz[:, :2] / resolution).astype(int)
    order = np.lexsort((xyz[:, 2], cells[:, 1], cells[:, 0]))
    _, first, counts = np.unique(cells[order], axis=0, return_index=True, return_counts=True)
    samples = xyz[order[first]].copy()
    # Batch columns with the same lower-envelope population. This is exactly
    # the per-column median, without hundreds of Python/NumPy calls per scan.
    ordered = xyz[order]
    groups = np.repeat(np.arange(len(first)), counts)
    lower = ordered[:, 2] <= np.repeat(ordered[first, 2], counts)+.02
    lower_points = ordered[lower]
    sizes = np.bincount(groups[lower], minlength=len(first))
    starts = np.r_[0, np.cumsum(sizes)[:-1]]
    for count in np.unique(sizes[sizes > 1]):
        indices = np.flatnonzero(sizes == count)
        columns = lower_points[starts[indices, None]+np.arange(count)]
        samples[indices] = np.median(columns, axis=1)
    pairs = cKDTree(samples[:, :2]).query_pairs(max_gap, output_type="ndarray")
    if len(pairs):
        delta = samples[pairs[:, 0]] - samples[pairs[:, 1]]
        limits = np.min(max_slope_deg(samples[pairs, :2]), axis=1) if callable(max_slope_deg) else max_slope_deg
        valid = abs(delta[:, 2]) <= np.tan(np.deg2rad(limits)) * np.linalg.norm(delta[:, :2], axis=1) + .008
        pairs = pairs[valid]
    rows = pairs[:, 0] if len(pairs) else np.empty(0, int)
    cols = pairs[:, 1] if len(pairs) else np.empty(0, int)
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(samples), len(samples)))
    _, components = connected_components(graph, directed=False)
    seeds = abs(samples[:, 2] - reference_height) < .035
    return samples[np.isin(components, np.unique(components[seeds]))]


def interpolate_surface(samples, query_xy, max_gap, max_slope_deg):
    """Interpolate only inside small, measured, gently sloped triangles.

    Never extrapolate beyond returns or bridge a hole wider than max_gap.
    """
    result = np.full(len(query_xy), np.nan)
    if len(samples) < 3 or not len(query_xy):
        return result
    try:
        tri = Delaunay(samples[:, :2])
    except QhullError:
        return result
    vertices = samples[tri.simplices]
    edges = np.stack([vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0]], axis=1)
    cross = np.cross(edges[:, 0], edges[:, 1])
    slope = np.linalg.norm(cross[:, :2], axis=1) / np.maximum(abs(cross[:, 2]), 1e-12)
    longest = np.maximum.reduce([np.linalg.norm(vertices[:, a, :2] - vertices[:, b, :2], axis=1)
                                 for a, b in [(0, 1), (1, 2), (2, 0)]])
    limits = np.min(max_slope_deg(vertices[:, :, :2]), axis=1) if callable(max_slope_deg) else max_slope_deg
    valid_tri = (longest <= max_gap) & (slope <= np.tan(np.deg2rad(limits)) + .02)
    simplex = tri.find_simplex(query_xy)
    indices = np.flatnonzero((simplex >= 0) & valid_tri[np.maximum(simplex, 0)])
    if len(indices):
        ids = simplex[indices]
        delta = query_xy[indices] - vertices[ids, 0, :2]
        result[indices] = vertices[ids, 0, 2] - np.sum(delta * cross[ids, :2], axis=1) / cross[ids, 2]
    return result


class GroundPatch:
    """Cached bounded interpolation for repeated foot queries at control rate."""
    def __init__(self, samples, max_gap, max_slope_deg):
        self.tri = None
        if len(samples) < 3:
            return
        try:
            self.tri = Delaunay(samples[:, :2])
        except QhullError:
            return
        self.vertices = samples[self.tri.simplices]
        v = self.vertices
        self.cross = np.cross(v[:, 1]-v[:, 0], v[:, 2]-v[:, 0])
        slope = np.linalg.norm(self.cross[:, :2], axis=1)/np.maximum(abs(self.cross[:, 2]), 1e-12)
        longest = np.maximum.reduce([np.linalg.norm(v[:, a, :2]-v[:, b, :2], axis=1) for a,b in [(0,1),(1,2),(2,0)]])
        self.valid = (longest <= max_gap)&(slope <= np.tan(np.deg2rad(max_slope_deg))+.02)

    def height(self, xy):
        xy = np.atleast_2d(xy)
        result = np.full(len(xy), np.nan)
        if self.tri is None:
            return result
        ids = self.tri.find_simplex(xy)
        usable = np.flatnonzero((ids >= 0)&self.valid[np.maximum(ids,0)])
        if len(usable):
            k = ids[usable]
            delta = xy[usable]-self.vertices[k,0,:2]
            result[usable] = self.vertices[k,0,2]-np.sum(delta*self.cross[k,:2],axis=1)/self.cross[k,2]
        return result
