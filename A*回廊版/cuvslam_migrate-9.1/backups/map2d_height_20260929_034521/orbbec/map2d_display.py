"""Display/selection helpers. Display pixels never replace planning geometry."""
import numpy as np
from scipy.spatial import cKDTree


class PointCloudIndex:
    def __init__(self, points, ids, candidate_ids=()):
        self.xz = np.asarray(points, dtype=float)[:, [0, 2]]
        self.ids = np.asarray(ids, dtype=np.int64)
        self.tree = cKDTree(self.xz)
        self.candidate = np.isin(self.ids, np.asarray(candidate_ids, dtype=np.int64))
        self.alive = np.ones(len(self.ids), dtype=bool)

    def set_erased(self, erased):
        self.alive = ~np.isin(self.ids, np.fromiter(erased, dtype=np.int64))

    def visible(self, xlim, zlim):
        x, z = self.xz.T
        return np.flatnonzero(self.alive & (x >= xlim[0]) & (x <= xlim[1]) &
                             (z >= zlim[0]) & (z <= zlim[1]))

    def erase_stroke(self, start, end, radius):
        """Exact capsule hit test, including points between mouse events."""
        start, end = np.asarray(start), np.asarray(end)
        delta = end - start
        length = np.linalg.norm(delta)
        idx = np.asarray(self.tree.query_ball_point((start + end) / 2,
                                                   length / 2 + radius), dtype=int)
        if not len(idx):
            return self.ids[idx]
        pts = self.xz[idx]
        t = np.clip((pts-start) @ delta / length**2, 0, 1) if length else np.zeros(len(idx))
        distance2 = np.sum((pts - (start + t[:, None] * delta))**2, axis=1)
        return self.ids[idx[distance2 <= radius**2 + 1e-12]]

    def raster(self, indices, xlim, zlim, width, height):
        """One screen pixel per occupied bin; keep rare points, no random sampling."""
        rgb = np.full((height, width, 3), 255, dtype=np.uint8)
        pts = self.xz[indices]
        cc = np.clip(((pts[:, 0]-xlim[0]) / (xlim[1]-xlim[0]) * width).astype(int), 0, width-1)
        rr = np.clip(((pts[:, 1]-zlim[0]) / (zlim[1]-zlim[0]) * height).astype(int), 0, height-1)
        rgb[rr, cc] = (90, 90, 90)
        cand = self.candidate[indices]
        rgb[rr[cand], cc[cand]] = (215, 45, 35)
        return rgb

    def screen_indices(self, indices, xlim, zlim, width, height):
        """Keep an original representative of each occupied screen pixel/layer."""
        pts = self.xz[indices]
        cc = np.clip(((pts[:,0]-xlim[0])/(xlim[1]-xlim[0])*width).astype(int),0,width-1)
        rr = np.clip(((pts[:,1]-zlim[0])/(zlim[1]-zlim[0])*height).astype(int),0,height-1)
        key = rr * width + cc
        _, first = np.unique(key, return_index=True)
        candidates = np.flatnonzero(self.candidate[indices])
        _, first_red = np.unique(key[candidates], return_index=True)
        return indices[first], indices[candidates[first_red]]
