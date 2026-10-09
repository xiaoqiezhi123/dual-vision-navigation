# Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA software released under the NVIDIA Community License is intended to be used to enable
# the further development of AI and robotics technologies. Such software has been designed, tested,
# and optimized for use with NVIDIA hardware, and this License grants permission to use the software
# solely with such hardware.
# Subject to the terms of this License, NVIDIA confirms that you are free to commercially use,
# modify, and distribute the software with NVIDIA hardware. NVIDIA does not claim ownership of any
# outputs generated using the software or derivative works thereof. Any code contributions that you
# share with NVIDIA are licensed to NVIDIA as feedback under this License and may be incorporated
# in future releases without notice or attribution.
# By using, reproducing, modifying, distributing, performing, or displaying any portion or element
# of the software or derivative works thereof, you agree to be bound by this License.

"""Minimal descriptor database + world-coordinate lookup on top of cuVSLAM.

cuVSLAM's public API does not expose feature descriptors, so "global feature
matching" (query an image -> find the matching map point -> read its world
coordinate) needs an extra descriptor layer. This module fills that gap with
the smallest possible implementation:

  * cuVSLAM provides a stable ``track_id -> world_xyz`` map via
    ``Tracker.get_final_landmarks()`` (world = odometry start frame).
  * cuVSLAM provides per-frame ``(u, v) <-> track_id`` via
    ``Tracker.get_last_observations(camera_index)``. The ``id`` fields of
    ``Observation`` and ``Landmark`` share the same track-id namespace.
  * We compute our own descriptors (ORB) at those ``(u, v)`` locations and
    store them keyed by ``track_id``. Querying matches descriptors back to
    ``track_id``, and world coordinates come from the id map.

Wiring into ``run_vio.py`` (inside the camera thread, after ``track``):

    from feature_map import FeatureMapDB

    db = FeatureMapDB()
    ...
    observations = tracker.get_last_observations(0)
    final = tracker.get_final_landmarks()          # dict[track_id, (x, y, z)]
    db.add_frame(images[0], observations, final)   # accumulate descriptors

    # later, to localize a point observed in a new image:
    matches = db.query(query_image)                # -> [{track_id, world_xyz, ...}]

Swap ORB for a learned local feature (SuperPoint / DISK) by replacing
``_extract`` / ``_match``; the id/coordinate plumbing stays the same.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# ORB patch size (px) used when computing descriptors at cuVSLAM keypoints.
ORB_PATCH_SIZE = 31
# Keep a border margin so descriptor patches stay inside the image.
BORDER_MARGIN = ORB_PATCH_SIZE // 2 + 1

WorldXYZ = Tuple[float, float, float]


class FeatureMapDB:
    """Descriptor database keyed by cuVSLAM track id, with world coordinates."""

    def __init__(self, nfeatures: int = 500) -> None:
        self._orb = cv2.ORB_create(nfeatures=nfeatures)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        # Parallel arrays, aligned by index.
        self._descs: np.ndarray = np.empty((0, 32), dtype=np.uint8)  # N x 32 bytes
        self._ids: List[int] = []
        self._id_to_index: Dict[int, int] = {}
        self._world: List[WorldXYZ] = []

    def __len__(self) -> int:
        return len(self._ids)

    @staticmethod
    def _fixed_keypoints(pts: np.ndarray) -> List[cv2.KeyPoint]:
        """Build keypoints with a fixed patch size and orientation.

        Using ``angle=0`` on both the database side and the query side keeps
        descriptors comparable (translation-invariant, but not rotation-
        invariant). See the module docstring for the trade-off.
        """
        return [
            cv2.KeyPoint(x=float(x), y=float(y), size=ORB_PATCH_SIZE, angle=0.0)
            for x, y in pts
        ]

    # --- building ---------------------------------------------------------

    def add_frame(
        self,
        image: np.ndarray,
        observations: Sequence,
        world_coords: Dict[int, WorldXYZ],
    ) -> int:
        """Add one frame's observations to the database.

        Args:
            image: Grayscale ``(H, W)`` uint8 image (e.g. cuVSLAM left IR frame).
            observations: ``Tracker.get_last_observations(0)`` result; each item
                needs ``.u``, ``.v`` and ``.id``.
            world_coords: ``Tracker.get_final_landmarks()`` result, mapping
                ``track_id -> (x, y, z)`` in the odometry start frame.

        Returns:
            Number of newly-added (previously unseen) track ids.
        """
        if image is None or image.size == 0 or not observations:
            return 0

        uvs = np.asarray([[o.u, o.v] for o in observations], dtype=np.float32)
        ids = [int(o.id) for o in observations]
        if uvs.shape[0] == 0:
            return 0

        h, w = image.shape[:2]
        # Drop points whose descriptor patch would fall outside the image.
        keep = (
            (uvs[:, 0] >= BORDER_MARGIN)
            & (uvs[:, 0] < w - BORDER_MARGIN)
            & (uvs[:, 1] >= BORDER_MARGIN)
            & (uvs[:, 1] < h - BORDER_MARGIN)
        )
        # Only keep ids that actually have a world coordinate to look up.
        keep &= np.array([tid in world_coords for tid in ids])

        uvs = uvs[keep]
        ids = np.array(ids)[keep]
        if uvs.shape[0] == 0:
            return 0

        keypoints = self._fixed_keypoints(uvs)
        _, descs = self._orb.compute(image, keypoints)

        added = 0
        for tid, desc, (x, y) in zip(ids, descs, uvs):
            tid = int(tid)
            if tid not in self._id_to_index:
                self._append(tid, desc, tuple(world_coords[tid]))
                added += 1
            else:
                # Refresh the world coordinate (e.g. after loop-closure
                # refinement); keep the first-seen descriptor.
                idx = self._id_to_index[tid]
                self._world[idx] = tuple(world_coords[tid])
        return added

    def _append(self, tid: int, desc: np.ndarray, xyz: WorldXYZ) -> None:
        self._descs = np.vstack([self._descs, desc.reshape(1, -1).astype(np.uint8)])
        self._id_to_index[tid] = len(self._ids)
        self._ids.append(tid)
        self._world.append(xyz)

    # --- query ------------------------------------------------------------

    def query(
        self,
        image: np.ndarray,
        ratio: float = 0.75,
        max_results: Optional[int] = None,
    ) -> List[Dict]:
        """Match a query image against the database and return world coordinates.

        Args:
            image: Grayscale ``(H, W)`` uint8 query image.
            ratio: Lowe ratio-test threshold (lower = stricter).
            max_results: Optional cap on the number of returned matches.

        Returns:
            List of dicts ``{track_id, world_xyz, query_uv, distance}``, sorted
            by distance (best first). Empty if the DB is empty or nothing matched.
        """
        if len(self._ids) == 0:
            return []
        kps = self._orb.detect(image, None)
        if not kps:
            return []
        # Recompute descriptors with the same fixed size/orientation recipe as
        # the database side, so descriptors are directly comparable.
        fixed_kps = self._fixed_keypoints(
            np.array([[kp.pt[0], kp.pt[1]] for kp in kps], dtype=np.float32)
        )
        _, qdescs = self._orb.compute(image, fixed_kps)
        if qdescs is None:
            return []

        raw = self._matcher.knnMatch(qdescs, self._descs, k=2)
        results: List[Dict] = []
        for m, n in raw:
            if m.distance < ratio * n.distance:
                idx = m.trainIdx
                kp = kps[m.queryIdx]
                results.append(
                    {
                        "track_id": self._ids[idx],
                        "world_xyz": self._world[idx],
                        "query_uv": (float(kp.pt[0]), float(kp.pt[1])),
                        "distance": float(m.distance),
                    }
                )
        results.sort(key=lambda r: r["distance"])
        return results[:max_results] if max_results is not None else results

    def world_xyz_of(self, track_id: int) -> Optional[WorldXYZ]:
        """Look up a world coordinate directly by track id (no matching)."""
        idx = self._id_to_index.get(track_id)
        return self._world[idx] if idx is not None else None


def _demo() -> None:
    """Self-contained smoke test (no camera needed): proves the plumbing works."""
    # A clean high-contrast scene whose corners ORB reliably detects.
    base = np.full((240, 320), 128, dtype=np.uint8)
    cv2.rectangle(base, (60, 60), (90, 90), 255, -1)
    cv2.rectangle(base, (180, 70), (210, 100), 0, -1)
    cv2.rectangle(base, (100, 160), (130, 190), 255, -1)
    cv2.circle(base, (240, 200), 16, 0, -1)
    cv2.circle(base, (50, 210), 16, 255, -1)

    # Play the role of cuVSLAM: detect a few strong corners and assign them
    # track ids + (fake) world coordinates.
    detector = cv2.ORB_create(nfeatures=6)
    kps = detector.detect(base, None)

    class Obs:
        def __init__(self, tid, u, v):
            self.id, self.u, self.v = tid, u, v

    observations = [Obs(i, kp.pt[0], kp.pt[1]) for i, kp in enumerate(kps, start=1)]
    world = {
        i: (0.1 * i, -0.2 * i, 2.0 + 0.1 * i)
        for i, kp in enumerate(kps, start=1)
    }

    db = FeatureMapDB()
    db.add_frame(base, observations, world)

    # Query with a copy of the scene shifted by an integer offset (so the
    # descriptor patches are pixel-identical, not re-interpolated).
    shift = cv2.warpAffine(
        base, np.float32([[1, 0, 5], [0, 1, -3]]), (320, 240)
    )
    matches = db.query(shift)
    print(f"DB size: {len(db)}; matches: {len(matches)}")
    recovered = {m["track_id"] for m in matches}
    for m in matches[:6]:
        print(
            f"  track_id={m['track_id']} -> world_xyz={m['world_xyz']} "
            f"(dist={m['distance']:.1f}, uv={m['query_uv']})"
        )
    assert recovered, "expected at least one track id to be recovered"
    assert any(m["world_xyz"] == world[m["track_id"]] for m in matches), (
        "recovered world coordinate must match the stored one"
    )
    print("OK: descriptor DB -> world coordinate round-trip works.")


if __name__ == "__main__":
    _demo()
