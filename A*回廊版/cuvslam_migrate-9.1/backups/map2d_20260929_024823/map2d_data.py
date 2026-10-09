# -*- coding: utf-8 -*-
"""map2d_data.py —— 二维规划地图核心数据与算法（V1）

职责（见《二维规划地图 V1 改动说明》§3-§8）：
  - 调用匹配版本的 map_extractor 从 cuVSLAM 保存地图目录（data.mdb）离线导出
    全局 landmarks / poses / edges（source_map.json）；
  - 三值占用栅格（int8：-1 未知 / 0 自由 / 100 障碍，初始全未知）；
  - 人工标注多边形合成（优先级：人工障碍 > 人工未知 > 候选障碍 > 人工自由 > 默认未知）；
  - 机器人圆形包络膨胀（欧氏距离变换）与软代价；
  - 地图包读写（grid.npz + map.json + annotations.json + preview.png）与校验；
  - 供 A* 使用的查询接口：load_planning_map / world_to_grid / grid_to_world /
    is_traversable / edge_is_free。

本模块不加载 cuVSLAM、不启动相机；栅格计算与测试独立于导出器二进制。
坐标约定：与 cuVSLAM 参考地图一致（X 右、Y 下、Z 前），二维平面为 XZ。
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple, List

import numpy as np
from scipy.ndimage import distance_transform_edt

UNKNOWN = -1
FREE = 0
OBSTACLE = 100

SCHEMA_VERSION = 1


# ---------------------------------------------------------------------------
# 坐标与栅格纯函数
# ---------------------------------------------------------------------------

def world_to_grid(x: float, z: float, origin_xz: Tuple[float, float],
                  resolution_m: float, shape: Tuple[int, int]) -> Optional[Tuple[int, int]]:
    """世界坐标 (x, z) → (row, col)；越界返回 None。

    row = floor((z - z_min) / r)，col = floor((x - x_min) / r)；
    有效边界左闭右开 [x_min, x_min + width*r) × [z_min, z_min + height*r)。
    """
    x_min, z_min = origin_xz
    height, width = shape
    if resolution_m <= 0:
        raise ValueError("resolution_m 必须 > 0")
    col = int(np.floor((x - x_min) / resolution_m))
    row = int(np.floor((z - z_min) / resolution_m))
    if not (0 <= col < width and 0 <= row < height):
        return None
    return (row, col)


def grid_to_world(row: int, col: int, origin_xz: Tuple[float, float],
                  resolution_m: float, shape: Tuple[int, int]) -> Optional[Tuple[float, float]]:
    """(row, col) → 格子中心世界坐标 (x, z)；越界返回 None。"""
    height, width = shape
    if not (0 <= col < width and 0 <= row < height):
        return None
    x_min, z_min = origin_xz
    return (x_min + (col + 0.5) * resolution_m, z_min + (row + 0.5) * resolution_m)


def align_bounds(x_min: float, z_min: float, x_max: float, z_max: float,
                 resolution_m: float) -> Tuple[Tuple[float, float], Tuple[int, int]]:
    """把世界范围向外对齐到栅格边界，返回 (origin_xz, (height, width))。"""
    x0 = np.floor(x_min / resolution_m) * resolution_m
    z0 = np.floor(z_min / resolution_m) * resolution_m
    width = int(np.ceil((x_max - x0) / resolution_m))
    height = int(np.ceil((z_max - z0) / resolution_m))
    return (x0, z0), (height, width)


# ---------------------------------------------------------------------------
# 多边形栅格化（标注用）
# ---------------------------------------------------------------------------

def _point_in_polygon(px: np.ndarray, pz: np.ndarray, x: float, z: float) -> bool:
    """射线法判断 (x, z) 是否在多边形内（含边界）。"""
    inside = False
    n = len(px)
    for i in range(n):
        x1, z1 = px[i], pz[i]
        x2, z2 = px[(i + 1) % n], pz[(i + 1) % n]
        if ((z1 > z) != (z2 > z)) and (x < (x2 - x1) * (z - z1) / (z2 - z1) + x1):
            inside = not inside
    return inside


def validate_polygon(vertices: List[Tuple[float, float]]) -> np.ndarray:
    """校验多边形：至少 3 顶点、无自交、面积非零。返回 (N,2) 数组。"""
    pts = np.asarray(vertices, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < 3 or pts.shape[1] != 2:
        raise ValueError("多边形至少需要 3 个 (x, z) 顶点")
    if not np.all(np.isfinite(pts)):
        raise ValueError("多边形顶点含 NaN/Inf")
    # 面积（鞋带公式）
    xs, zs = pts[:, 0], pts[:, 1]
    area2 = np.sum(xs * np.roll(zs, -1) - np.roll(xs, -1) * zs)
    if abs(area2) < 1e-12:
        raise ValueError("多边形面积为零")
    # 自交检测：任意两条非相邻边相交即非法
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or j == (i + 1) % n or (i == 0 and (j + 1) % n == 0):
                continue
            c, d = pts[j], pts[(j + 1) % n]
            if _segments_intersect(a, b, c, d):
                raise ValueError("多边形自交")
    return pts


def _segments_intersect(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> bool:
    def ccw(p, q, r):
        return np.cross(q - p, r - p)

    d1, d2, d3, d4 = ccw(a, b, c), ccw(a, b, d), ccw(c, d, a), ccw(c, d, b)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    # 共线重叠（边界共享的相邻边不算自交，此处保守处理）
    return False


def rasterize_polygon(pts: np.ndarray, origin_xz: Tuple[float, float],
                      resolution_m: float, shape: Tuple[int, int],
                      mode: str) -> List[Tuple[int, int]]:
    """把多边形栅格化。mode 语义：
      - "conservative"（障碍/未知）：覆盖到的格子都算（中心或四角任一在内）；
      - "full_cover"（自由）：仅完全覆盖的格子（中心 + 四角都在内）。
    """
    x_min, z_min = origin_xz
    height, width = shape
    px, pz = pts[:, 0], pts[:, 1]
    x0 = min(px.min(), x_min + width * resolution_m)
    x1 = max(px.max(), x_min)
    z0 = min(pz.min(), z_min + height * resolution_m)
    z1 = max(pz.max(), z_min)
    col_lo = max(0, int(np.floor((x0 - x_min) / resolution_m)))
    col_hi = min(width - 1, int(np.floor((x1 - x_min) / resolution_m)))
    row_lo = max(0, int(np.floor((z0 - z_min) / resolution_m)))
    row_hi = min(height - 1, int(np.floor((z1 - z_min) / resolution_m)))
    cells = []
    for row in range(row_lo, row_hi + 1):
        for col in range(col_lo, col_hi + 1):
            cx, cz = grid_to_world(row, col, origin_xz, resolution_m, shape)
            if mode == "full_cover":
                corners = [(cx - resolution_m / 2, cz - resolution_m / 2),
                           (cx + resolution_m / 2, cz - resolution_m / 2),
                           (cx + resolution_m / 2, cz + resolution_m / 2),
                           (cx - resolution_m / 2, cz + resolution_m / 2),
                           (cx, cz)]
                if all(_point_in_polygon(px, pz, x, z) for x, z in corners):
                    cells.append((row, col))
            else:  # conservative
                corners = [(cx - resolution_m / 2, cz - resolution_m / 2),
                           (cx + resolution_m / 2, cz - resolution_m / 2),
                           (cx + resolution_m / 2, cz + resolution_m / 2),
                           (cx - resolution_m / 2, cz + resolution_m / 2),
                           (cx, cz)]
                if any(_point_in_polygon(px, pz, x, z) for x, z in corners):
                    cells.append((row, col))
    return cells


# ---------------------------------------------------------------------------
# 栅格合成与膨胀
# ---------------------------------------------------------------------------

def build_occupancy(origin_xz: Tuple[float, float], resolution_m: float,
                    shape: Tuple[int, int],
                    candidate_cells: List[Tuple[int, int]],
                    annotations: Optional[dict] = None) -> np.ndarray:
    """三值栅格合成。初始化全未知；按固定优先级应用：
    候选障碍 → 人工自由(仅完全覆盖) → 人工未知 → 人工障碍。
    annotations: {"free": [...], "obstacle": [...], "unknown": [...],
                  "candidate_remove": [...]}，多边形为 (x,z) 顶点列表。
    """
    height, width = shape
    grid = np.full((height, width), UNKNOWN, dtype=np.int8)
    for (row, col) in candidate_cells:
        if 0 <= row < height and 0 <= col < width:
            grid[row, col] = OBSTACLE
    annotations = annotations or {}
    for name, mode in (("free", "full_cover"), ("unknown", "conservative"),
                       ("obstacle", "conservative")):
        for verts in annotations.get(name, []):
            pts = validate_polygon(verts)
            for (row, col) in rasterize_polygon(pts, origin_xz, resolution_m, shape, mode):
                if name == "free":
                    grid[row, col] = FREE
                elif name == "unknown":
                    grid[row, col] = UNKNOWN
                else:
                    grid[row, col] = OBSTACLE
    # 墙线段（WallSeg 标注）：两两端点，保守覆盖所有相交格，按障碍优先级。
    for seg in annotations.get("obstacle_segments", []):
        seg = np.asarray(seg, dtype=np.float64)
        if seg.shape != (2, 2) or not np.all(np.isfinite(seg)):
            raise ValueError(f"墙线段必须为 2 个 (x,z) 端点: {seg}")
        c0 = world_to_grid(seg[0, 0], seg[0, 1], origin_xz, resolution_m, shape)
        c1 = world_to_grid(seg[1, 0], seg[1, 1], origin_xz, resolution_m, shape)
        if c0 is None or c1 is None:
            raise ValueError(f"墙线段端点越界: {seg}")
        for (row, col) in _supercover_line(c0[0], c0[1], c1[0], c1[1]):
            grid[row, col] = OBSTACLE
    return grid


def compute_inflation(occupancy: np.ndarray, resolution_m: float,
                      robot_radius_m: float, safety_margin_m: float,
                      soft_band_m: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """由原始占用栅格计算 clearance / traversable / cost（不改动 occupancy）。

    保守圆形包络：R_required = robot_radius_m + safety_margin_m。
    只对自由掩码补零外圈后做欧氏距离变换（sampling=r），
    clearance = max(0, 中心距 - sqrt(2)*r)（保守下界，语义写入元数据）。
    traversable = 自由 且 clearance > R_required + eps；其余（障碍/未知/地图外）不可通行。
    cost = clip(1 - (clearance - R_required)/soft_band, 0, 1)（仅可通行格），否则 +inf。
    """
    if robot_radius_m <= 0 or not np.isfinite(robot_radius_m):
        raise ValueError("robot_radius_m 必须为有限正数")
    if safety_margin_m < 0 or not np.isfinite(safety_margin_m):
        raise ValueError("safety_margin_m 必须为有限非负数")
    if soft_band_m <= 0 or not np.isfinite(soft_band_m):
        raise ValueError("soft_band_m 必须为有限正数")
    free_mask = occupancy == FREE
    padded = np.pad(free_mask, 1, mode="constant", constant_values=False)
    edt = distance_transform_edt(padded, sampling=resolution_m)
    center_dist = edt[1:-1, 1:-1]
    clearance = np.maximum(0.0, center_dist - np.sqrt(2.0) * resolution_m)
    r_required = robot_radius_m + safety_margin_m
    eps = 1e-9
    traversable = free_mask & (clearance > r_required + eps)
    band = np.clip(1.0 - (clearance - r_required) / soft_band_m, 0.0, 1.0)
    cost = np.full_like(clearance, np.inf, dtype=np.float64)
    cost[traversable] = band[traversable]
    return clearance, traversable, cost


# ---------------------------------------------------------------------------
# 查询接口（A* 用）
# ---------------------------------------------------------------------------

def edge_is_free(occ: np.ndarray, traversable: np.ndarray,
                 cell_a: Tuple[int, int], cell_b: Tuple[int, int]) -> bool:
    """两格是否可直接通行：
      - 8 邻域；斜向必须同时检查两个相邻正交格（禁止穿角）；
      - 线段查询保守覆盖所有相交格子（不只检查端点）。
    """
    (ra, ca), (rb, cb) = cell_a, cell_b
    h, w = occ.shape
    if not (0 <= ra < h and 0 <= ca < w and 0 <= rb < h and 0 <= cb < w):
        return False
    if not (traversable[ra, ca] and traversable[rb, cb]):
        return False
    dr, dc = rb - ra, cb - ca
    if max(abs(dr), abs(dc)) > 1:
        # 长线段：超覆盖光栅化，所有相交格子都必须可通行
        for (r, c) in _supercover_line(ra, ca, rb, cb):
            if not traversable[r, c]:
                return False
        return True
    if abs(dr) == 1 and abs(dc) == 1:
        # 斜向：穿角检查
        if not traversable[ra, cb] or not traversable[rb, ca]:
            return False
    return True


def _supercover_line(r0: int, c0: int, r1: int, c1: int) -> List[Tuple[int, int]]:
    """Amanatides & Woo 超覆盖光栅化：返回线段穿过的所有格子。"""
    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


# ---------------------------------------------------------------------------
# 导出器调用与 source_map.json
# ---------------------------------------------------------------------------

def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run_extractor(extractor: Path, map_dir: Path, timeout_s: float = 600.0) -> dict:
    """把静态源地图复制到临时目录，用只读方式调用 map_extractor 导出 JSON。

    返回解析后的 dict；源目录不创建锁文件、不写回。
    """
    if not extractor.is_file() or not os.access(extractor, os.X_OK):
        raise RuntimeError(f"导出器不可执行: {extractor}")
    db = map_dir / "data.mdb"
    if not db.is_file() or db.stat().st_size == 0:
        raise RuntimeError(f"地图目录缺少有效 data.mdb: {map_dir}")
    with tempfile.TemporaryDirectory(prefix="map2d_src_") as src_tmp, \
            tempfile.TemporaryDirectory(prefix="map2d_out_") as out_tmp:
        shutil.copy2(db, Path(src_tmp) / "data.mdb")
        t0 = time.monotonic()
        proc = subprocess.run(
            [str(extractor), f"--map_path={src_tmp}"],
            cwd=out_tmp,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=timeout_s,
        )
        if proc.returncode != 0:
            raise RuntimeError(
                f"map_extractor 退出码 {proc.returncode}:\n"
                f"{proc.stdout.decode(errors='replace')[-2000:]}\n"
                f"{proc.stderr.decode(errors='replace')[-2000:]}")
        out_json = Path(out_tmp) / "map.json"
        if not out_json.is_file():
            raise RuntimeError("map_extractor 未生成 map.json")
        data = json.loads(out_json.read_text(encoding="utf-8"))
        _validate_source_json(data)
        return data


def _validate_source_json(data: dict) -> None:
    if "landmarks" not in data or not isinstance(data["landmarks"], list):
        raise RuntimeError("导出 JSON 缺少 landmarks 列表")
    if not data["landmarks"]:
        raise RuntimeError("导出 JSON 无任何地图点")
    ids = set()
    for lm in data["landmarks"]:
        if not isinstance(lm.get("id"), int) or "pose" not in lm:
            raise RuntimeError("地图点结构非法: 需要 int id 与 pose")
        p = lm["pose"]
        for k in ("x", "y", "z"):
            v = p.get(k)
            if not isinstance(v, (int, float)) or not np.isfinite(float(v)):
                raise RuntimeError(f"地图点 {lm.get('id')} 的 {k} 坐标非法: {v}")
        if lm["id"] in ids:
            raise RuntimeError(f"地图点 id 重复: {lm['id']}")
        ids.add(lm["id"])
    if "poses" not in data or not isinstance(data["poses"], list):
        raise RuntimeError("导出 JSON 缺少 poses 列表")
    if "edges" not in data or not isinstance(data["edges"], list):
        raise RuntimeError("导出 JSON 缺少 edges 列表")


def landmark_xyz(data: dict) -> np.ndarray:
    """landmarks → (N,3) 全局 XYZ 数组（cuVSLAM 系：X 右、Y 下、Z 前）。"""
    pts = np.array([[lm["pose"]["x"], lm["pose"]["y"], lm["pose"]["z"]]
                    for lm in data["landmarks"]], dtype=np.float64)
    return pts


def exclude_landmarks(pts: np.ndarray, ids: np.ndarray, erased_ids) -> tuple:
    """按擦除点 id 剔除地图点（标注橡皮擦语义：被擦除的点不再参与障碍判定）。

    返回 (过滤后 pts, 过滤后 ids)。erased_ids 为可迭代的 int id 集合。
    """
    pts = np.asarray(pts, dtype=np.float64)
    ids = np.asarray(ids, dtype=np.int64)
    erased = set(int(i) for i in (erased_ids or []))
    if not erased:
        return pts, ids
    keep = ~np.isin(ids, np.array(sorted(erased), dtype=np.int64))
    return pts[keep], ids[keep]


def map_stats(data: dict) -> dict:
    pts = landmark_xyz(data)
    return {
        "num_landmarks": len(data["landmarks"]),
        "num_keyframes": len(data["poses"]),
        "num_edges": len(data["edges"]),
        "xyz_min": [float(v) for v in pts.min(axis=0)],
        "xyz_max": [float(v) for v in pts.max(axis=0)],
    }


# ---------------------------------------------------------------------------
# 地图包读写
# ---------------------------------------------------------------------------

@dataclass
class PlanningMap:
    """可重复加载的地图包对象。"""
    directory: Path
    meta: dict
    occupancy: np.ndarray
    traversable: np.ndarray
    clearance_m: np.ndarray
    cost: np.ndarray
    reviewed: bool

    def world_to_grid(self, x: float, z: float) -> Optional[Tuple[int, int]]:
        return world_to_grid(x, z, self.meta["origin_xz"],
                             self.meta["resolution_m"], self.occupancy.shape)

    def grid_to_world(self, row: int, col: int) -> Optional[Tuple[float, float]]:
        return grid_to_world(row, col, self.meta["origin_xz"],
                             self.meta["resolution_m"], self.occupancy.shape)

    def is_traversable(self, row: int, col: int) -> bool:
        h, w = self.occupancy.shape
        if not (0 <= row < h and 0 <= col < w):
            return False
        return bool(self.traversable[row, col])

    def edge_is_free(self, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
        return edge_is_free(self.occupancy, self.traversable, a, b)


def save_map_package(out_dir: Path, meta: dict, occupancy: np.ndarray,
                     traversable: np.ndarray, clearance_m: np.ndarray,
                     cost: np.ndarray, annotations: dict,
                     source_map: Optional[dict] = None) -> Path:
    """写地图包到 out_dir（版本化目录）。返回发布目录。

    先写临时目录，校验通过后再发布；默认不覆盖已存在版本。
    """
    out_dir = Path(out_dir)
    if out_dir.exists() and any(out_dir.iterdir()):
        raise FileExistsError(f"输出目录已存在且非空: {out_dir}")
    tmp = out_dir.with_name(out_dir.name + ".tmp")
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir(parents=True)
    try:
        shapes = [occupancy.shape, traversable.shape, clearance_m.shape, cost.shape]
        if any(s != occupancy.shape for s in shapes):
            raise ValueError("四个栅格数组形状不一致")
        if occupancy.dtype != np.int8:
            occupancy = occupancy.astype(np.int8)
        np.savez_compressed(tmp / "grid.npz", occupancy=occupancy,
                            traversable=traversable, clearance_m=clearance_m, cost=cost)
        (tmp / "annotations.json").write_text(
            json.dumps(annotations, ensure_ascii=False, indent=2), encoding="utf-8")
        if source_map is not None:
            (tmp / "source_map.json").write_text(
                json.dumps(source_map, ensure_ascii=False), encoding="utf-8")
        meta = dict(meta)
        meta.setdefault("schema_version", SCHEMA_VERSION)
        meta.setdefault("reviewed", False)
        meta["grid_sha256"] = sha256_file(tmp / "grid.npz")
        meta["annotations_sha256"] = sha256_file(tmp / "annotations.json")
        (tmp / "map.json").write_text(
            json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        # 全部写入后再发布
        tmp.rename(out_dir)
        return out_dir
    finally:
        if tmp.exists():
            shutil.rmtree(tmp, ignore_errors=True)


def load_map_package(directory: Path, expected_map_id: Optional[str] = None,
                     require_reviewed: bool = True) -> PlanningMap:
    """加载并校验地图包。规划用途必须 require_reviewed=True（拒绝未审核地图）。"""
    directory = Path(directory)
    meta_path = directory / "map.json"
    if not meta_path.is_file():
        raise FileNotFoundError(f"缺少 map.json: {directory}")
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    if meta.get("schema_version") != SCHEMA_VERSION:
        raise RuntimeError(f"schema_version 不匹配: {meta.get('schema_version')}")
    if expected_map_id is not None and meta.get("map_id") != expected_map_id:
        raise RuntimeError(f"map_id 不匹配: 期望 {expected_map_id}, 实际 {meta.get('map_id')}")
    if require_reviewed and not meta.get("reviewed", False):
        raise RuntimeError(f"地图未通过审核（reviewed=false），不能用于规划: {directory}")
    grid_path = directory / "grid.npz"
    if not grid_path.is_file():
        raise FileNotFoundError(f"缺少 grid.npz: {directory}")
    if sha256_file(grid_path) != meta.get("grid_sha256"):
        raise RuntimeError("grid.npz 校验和不匹配（文件损坏或被篡改）")
    with np.load(grid_path, allow_pickle=False) as npz:
        occupancy = npz["occupancy"]
        traversable = npz["traversable"]
        clearance_m = npz["clearance_m"]
        cost = npz["cost"]
    if occupancy.shape != traversable.shape or occupancy.shape != clearance_m.shape \
            or occupancy.shape != cost.shape:
        raise RuntimeError("栅格数组形状不一致")
    if not set(np.unique(occupancy)).issubset({UNKNOWN, FREE, OBSTACLE}):
        raise RuntimeError("occupancy 含非法数值")
    return PlanningMap(directory=directory, meta=meta, occupancy=occupancy,
                       traversable=traversable, clearance_m=clearance_m, cost=cost,
                       reviewed=bool(meta.get("reviewed", False)))


def load_planning_map(directory: Path, expected_map_id: Optional[str] = None) -> PlanningMap:
    """A* 规划入口：拒绝未审核地图（编辑器草稿请用 load_map_package(..., require_reviewed=False)）。"""
    return load_map_package(directory, expected_map_id=expected_map_id, require_reviewed=True)
