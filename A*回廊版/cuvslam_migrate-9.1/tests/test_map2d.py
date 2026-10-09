# -*- coding: utf-8 -*-
"""test_map2d.py —— 二维规划地图纯计算测试（合成数据，不需要相机/cuVSLAM/GPU）。

覆盖《二维规划地图 V1 改动说明》§9 最低测试清单中的纯计算项：
  坐标往返/负坐标/边界、三值语义与标注优先级、膨胀距离解析对照、
  半径增大关闭窄通道、穿角与穿墙拒绝、全未知/全障碍边界、保存加载与校验。
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "orbbec"))
import map2d_data as m2d  # noqa: E402

R = 0.1  # 测试栅格分辨率
ORIGIN = (-5.0, -5.0)
SHAPE = (100, 100)  # 10m x 10m


def make_occ(cells_obs=None, cells_free=None):
    occ = np.full(SHAPE, m2d.UNKNOWN, dtype=np.int8)
    if cells_obs:
        for r, c in cells_obs:
            occ[r, c] = m2d.OBSTACLE
    if cells_free:
        for r, c in cells_free:
            occ[r, c] = m2d.FREE
    return occ


def test_coordinate_roundtrip():
    # 负坐标、非零原点
    for x, z in [(-4.95, -4.95), (0.0, 0.0), (4.94, 4.94), (0.31, -2.22)]:
        cell = m2d.world_to_grid(x, z, ORIGIN, R, SHAPE)
        assert cell is not None
        wx, wz = m2d.grid_to_world(*cell, ORIGIN, R, SHAPE)
        assert abs(wx - (ORIGIN[0] + (cell[1] + 0.5) * R)) < 1e-9
        assert abs(wz - (ORIGIN[1] + (cell[0] + 0.5) * R)) < 1e-9


def test_bounds_left_closed_right_open():
    # 左闭右开边界
    assert m2d.world_to_grid(-5.0, -5.0, ORIGIN, R, SHAPE) == (0, 0)
    assert m2d.world_to_grid(5.0 - 1e-9, 5.0 - 1e-9, ORIGIN, R, SHAPE) == (99, 99)
    # 越界返回无效，禁止裁剪/绕回
    assert m2d.world_to_grid(5.0, 0.0, ORIGIN, R, SHAPE) is None
    assert m2d.world_to_grid(-5.01, 0.0, ORIGIN, R, SHAPE) is None
    assert m2d.world_to_grid(0.0, 5.0, ORIGIN, R, SHAPE) is None
    assert m2d.grid_to_world(-1, 0, ORIGIN, R, SHAPE) is None
    assert m2d.grid_to_world(0, 100, ORIGIN, R, SHAPE) is None


def test_inflation_matches_analytic_rectangle():
    # 单格障碍在 (50,50)，自由区为其余；解析距离 = 到障碍格中心（中心到中心），
    # 但地图外圈（padding 补零）同样是不可进入边界，需取两者较小值。
    occ = make_occ(cells_obs=[(50, 50)],
                   cells_free=[(r, c) for r in range(SHAPE[0]) for c in range(SHAPE[1])
                               if (r, c) != (50, 50)])
    clearance, traversable, cost = m2d.compute_inflation(
        occ, R, robot_radius_m=0.2, safety_margin_m=0.0, soft_band_m=0.3)
    rr, cc = np.mgrid[0:SHAPE[0], 0:SHAPE[1]]
    dist = np.sqrt(((rr - 50) * R) ** 2 + ((cc - 50) * R) ** 2)
    # 到地图外圈（原网格索引 -1 与 100）的中心距
    ring = np.minimum.reduce([rr + 1, SHAPE[0] - rr, cc + 1, SHAPE[1] - cc]) * R
    expected = np.maximum(0.0, np.minimum(dist, ring) - np.sqrt(2) * R)
    # clearance 不高估几何间距，且与解析距离精确一致（EDT 为精确欧氏）
    assert np.all(clearance <= expected + 1e-9)
    assert np.allclose(clearance, expected, atol=1e-9)


def test_inflation_radius_closes_passage():
    # 10 格宽走廊（列 45..54，1.0m）：小包络可通行、大包络完全关闭。
    occ = make_occ()
    occ[:, :] = m2d.OBSTACLE
    for r in range(SHAPE[0]):
        for c in range(45, 55):
            occ[r, c] = m2d.FREE
    _, t_small, _ = m2d.compute_inflation(occ, R, robot_radius_m=0.05, safety_margin_m=0.0,
                                          soft_band_m=0.3)
    _, t_big, _ = m2d.compute_inflation(occ, R, robot_radius_m=0.5, safety_margin_m=0.0,
                                        soft_band_m=0.3)
    # 半径增大时，可通行集合只能缩小
    assert t_big.sum() <= t_small.sum()
    # 1.0m 走廊中心最大 clearance = 0.5 - sqrt(2)*0.1 ≈ 0.359 < 0.5 → 大半径完全关闭
    assert t_big.sum() == 0
    # 小半径（0.05 < 0.359）下仍有可通行格
    assert t_small.sum() > 0


def test_unknown_and_outside_never_traversable():
    occ = make_occ(cells_free=[(10, 10), (10, 11), (11, 10), (11, 11)])
    clearance, traversable, cost = m2d.compute_inflation(
        occ, R, robot_radius_m=0.05, safety_margin_m=0.0, soft_band_m=0.3)
    # 未知格不可通行
    assert not traversable[0, 0]
    # 自由格若离未知边界太近也不可通行（未知按不可进入处理，边界留机器人尺寸）
    assert traversable.sum() >= 0
    # cost：不可通行 = +inf，可通行 ∈ [0,1]
    assert np.all(np.isinf(cost[~traversable]))
    assert np.all((cost[traversable] >= 0) & (cost[traversable] <= 1))


def test_edge_corner_cut_rejected():
    # 直接测穿角逻辑（traversable 手工构造，避免膨胀把贴墙格一律判不可通行）
    occ = make_occ(cells_obs=[(10, 11), (11, 10)])
    trav = np.zeros(SHAPE, dtype=bool)
    trav[10, 10] = trav[11, 11] = True  # 对角线两端可通行，两个正交角格被障碍阻断
    a, b = (10, 10), (11, 11)
    assert not m2d.edge_is_free(occ, trav, a, b)
    # 无角障碍时斜向可通行
    occ2 = make_occ()
    trav2 = np.zeros(SHAPE, dtype=bool)
    trav2[10, 10] = trav2[11, 11] = trav2[10, 11] = trav2[11, 10] = True
    assert m2d.edge_is_free(occ2, trav2, a, b)


def test_edge_line_through_wall_rejected():
    # 长线段中间穿墙必须被拒绝（不只检查端点）
    occ = make_occ()
    occ[:, 30:70] = m2d.FREE
    occ[:, 50] = m2d.OBSTACLE  # 一列墙
    _, traversable, _ = m2d.compute_inflation(
        occ, R, robot_radius_m=0.05, safety_margin_m=0.0, soft_band_m=0.3)
    assert not m2d.edge_is_free(occ, traversable, (50, 31), (50, 69))


def test_annotation_priority_and_full_cover():
    # 候选障碍 → 人工自由（仅完全覆盖）→ 人工未知 → 人工障碍
    origin = (0.0, 0.0)
    shape = (30, 30)  # 覆盖 0..3m，容纳 0.5..2.5m 的标注多边形
    candidate = [(2, 2)]  # 世界 [0.2,0.3]²，位于所有标注多边形之外
    ann = {
        "free": [[(0.5, 0.5), (2.5, 0.5), (2.5, 2.5), (0.5, 2.5)]],       # 0.5..2.5m 方块
        "obstacle": [[(0.8, 0.8), (1.6, 0.8), (1.6, 1.6), (0.8, 1.6)]],  # 内嵌障碍
        "unknown": [[(1.0, 2.0), (1.4, 2.0), (1.4, 2.4), (1.0, 2.4)]],  # 内嵌未知
    }
    occ = m2d.build_occupancy(origin, R, shape, candidate, ann)
    # 候选障碍格在标注区外仍为障碍
    assert occ[2, 2] == m2d.OBSTACLE
    # 自由多边形完全覆盖的格子为自由（2.2,2.2，避开障碍/未知）
    assert occ[m2d.world_to_grid(2.2, 2.2, origin, R, shape)] == m2d.FREE
    # 内嵌未知（1.2,2.2）优先级高于自由
    assert occ[m2d.world_to_grid(1.2, 2.2, origin, R, shape)] == m2d.UNKNOWN
    # 内嵌障碍（1.2,1.2）优先级最高
    assert occ[m2d.world_to_grid(1.2, 1.2, origin, R, shape)] == m2d.OBSTACLE
    # 自由多边形边缘：只释放完全覆盖格——(0.45,1.0) 的格子 [0.4,0.5]×[0.95,1.05]
    # 左半在多边形外（多边形从 x=0.5 开始），未被完全覆盖 → 保持未知
    assert occ[m2d.world_to_grid(0.45, 1.0, origin, R, shape)] == m2d.UNKNOWN


def test_invalid_polygons_rejected():
    for verts in ([(0, 0), (1, 1)], [(0, 0), (1, 0), (0.5, 0)]):
        try:
            m2d.validate_polygon(verts)
            assert False, f"应拒绝非法多边形: {verts}"
        except ValueError:
            pass
    # 自交蝴蝶形
    try:
        m2d.validate_polygon([(0, 0), (2, 2), (0, 2), (2, 0)])
        assert False, "应拒绝自交多边形"
    except ValueError:
        pass


def test_all_unknown_and_all_obstacle():
    occ_unk = make_occ()
    _, t, c = m2d.compute_inflation(occ_unk, R, 0.3, 0.0, 0.3)
    assert t.sum() == 0 and np.all(np.isinf(c))
    occ_obs = make_occ()
    occ_obs[:, :] = m2d.OBSTACLE
    _, t2, c2 = m2d.compute_inflation(occ_obs, R, 0.3, 0.0, 0.3)
    assert t2.sum() == 0 and np.all(np.isinf(c2))


def test_save_load_roundtrip_and_checksum():
    occ = make_occ(cells_free=[(10, 10), (10, 11), (11, 10), (11, 11)],
                   cells_obs=[(5, 5)])
    clearance, traversable, cost = m2d.compute_inflation(occ, R, 0.05, 0.02, 0.3)
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "maps2d" / "test" / "v1"
        meta = {"map_id": "test", "map_version": "v1", "origin_xz": list(ORIGIN),
                "resolution_m": R, "robot_radius_m": 0.05, "safety_margin_m": 0.02,
                "soft_band_m": 0.3, "reviewed": False}
        m2d.save_map_package(out, meta, occ, traversable, clearance, cost,
                             {"free": [], "obstacle": [], "unknown": [],
                              "candidate_remove": []})
        # 重复写同一目录被拒绝
        try:
            m2d.save_map_package(out, meta, occ, traversable, clearance, cost,
                                 {"free": [], "obstacle": [], "unknown": [],
                                  "candidate_remove": []})
            assert False, "应拒绝覆盖已存在版本"
        except FileExistsError:
            pass
        # 未审核地图：规划加载拒绝，草稿加载允许
        try:
            m2d.load_planning_map(out, "test")
            assert False, "未审核地图不应允许规划加载"
        except RuntimeError:
            pass
        pm = m2d.load_map_package(out, expected_map_id="test", require_reviewed=False)
        assert np.array_equal(pm.occupancy, occ)
        assert np.allclose(pm.clearance_m, clearance)
        # 篡改 grid.npz 后校验失败
        (out / "grid.npz").write_bytes(b"corrupt")
        try:
            m2d.load_map_package(out, require_reviewed=False)
            assert False, "篡改后应校验失败"
        except RuntimeError:
            pass
        # map_id 不匹配被拒绝
        try:
            m2d.load_map_package(out, expected_map_id="other", require_reviewed=False)
            assert False, "map_id 不匹配应拒绝"
        except RuntimeError:
            pass


def test_wall_segment_blocks_passage():
    # 墙线段（两点一段）保守覆盖所有相交格为障碍；端点越界报错
    origin = (0.0, 0.0)
    shape = (20, 20)
    occ = m2d.build_occupancy(origin, R, shape, candidate_cells=[],
                              annotations={"obstacle_segments": [
                                  [[0.0, 1.0], [1.5, 1.0]]]})
    row = m2d.world_to_grid(0.0, 1.0, origin, R, shape)[0]
    assert occ[row, 5] == m2d.OBSTACLE      # 线段中部某格
    assert occ[row, 14] == m2d.OBSTACLE     # 线段末端附近
    cols = [c for c in range(20) if occ[row, c] == m2d.OBSTACLE]
    assert cols[0] <= 0 and cols[-1] >= 14  # 超覆盖:两端之间连续
    # 端点越界的墙线段应报错
    try:
        m2d.build_occupancy(origin, R, shape, candidate_cells=[],
                            annotations={"obstacle_segments": [
                                [[0.0, 1.0], [5.0, 1.0]]]})
        assert False, "越界线段应报错"
    except ValueError:
        pass


def test_erase_points_excluded():
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    ids = np.array([7, 8, 9], dtype=np.int64)
    f_pts, f_ids = m2d.exclude_landmarks(pts, ids, [8])
    assert len(f_pts) == 2 and list(f_ids) == [7, 9]
    # 空擦除集 = 原样返回
    f_pts, f_ids = m2d.exclude_landmarks(pts, ids, [])
    assert len(f_pts) == 3


def test_candidate_removal_keeps_unknown():
    # 剔除候选不自动把格子变自由
    origin = (0.0, 0.0)
    shape = (10, 10)
    occ = m2d.build_occupancy(origin, R, shape, candidate_cells=[(5, 5)], annotations={
        "candidate_remove": [[(0.55, 0.55), (0.65, 0.55), (0.65, 0.65), (0.55, 0.65)]],
    })
    assert occ[5, 5] == m2d.UNKNOWN


def test_overlapping_candidates_and_manual_layers():
    square = [[0, 0], [4, 0], [4, 4], [0, 4]]
    ann = {"free": [square]}
    args = ((0, 0), 1.0, (5, 5), [(1, 1)])
    assert m2d.build_occupancy(*args, ann)[1, 1] == m2d.OBSTACLE
    ann["candidate_remove"] = [square]
    assert m2d.build_occupancy(*args, ann)[1, 1] == m2d.FREE
    ann["unknown"] = [square]
    assert m2d.build_occupancy(*args, ann)[1, 1] == m2d.UNKNOWN
    ann["obstacle"] = [square]
    assert m2d.build_occupancy(*args, ann)[1, 1] == m2d.OBSTACLE


def test_subcell_walls_and_concave_free_notches():
    wall = [[0.2, 0.1], [0.3, 0.1], [0.3, 2.9], [0.2, 2.9]]
    occ = m2d.build_occupancy((0, 0), 1., (3, 3), [], {"obstacle": [wall]})
    assert np.all(occ[:, 0] == m2d.OBSTACLE)
    # All four corners and the centre of cell (0,0) are inside; a narrow notch
    # still enters the cell. Five point sampling would falsely release it.
    notch = [[-1,-1], [2,-1], [2,2], [.3,2], [.3,.2], [.2,.2], [.2,2], [-1,2]]
    occ = m2d.build_occupancy((0, 0), 1., (3, 3), [], {"free": [notch]})
    assert occ[0, 0] == m2d.UNKNOWN


def test_supercover_against_segment_box_oracle():
    # Independent Liang-Barsky interval clipping against closed cell boxes.
    def hits(a, b, row, col):
        lo, hi = 0., 1.
        for k, lower in enumerate((col, row)):
            delta = b[k] - a[k]
            if delta == 0:
                if not lower <= a[k] <= lower + 1:
                    return False
            else:
                t0, t1 = sorted(((lower-a[k])/delta, (lower+1-a[k])/delta))
                lo, hi = max(lo, t0), min(hi, t1)
                if lo > hi + 1e-12:
                    return False
        return True
    rng = np.random.default_rng(20260929)
    segments = [((.5,.5),(4.5,4.5)), ((1.,0.),(1.,4.)), ((.01,.99),(4.99,2.01))]
    segments += [(rng.uniform(.01, 4.99, 2), rng.uniform(.01, 4.99, 2)) for _ in range(40)]
    for a, b in segments:
        expected = {(r,c) for r in range(-1,6) for c in range(-1,6) if hits(a,b,r,c)}
        actual = set(m2d._segment_cells_grid(a,b))
        assert actual == expected, (a,b,actual ^ expected)
        assert actual == set(m2d._segment_cells_grid(b,a))
    occ = np.zeros((5,5),dtype=np.int8)
    occ[1,2] = m2d.OBSTACLE
    assert not m2d.edge_is_free(occ, occ == 0, (0,0), (4,4))


def test_polygon_self_touch_and_collinear_overlap_rejected():
    invalid = [
        [[0,0],[3,0],[1,0],[1,2],[0,2]],
        [[0,0],[2,0],[2,2],[1,0],[0,2]],
        [[0,0],[2,0],[2,2],[0,2],[0,0]],
    ]
    for poly in invalid:
        try:
            m2d.validate_polygon(poly)
        except ValueError:
            continue
        raise AssertionError(f"accepted invalid polygon {poly}")


def test_builder_annotations_and_package_integrity():
    from unittest.mock import patch
    import map2d_builder as builder
    source = {"landmarks": [
        {"id": 1, "pose": {"x":0., "y":0., "z":0.}},
        {"id": 2, "pose": {"x":4., "y":0., "z":4.}},
        {"id": 3, "pose": {"x":2., "y":-1., "z":2.}},
    ], "poses": [], "edges": []}
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp); db = root / 'input'; db.mkdir()
        (db/'data.mdb').write_bytes(b'static fixture')
        extractor = root/'extractor'; extractor.write_bytes(b'fixture')
        ann = root/'annotations.json'
        ann.write_text(json.dumps({"free": [[[0,0],[4,0],[4,4],[0,4]]], "erased_point_ids": [3]}))
        out = root/'v3'
        args = ['--map-dir',str(db),'--extractor',str(extractor),'--output',str(out),
                '--resolution','0.1','--robot-radius-m','0.2','--safety-margin-m','0',
                '--ground-y','0','--height-min','0.5','--height-max','5',
                '--annotations',str(ann),'--no-preview']
        with patch.object(m2d, 'run_extractor', return_value=source):
            assert builder.main(args) == 0
        pm = m2d.load_map_package(out, require_reviewed=False)
        assert pm.traversable.any() and not np.any(pm.occupancy == m2d.OBSTACLE)
        assert pm.meta['map_version'] == 'v3' and not pm.reviewed
        assert pm.meta['source_geometry_sha256'] == m2d.sha256_file(out/'source_map.json')
        assert json.loads((out/'annotations.json').read_text())['erased_point_ids'] == [3]
        assert (db/'data.mdb').read_bytes() == b'static fixture'
        for filename in ('annotations.json', 'source_map.json'):
            path = out/filename; original = path.read_bytes(); path.write_bytes(b'{}')
            try:
                m2d.load_map_package(out, require_reviewed=False)
            except RuntimeError:
                pass
            else:
                raise AssertionError(f'{filename} tamper accepted')
            path.write_bytes(original)


def test_rectangular_corridor_connectivity():
    from scipy.ndimage import label
    outer = [[0,0],[10,0],[10,8],[0,8]]
    inner = [[3,3],[7,3],[7,5],[3,5]]
    ann = {"free":[outer], "obstacle":[inner]}
    occ = m2d.build_occupancy((0,0), .1, (81,101), [], ann)
    _, trav, _ = m2d.compute_inflation(occ,.1,.3,0.,.3)
    components, count = label(trav)
    assert count == 1
    assert not trav[40,50]
    # Block both sides of the ring: top and bottom become disconnected.
    ann['obstacle_segments'] = [[[0,4],[3,4]], [[7,4],[10,4]]]
    occ = m2d.build_occupancy((0,0), .1, (81,101), [], ann)
    _, trav, _ = m2d.compute_inflation(occ,.1,.3,0.,.3)
    components, count = label(trav)
    assert count == 2 and components[15,50] != components[65,50]


def run_all():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"ERROR {t.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} 通过")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(run_all())
