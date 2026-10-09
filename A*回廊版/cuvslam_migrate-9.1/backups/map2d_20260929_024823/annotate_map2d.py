# -*- coding: utf-8 -*-
"""annotate_map2d.py —— 二维规划地图人工标注工具（阶段 3，纯鼠标操作）

性能设计：7.6 万级特征点不以散点绘制，而是预渲染成 XZ 密度图（imshow），
拖动平移/擦除只重绘轻量图层 → 交互流畅。

操作：
  左键 = 加顶点 / 右键 = 闭合多边形（Free/Obstacle/Unknown/RemoveCand 模式）
  WallSeg 模式：左键点两点 = 一段墙线；右键取消起点
  Erase 模式：按住左键拖动擦除画刷内特征点；右键结束
  中键拖动 = 平移；滚轮 = 缩放
  键盘：1/2/3/4/5/6 切模式，z 撤销顶点，x 删最后元素，w 保存，q 退出，
        [ / ] 缩小/放大橡皮擦半径（0.05~3.0m）
界面标签英文（避免 CJK 字体缺失乱码）；标注顶点保存为米制 (x,z)。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np


class Annotator:
    MODES = [("Free", "free", "#2ca02c"), ("Obstacle", "obstacle", "#111111"),
             ("Unknown", "unknown", "#1f77b4"), ("RemoveCand", "candidate_remove", "#ff7f0e"),
             ("WallSeg", "wallseg", "#444444"), ("Erase", "erase", "#cc0000")]
    MODE_KEYS = {"1": "free", "2": "obstacle", "3": "unknown",
                 "4": "candidate_remove", "5": "wallseg", "6": "erase"}
    ERASE_RADIUS_M = 0.3
    ERASE_RADIUS_MIN = 0.05
    ERASE_RADIUS_MAX = 3.0

    def __init__(self, pts: np.ndarray, candidates: np.ndarray | None,
                 annotations: dict, out_path: Path, landmark_ids=None,
                 cand_ids=None):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button

        matplotlib.rcParams["keymap.fullscreen"] = ""
        matplotlib.rcParams["keymap.save"] = ""
        matplotlib.rcParams["keymap.quit"] = ""

        self.pts = pts
        self.candidates = candidates
        self.annotations = annotations
        self.out_path = out_path
        self.mode = "free"
        self.current: list = []
        self.current_line = None
        self.vertex_labels: list = []
        self.poly_artists = []
        self.wall_start = None
        self.wall_start_marker = None
        self._pan = None
        self.landmark_ids = (landmark_ids if landmark_ids is not None
                             else np.arange(len(pts)))
        self.cand_ids = (cand_ids if cand_ids is not None
                         else np.arange(len(candidates))
                         if candidates is not None else np.array([], dtype=np.int64))
        self.erased = set(annotations.get("erased_point_ids", []))
        self._erasing = False
        self.brush = None
        self._last_redraw = 0.0

        self.fig, self.ax = plt.subplots(figsize=(13, 11))
        plt.subplots_adjust(bottom=0.10)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Z (m)")
        self._rebuild_points()          # 散点底图（锐利；擦除仅在松开时重建）
        self._redraw_existing()
        self._update_title()

        # ---- 底部按钮：等宽平均布局 ----
        self.buttons = {}
        util_actions = [("UndoPt", self._undo_point), ("DelPoly", self._del_poly),
                        ("Save", self._save)]
        n_buttons = len(self.MODES) + len(util_actions)
        w = 0.96 / n_buttons
        for i, (name, mode, color) in enumerate(self.MODES):
            bx = self.fig.add_axes([0.02 + i * w, 0.02, w - 0.008, 0.05])
            self.buttons[mode] = Button(bx, f"{name}({self.MODE_KEYS and mode[:1].upper()})",
                                        color=color, hovercolor="#cccccc")
            self.buttons[mode].on_clicked(lambda _e, m=mode: self._set_mode(m))
        for j, (label, fn) in enumerate(util_actions):
            bx = self.fig.add_axes([0.02 + (len(self.MODES) + j) * w, 0.02,
                                    w - 0.008, 0.05])
            self.buttons[label] = Button(bx, label, color="#eeeeee",
                                         hovercolor="#cccccc")
            self.buttons[label].on_clicked(lambda _e, f=fn: f())

        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        print("操作: 左键加点/右键闭合; 中键拖动平移, 滚轮缩放; Erase 按住左键擦除,"
              " [/] 调半径; 键盘 1-6 切模式 z撤销 x删除 w保存 q退出")

    # ------------------------------------------------------------------ 底图
    def _rebuild_points(self):
        """按擦除集重建点云/候选散点底图（锐利；擦除仅在松开时重建一次）。"""
        erased_arr = np.array(sorted(self.erased), dtype=np.int64)
        vis = ~np.isin(self.landmark_ids, erased_arr)
        pts_v = self.pts[vis]
        x_min, x_max = self.pts[:, 0].min(), self.pts[:, 0].max()
        z_min, z_max = self.pts[:, 2].min(), self.pts[:, 2].max()
        if getattr(self, "sc_pts", None) is None:
            self.sc_pts = self.ax.scatter(pts_v[:, 0], pts_v[:, 2], s=0.2, c="0.4",
                                          rasterized=True, zorder=1)
            self.sc_cand = None
            if self.candidates is not None:
                cv = ~np.isin(self.cand_ids, erased_arr)
                self.sc_cand = self.ax.scatter(
                    self.candidates[cv, 0], self.candidates[cv, 2], s=1.5,
                    c="tab:red", rasterized=True, zorder=2)
        else:
            self.sc_pts.set_offsets(np.column_stack([pts_v[:, 0], pts_v[:, 2]]))
            if self.sc_cand is not None:
                cv = ~np.isin(self.cand_ids, erased_arr)
                cands = self.candidates[cv]
                self.sc_cand.set_offsets(
                    np.column_stack([cands[:, 0], cands[:, 2]]))
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(z_min, z_max)
        self.ax.set_aspect("equal", adjustable="box")

    # ------------------------------------------------------------------ 工具
    def _throttled_redraw(self):
        now = time.monotonic()
        if now - self._last_redraw < 0.08:
            return
        self._last_redraw = now
        self.fig.canvas.draw_idle()

    def _set_mode(self, m):
        self.mode = m
        self._update_title()

    def _update_title(self):
        r = self.ERASE_RADIUS_M
        self.ax.set_title(
            f"MODE: {self.mode}   erased={len(self.erased)}   brush={r:.2f}m"
            f"   (left=vertex / right=close; [/] brush radius)",
            fontsize=10)
        self.fig.canvas.draw_idle()

    def _redraw_existing(self):
        from matplotlib.patches import Polygon
        for mode, polys in self.annotations.items():
            if mode == "obstacle_segments":
                for seg in polys:
                    (line,) = self.ax.plot([seg[0][0], seg[1][0]],
                                           [seg[0][1], seg[1][1]],
                                           color="#111111", linewidth=5,
                                           solid_capstyle="round", zorder=3)
                    self.poly_artists.append((line, "obstacle_segments"))
                continue
            for v in polys:
                arr = np.asarray(v)
                color = dict(self.MODES).get(mode, "gray")
                patch = self.ax.add_patch(Polygon(
                    arr, closed=True, fill=True, alpha=0.25,
                    facecolor=color, edgecolor=color, zorder=3))
                self.poly_artists.append((patch, mode))

    # ------------------------------------------------------------------ 交互
    def _on_click(self, event):
        if event.inaxes is not self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        if event.button == 2:
            self._pan = (event.xdata, event.ydata,
                         self.ax.get_xlim(), self.ax.get_ylim())
            return
        if self.mode == "erase":
            self._on_click_erase(event)
            return
        if self.mode == "wallseg":
            self._on_click_wallseg(event)
            return
        if event.button == 1:
            self.current.append((float(event.xdata), float(event.ydata)))
            self._draw_current()
        elif event.button == 3:
            self._close_current()

    def _on_release(self, event):
        if event.button == 2:
            self._pan = None
        elif event.button == 1 and self.mode == "erase" and self._erasing:
            self._erasing = False
            self._rebuild_points()
            self.fig.canvas.draw_idle()
            print(f"擦除结束，累计擦除 {len(self.erased)} 点")

    def _on_motion(self, event):
        self._on_motion_pan(event)
        self._on_motion_erase(event)

    def _on_motion_pan(self, event):
        if self._pan is None or event.inaxes is not self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        x0, z0, (xl0, xl1), (zl0, zl1) = self._pan
        dx = event.xdata - x0
        dz = event.ydata - z0
        self.ax.set_xlim(xl0 - dx, xl1 - dx)
        self.ax.set_ylim(zl0 - dz, zl1 - dz)
        self._throttled_redraw()

    def _on_motion_erase(self, event):
        if self.mode != "erase":
            return
        from matplotlib.patches import Circle
        if event.inaxes is not self.ax or event.xdata is None:
            if self.brush is not None:
                self.brush.set_visible(False)
                self._throttled_redraw()
            return
        if self.brush is None:
            self.brush = Circle((0, 0), self.ERASE_RADIUS_M, fill=False,
                                edgecolor="#cc0000", linewidth=1.2, alpha=0.8)
            self.ax.add_patch(self.brush)
        self.brush.center = (event.xdata, event.ydata)
        self.brush.set_radius(self.ERASE_RADIUS_M)
        self.brush.set_visible(True)
        if self._erasing:
            self._apply_erase(event.xdata, event.ydata)
            self._throttled_redraw()

    def _apply_erase(self, x, z):
        d2 = (self.pts[:, 0] - x) ** 2 + (self.pts[:, 2] - z) ** 2
        hit = np.where(d2 <= self.ERASE_RADIUS_M ** 2)[0]
        for idx in hit:
            self.erased.add(int(self.landmark_ids[idx]))

    def _on_click_erase(self, event):
        if event.button == 1:
            self._erasing = True
            self._apply_erase(event.xdata, event.ydata)
        elif event.button == 3:
            self._erasing = False
            self._rebuild_points()
            self.fig.canvas.draw_idle()

    # ------------------------------------------------------------------ 多边形
    def _draw_current(self):
        if self.current_line is not None:
            self.current_line.remove()
            self.current_line = None
        for lb in self.vertex_labels:
            lb.remove()
        self.vertex_labels = []
        if not self.current:
            return
        color = dict(self.MODES).get(self.mode, "gray")
        xs = [p[0] for p in self.current] + [self.current[0][0]]
        zs = [p[1] for p in self.current] + [self.current[0][1]]
        (self.current_line,) = self.ax.plot(
            xs, zs, color=color, linewidth=1.4, linestyle="--",
            marker="o", markersize=6, markerfacecolor=color, alpha=0.9, zorder=4)
        for i, (x, z) in enumerate(self.current):
            self.vertex_labels.append(self.ax.annotate(
                str(i + 1), (x, z), textcoords="offset points", xytext=(6, 6),
                fontsize=9, color=color, fontweight="bold", zorder=5))
        self.fig.canvas.draw_idle()

    def _clear_current(self):
        self.current = []
        if self.current_line is not None:
            self.current_line.remove()
            self.current_line = None
        for lb in self.vertex_labels:
            lb.remove()
        self.vertex_labels = []
        self._update_title()

    def _close_current(self):
        from matplotlib.patches import Polygon
        if len(self.current) < 3:
            print("polygon needs >=3 vertices, discarded")
            self._clear_current()
            return
        try:
            import map2d_data as m2d
            m2d.validate_polygon(self.current)
        except ValueError as e:
            print(f"invalid polygon ({e}), discarded")
            self._clear_current()
            return
        self.annotations[self.mode].append([list(p) for p in self.current])
        arr = np.asarray(self.current)
        color = dict(self.MODES).get(self.mode, "gray")
        patch = self.ax.add_patch(Polygon(arr, closed=True, fill=True, alpha=0.25,
                                          facecolor=color, edgecolor=color, zorder=3))
        self.poly_artists.append((patch, self.mode))
        print(f"saved {self.mode} polygon ({len(self.current)} vertices)")
        self._clear_current()

    def _undo_point(self):
        if self.current:
            self.current.pop()
            self._draw_current()

    def _on_click_wallseg(self, event):
        if event.button == 3:
            self._cancel_wall_start()
            return
        if event.button != 1:
            return
        pt = (float(event.xdata), float(event.ydata))
        if self.wall_start is None:
            self.wall_start = pt
            (self.wall_start_marker,) = self.ax.plot(
                [pt[0]], [pt[1]], marker="o", markersize=8,
                markerfacecolor="#444444", markeredgecolor="red", zorder=4)
            self.fig.canvas.draw_idle()
        else:
            seg = [list(self.wall_start), list(pt)]
            self.annotations.setdefault("obstacle_segments", []).append(seg)
            (line,) = self.ax.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]],
                                   color="#111111", linewidth=5,
                                   solid_capstyle="round", zorder=3)
            self.poly_artists.append((line, "obstacle_segments"))
            print(f"saved wall segment ({len(self.annotations['obstacle_segments'])} total)")
            self._cancel_wall_start()

    def _cancel_wall_start(self):
        self.wall_start = None
        if self.wall_start_marker is not None:
            self.wall_start_marker.remove()
            self.wall_start_marker = None
        self._update_title()

    def _del_poly(self):
        if self.poly_artists:
            patch, key = self.poly_artists.pop()
            patch.remove()
            self.annotations[key].pop()
            print(f"deleted last {key} item")
            self.fig.canvas.draw_idle()

    def _save(self):
        self.annotations["erased_point_ids"] = sorted(int(i) for i in self.erased)
        self.out_path.write_text(
            json.dumps(self.annotations, ensure_ascii=False, indent=2),
            encoding="utf-8")
        print(f"saved -> {self.out_path} (erased {len(self.erased)} points)")

    # ------------------------------------------------------------------ 键盘/滚轮
    def _on_key(self, event):
        k = event.key
        if k is None:
            return
        if k in self.MODE_KEYS:
            self._set_mode(self.MODE_KEYS[k])
        elif k == "z":
            self._undo_point()
        elif k == "x":
            self._del_poly()
        elif k == "w":
            self._save()
        elif k == "[":
            self.ERASE_RADIUS_M = max(self.ERASE_RADIUS_MIN, self.ERASE_RADIUS_M / 1.5)
            self._update_title()
            print(f"brush radius = {self.ERASE_RADIUS_M:.2f}m")
        elif k == "]":
            self.ERASE_RADIUS_M = min(self.ERASE_RADIUS_MAX, self.ERASE_RADIUS_M * 1.5)
            self._update_title()
            print(f"brush radius = {self.ERASE_RADIUS_M:.2f}m")
        elif k == "q":
            self._save()
            sys.exit(0)

    def _on_scroll(self, event):
        if event.inaxes is not self.ax:
            return
        factor = 1.0 / 1.25 if event.button == "up" else 1.25
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        xd, yd = event.xdata, event.ydata
        self.ax.set_xlim([xd - (xd - xlim[0]) * factor, xd + (xlim[1] - xd) * factor])
        self.ax.set_ylim([yd - (yd - ylim[0]) * factor, yd + (ylim[1] - yd) * factor])
        self._throttled_redraw()


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="二维规划地图人工标注（XZ 俯视）")
    p.add_argument("--source-json", required=True,
                   help="导出的 source_map.json（含 landmarks 全局 XYZ）")
    p.add_argument("--out", default="annotations.json", help="标注输出文件")
    p.add_argument("--annotations", default=None, help="载入已有标注继续编辑")
    p.add_argument("--ground-y", type=float, default=None,
                   help="地面高度（显示候选障碍红点用；不给则显示全部点）")
    p.add_argument("--height-min", type=float, default=0.5)
    p.add_argument("--height-max", type=float, default=5.0)
    args = p.parse_args(argv)

    data = json.loads(Path(args.source_json).read_text(encoding="utf-8"))
    pts = np.array([[lm["pose"]["x"], lm["pose"]["y"], lm["pose"]["z"]]
                    for lm in data["landmarks"]], dtype=np.float64)
    ids = np.array([lm["id"] for lm in data["landmarks"]], dtype=np.int64)
    candidates = None
    cand_ids = None
    if args.ground_y is not None:
        h = args.ground_y - pts[:, 1]
        mask = (h >= args.height_min) & (h <= args.height_max)
        candidates = pts[mask]
        cand_ids = ids[mask]

    annotations = {"free": [], "obstacle": [], "unknown": [], "candidate_remove": [],
                   "obstacle_segments": []}
    if args.annotations:
        loaded = json.loads(Path(args.annotations).read_text(encoding="utf-8"))
        for k in annotations:
            annotations[k] = list(loaded.get(k, []))
        if "erased_point_ids" in loaded:
            annotations["erased_point_ids"] = list(loaded["erased_point_ids"])

    Annotator(pts, candidates, annotations, Path(args.out),
              landmark_ids=ids, cand_ids=cand_ids)
    import matplotlib.pyplot as plt
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
