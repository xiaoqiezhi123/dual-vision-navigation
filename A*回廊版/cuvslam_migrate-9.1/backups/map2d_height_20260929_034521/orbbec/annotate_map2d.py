# -*- coding: utf-8 -*-
"""annotate_map2d.py —— 二维规划地图人工标注工具（阶段 3，纯鼠标操作）

性能设计：概览按屏幕像素聚合，局部放大显示原始点；橡皮擦使用 KDTree，
笔刷用 blit 局部刷新。仅显示聚合，保存/擦除/规划始终使用原始点和米制坐标。

操作：
  左键 = 加顶点 / 右键 = 闭合多边形（Free/Obstacle/Unknown/RemoveCand 模式）
  WallSeg 模式：左键点两点 = 一段墙线；右键取消起点
  Erase 模式：按住左键拖动擦除画刷内特征点；右键结束
  中键拖动 = 平移；滚轮 = 缩放
  键盘：1/2/3/4/5/6 切模式，z 撤销顶点，x 删最后元素，w 保存，q 退出，
        [ / ] 缩小/放大橡皮擦半径（0.05~3.0m），也可使用滑块/输入框/加减按钮
底部中英双语按钮、悬停说明、? 帮助；无中文字体时回退英文。顶点保存为米制 (x,z)。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from map2d_display import PointCloudIndex


class Annotator:
    MODES = [("Free", "free", "#2ca02c"), ("Obstacle", "obstacle", "#111111"),
             ("Unknown", "unknown", "#1f77b4"), ("RemoveCand", "candidate_remove", "#ff7f0e"),
             ("WallSeg", "wallseg", "#444444"), ("Erase", "erase", "#cc0000")]
    MODE_KEYS = {"1": "free", "2": "obstacle", "3": "unknown",
                 "4": "candidate_remove", "5": "wallseg", "6": "erase"}
    COLORS = {mode: color for _, mode, color in MODES}
    POLYGON_KEYS = ("free", "obstacle", "unknown", "candidate_remove")
    ERASE_RADIUS_M = 0.3
    ERASE_RADIUS_MIN = 0.05
    ERASE_RADIUS_MAX = 3.0
    LABELS_ZH = {"free": "自由区", "obstacle": "障碍区", "unknown": "未知区",
                 "candidate_remove": "剔除候选", "wallseg": "墙线段", "erase": "擦除特征点",
                 "UndoPt": "撤销顶点", "DelPoly": "删除末项", "Undo": "撤销操作",
                 "Redo": "重做操作", "Home": "返回全图", "Save": "保存标注", "Help": "操作说明"}
    HELP_ZH = {
        "free": "Free：圈出已确认可走的物理空间；左键加点、右键闭合。机器人尺寸由生成地图时处理。",
        "obstacle": "Obstacle：圈出墙体或固定障碍；左键加点、右键闭合。优先级最高。",
        "unknown": "Unknown：圈出未核验区域；左键加点、右键闭合。规划时禁止进入。",
        "candidate_remove": "RemoveCand：圈选剔除自动候选障碍的范围；不会删除人工障碍，也不会自动变成自由区。",
        "wallseg": "WallSeg：左键依次点两个端点，补一段墙；右键取消起点。",
        "erase": "Erase：按住左键拖动擦除原始特征点，松开更新画面；下方调半径，可用 Ctrl+Z 撤销。",
        "UndoPt": "UndoPt / z：撤销正在绘制、尚未闭合的多边形的最后一个顶点。",
        "DelPoly": "DelPoly / x：删除最后一个显示的多边形或墙线段；可用 Undo 恢复。",
        "Undo": "Undo / Ctrl+Z：撤销一次完整标注、删除或擦除操作；最多保留 100 次。",
        "Redo": "Redo / Ctrl+Y：恢复刚撤销的操作；开始新编辑后清空重做记录。",
        "Home": "Home / h：恢复全图视野，不改变已保存或正在编辑的标注。",
        "Save": "Save / w：保存已完成的标注和擦除记录；未闭合的多边形不会保存。",
        "Help": "Help / ?：打开完整按钮说明和快捷键。中键拖动平移，滚轮缩放，Esc 取消当前绘制。",
        "radius": "擦除半径单位为米（不是直径）；范围 0.05–3.00m。拖动滑块、输入数字后回车，或点击 − / +。",
    }
    HELP_EN = {
        "free": "Free: mark confirmed physical free space. Left click adds vertices; right click closes. Inflation is applied later.",
        "obstacle": "Obstacle: mark walls or fixed obstacles. Left click adds vertices; right click closes. Highest priority.",
        "unknown": "Unknown: mark unverified regions. These remain blocked for planning.",
        "candidate_remove": "RemoveCand: remove automatic candidates in the polygon. Does not erase manual obstacles or declare free space.",
        "wallseg": "WallSeg: two left clicks add a wall segment; right click cancels the first endpoint.",
        "erase": "Erase: hold left mouse button and drag. Release refreshes points. Radius below is in metres. Ctrl+Z restores a stroke.",
        "UndoPt": "UndoPt / z: remove the last vertex of the unfinished polygon.",
        "DelPoly": "DelPoly / x: delete the last displayed polygon or wall segment. Undo restores it.",
        "Undo": "Undo / Ctrl+Z: undo one completed annotation, deletion or eraser stroke (up to 100 actions).",
        "Redo": "Redo / Ctrl+Y: redo an undone action; a new edit clears the redo history.",
        "Home": "Home / h: restore the full view without editing annotations.",
        "Save": "Save / w: save completed annotations and erased IDs. Unfinished polygons are not saved.",
        "Help": "Help / ?: show controls. Middle drag pans, wheel zooms, Esc cancels unfinished drawing.",
        "radius": "Eraser radius (not diameter), 0.05–3.00 metres. Drag slider, enter a number and press Enter, or use - / +.",
    }

    def __init__(self, pts: np.ndarray, candidates: np.ndarray | None,
                 annotations: dict, out_path: Path, landmark_ids=None,
                 cand_ids=None, backend="TkAgg", display_mode="auto", erase_radius_m=0.3):
        import matplotlib
        matplotlib.use(backend)
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, Slider, TextBox
        from matplotlib import font_manager

        cjk_fonts = [f.name for f in font_manager.fontManager.ttflist
                     if "CJK" in f.name or "WenQuanYi" in f.name]
        self._ui_font = cjk_fonts[0] if cjk_fonts else "DejaVu Sans"
        self._has_cjk = bool(cjk_fonts)
        self._help_fig = None
        self._hover_key = None
        self._syncing_radius = False
        if not np.isfinite(erase_radius_m) or not self.ERASE_RADIUS_MIN <= erase_radius_m <= self.ERASE_RADIUS_MAX:
            raise ValueError("擦除半径必须在 0.05–3.00m 之间")
        self.ERASE_RADIUS_M = round(float(erase_radius_m), 2)

        matplotlib.rcParams["keymap.fullscreen"] = ""
        matplotlib.rcParams["keymap.save"] = ""
        matplotlib.rcParams["keymap.quit"] = ""

        self.pts = pts
        self.candidates = candidates
        self.annotations = annotations
        for key in (*self.POLYGON_KEYS, "obstacle_segments"):
            self.annotations.setdefault(key, [])
        self.out_path = out_path
        self.mode = "free"
        self.current: list = []
        self.current_line = None
        self.vertex_labels: list = []
        self.poly_artists = []
        self.wall_start = None
        self.wall_start_marker = None
        self._pan = None
        self._pan_image = None
        self._pan_shift = (0., 0.)
        self.landmark_ids = (landmark_ids if landmark_ids is not None
                             else np.arange(len(pts)))
        self.cand_ids = (cand_ids if cand_ids is not None
                         else np.arange(len(candidates))
                         if candidates is not None else np.array([], dtype=np.int64))
        self.erased = set(annotations.get("erased_point_ids", []))
        self._erasing = False
        self.brush = None
        self._last_redraw = 0.0
        self.display_mode = display_mode
        self.index = PointCloudIndex(pts, self.landmark_ids, self.cand_ids)
        self._background = None
        self._frame_background = None
        self._capturing_frame = False
        self._updating_view = False
        self._view_pending = False
        self._history, self._redo_history = [], []
        self._erase_before = None
        self._stroke_last = None

        self.fig, self.ax = plt.subplots(figsize=(13, 11))
        plt.subplots_adjust(bottom=0.29)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Z (m)")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_autoscale_on(False)
        low, high = self.index.xz.min(axis=0), self.index.xz.max(axis=0)
        pad = np.maximum((high-low)*0.02, 0.1)
        self._home_limits = ((low[0]-pad[0], high[0]+pad[0]), (low[1]-pad[1], high[1]+pad[1]))
        self.ax.set_xlim(self._home_limits[0]); self.ax.set_ylim(self._home_limits[1])
        self.sc_pts = self.ax.scatter([], [], s=1, c="0.4", linewidths=0, zorder=1)
        self.sc_cand = self.ax.scatter([], [], s=3, c="tab:red", linewidths=0, zorder=2)
        self.pixel_pts, = self.ax.plot([], [], linestyle="", marker=",", color="0.35", zorder=1)
        self.pixel_cand, = self.ax.plot([], [], linestyle="", marker=",", color="tab:red", zorder=2)
        self.point_image = self.ax.imshow(np.ones((2,2,3)), origin="lower",
                                         interpolation="nearest", zorder=1,
                                         extent=[*self._home_limits[0], *self._home_limits[1]])
        self._view_timer = self.fig.canvas.new_timer(interval=33)
        self._view_timer.single_shot = True
        self._view_timer.add_callback(self._flush_view)
        self.ax.callbacks.connect("xlim_changed", self._queue_view)
        self.ax.callbacks.connect("ylim_changed", self._queue_view)
        self.fig.canvas.mpl_connect("resize_event", self._queue_view)
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        self._rebuild_points()
        self._redraw_existing()
        self._update_title()

        # ---- Two rows of labelled actions, a radius row, and contextual help ----
        self.buttons = {}
        self._control_help = {}
        util_actions = [("UndoPt", self._undo_point), ("DelPoly", self._del_poly),
                        ("Undo", self._undo), ("Redo", self._redo),
                        ("Home", self._home), ("Save", self._save), ("Help", self._show_help)]
        w = 0.96 / len(self.MODES)
        for i, (name, mode, color) in enumerate(self.MODES):
            bx = self.fig.add_axes([0.02 + i * w, 0.20, w - 0.008, 0.045])
            label = f"{name} ({i+1})" + (f"\n{self.LABELS_ZH[mode]}" if self._has_cjk else "")
            self.buttons[mode] = Button(bx, label,
                                        color=color, hovercolor="#cccccc")
            self.buttons[mode].label.set_fontsize(10)
            self.buttons[mode].label.set_fontfamily(self._ui_font)
            if mode in ("obstacle", "wallseg"):
                self.buttons[mode].label.set_color("white")
            self.buttons[mode].on_clicked(lambda _e, m=mode: self._set_mode(m))
            self._control_help[bx] = mode
        w = 0.96 / len(util_actions)
        for j, (label, fn) in enumerate(util_actions):
            bx = self.fig.add_axes([0.02 + j * w, 0.14, w - 0.008, 0.045])
            caption = label + (f"\n{self.LABELS_ZH[label]}" if self._has_cjk else "")
            self.buttons[label] = Button(bx, caption, color="#eeeeee",
                                         hovercolor="#cccccc")
            self.buttons[label].label.set_fontfamily(self._ui_font)
            self.buttons[label].label.set_fontsize(10)
            self.buttons[label].on_clicked(lambda _e, f=fn: f())
            self._control_help[bx] = label

        slider_ax = self.fig.add_axes([0.17, 0.087, 0.44, 0.022])
        self.radius_slider = Slider(slider_ax, "Radius (m)",
                                    self.ERASE_RADIUS_MIN, self.ERASE_RADIUS_MAX,
                                    valinit=self.ERASE_RADIUS_M, valstep=0.01, valfmt="%.2f")
        self.radius_slider.label.set_fontfamily(self._ui_font)
        if self._has_cjk:
            self.radius_slider.label.set_text("擦除半径 (m)")
        self.radius_slider.drawon = False
        self.radius_slider.on_changed(self._set_erase_radius)
        box_ax = self.fig.add_axes([0.79, 0.077, 0.085, 0.038])
        self.radius_box = TextBox(box_ax, "", initial=f"{self.ERASE_RADIUS_M:.2f}", textalignment="center")
        self.radius_box.drawon = False
        self.radius_box.on_submit(self._submit_radius)
        for label, x, factor in (("Radius-", 0.69, 1/1.5), ("Radius+", 0.91, 1.5)):
            bx = self.fig.add_axes([x, 0.077, 0.055, 0.038])
            self.buttons[label] = Button(bx, "−" if factor < 1 else "+", color="#eeeeee")
            self.buttons[label].on_clicked(lambda _e, f=factor: self._scale_erase_radius(f))
            self._control_help[bx] = "radius"
        self._control_help[slider_ax] = self._control_help[box_ax] = "radius"
        self.help_text = self.fig.text(0.025, 0.035, "", va="center", fontsize=10,
                                       fontfamily=self._ui_font, color="#333333")
        self._set_help_text(self.mode)
        self.fig.canvas.draw_idle()

        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_control_hover)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        print("操作: 左键加点/右键闭合; 中键拖动平移, 滚轮缩放; Erase 按住左键擦除,"
              " [/] 调半径; 1-6切模式 z撤销顶点 x删除 Ctrl+Z/Y撤销/重做 h全图 w保存 q退出")

    def _set_help_text(self, key=None, message=None):
        if not hasattr(self, "help_text"):
            return
        help_map = self.HELP_ZH if self._has_cjk else self.HELP_EN
        text = message or help_map.get(key, help_map[self.mode])
        if self.help_text.get_text() != text:
            self.help_text.set_text(text)
            self.fig.canvas.draw_idle()

    def _on_control_hover(self, event):
        key = self._control_help.get(event.inaxes, self.mode)
        if key != self._hover_key:
            self._hover_key = key
            self._set_help_text(key)

    def _set_erase_radius(self, value):
        if self._syncing_radius:
            return
        try:
            radius = float(value)
            if not np.isfinite(radius) or not self.ERASE_RADIUS_MIN <= radius <= self.ERASE_RADIUS_MAX:
                raise ValueError
        except (TypeError, ValueError):
            self._sync_radius_controls()
            self._set_help_text(message=("半径输入无效，请填写 0.05–3.00 米。" if self._has_cjk else
                                         "Invalid radius: enter 0.05–3.00 metres."))
            return
        self.ERASE_RADIUS_M = round(radius, 2)
        self._stroke_last = None
        if self.brush is not None:
            self.brush.set_radius(self.ERASE_RADIUS_M)
        self._sync_radius_controls()
        self._set_help_text("radius")
        self._update_title()

    def _sync_radius_controls(self):
        self._syncing_radius = True
        try:
            if hasattr(self, "radius_slider"):
                self.radius_slider.set_val(self.ERASE_RADIUS_M)
            if hasattr(self, "radius_box"):
                self.radius_box.set_val(f"{self.ERASE_RADIUS_M:.2f}")
        finally:
            self._syncing_radius = False

    def _submit_radius(self, text):
        self._set_erase_radius(text)

    def _scale_erase_radius(self, factor):
        self._set_erase_radius(np.clip(self.ERASE_RADIUS_M * factor,
                                      self.ERASE_RADIUS_MIN, self.ERASE_RADIUS_MAX))

    def _show_help(self):
        import matplotlib.pyplot as plt
        if self._help_fig is None or not plt.fignum_exists(self._help_fig.number):
            self._help_fig = plt.figure(figsize=(12, 8))
            self._help_fig.canvas.manager.set_window_title("Map2D controls")
            title = "二维地图标注：按钮与操作说明" if self._has_cjk else "Map2D controls"
            self._help_fig.text(.035, .95, title, fontsize=16, fontfamily=self._ui_font)
            help_map = self.HELP_ZH if self._has_cjk else self.HELP_EN
            for i, key in enumerate([m[1] for m in self.MODES] +
                                    ["UndoPt", "DelPoly", "Undo", "Redo", "Home", "Save", "Help", "radius"]):
                self._help_fig.text(.035, .89-i*.052, help_map[key], fontsize=10,
                                    va="top", fontfamily=self._ui_font, wrap=True)
            note = ("灰点＝原始特征；红点＝高度带内候选障碍。空白不代表自由。擦除不会自动释放自由区。\n"
                    "中键平移，滚轮缩放；多边形左键加点、右键闭合；w 保存，q 保存并退出。" if self._has_cjk else
                    "Grey = features; red = obstacle candidates. Empty space is not confirmed free.\n"
                    "Middle drag pans; wheel zooms. Left adds vertices, right closes. w saves; q saves and exits.")
            self._help_fig.text(.035,.06,note,fontsize=10,fontfamily=self._ui_font,color="#555555")
        self._help_fig.canvas.draw_idle()
        if "agg" != str(plt.get_backend()).lower():
            self._help_fig.show()

    # ------------------------------------------------------------------ 底图
    def _rebuild_points(self):
        """Update erasures without resetting the user's zoom/pan."""
        self.index.set_erased(self.erased)
        self._refresh_view()

    def _queue_view(self, *events):
        if self._updating_view:
            return
        if events and getattr(events[0], "name", None) == "resize_event":
            self._frame_background = None
        self._background = None
        if not self._view_pending:
            self._view_pending = True
            self._view_timer.start()

    def _flush_view(self):
        self._view_timer.stop()
        self._view_pending = False
        self._refresh_view()

    def _refresh_view(self):
        self._updating_view = True
        try:
            xl, zl = self.ax.get_xlim(), self.ax.get_ylim()
            idx = self.index.visible(xl, zl)
            raster = self.display_mode == "density"
            self.display_aggregated = self.display_mode == "auto" and len(idx) > 12000
            self.point_image.set_visible(raster)
            self.sc_pts.set_visible(not raster and not self.display_aggregated)
            self.sc_cand.set_visible(not raster and not self.display_aggregated)
            self.pixel_pts.set_visible(self.display_aggregated)
            self.pixel_cand.set_visible(self.display_aggregated)
            width = max(64, min(1200, int(self.ax.bbox.width)))
            height = max(64, min(1200, int(self.ax.bbox.height)))
            if raster:
                self.point_image.set_data(self.index.raster(idx, xl, zl, width, height))
                self.point_image.set_extent([*xl, *zl])
            elif self.display_aggregated:
                grey, red = self.index.screen_indices(idx, xl, zl, width, height)
                self.pixel_pts.set_data(self.index.xz[grey].T)
                self.pixel_cand.set_data(self.index.xz[red].T)
            else:
                self.sc_pts.set_offsets(self.index.xz[idx])
                self.sc_cand.set_offsets(self.index.xz[idx[self.index.candidate[idx]]])
            self._background = None
            self._draw_view()
        finally:
            self._updating_view = False

    def _on_draw(self, event):
        if self._capturing_frame:
            return
        self._frame_background = None
        if self.fig.canvas.supports_blit:
            self._background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
            if self.brush is not None and self.brush.get_visible():
                self.ax.draw_artist(self.brush)

    def _draw_view(self):
        """Redraw axes/ticks from a cached frame without repainting all buttons."""
        canvas = self.fig.canvas
        if not canvas.supports_blit:
            canvas.draw_idle()
            return
        if self._frame_background is None:
            self._capturing_frame = True
            self.ax.set_visible(False)
            try:
                canvas.draw()
                self._frame_background = canvas.copy_from_bbox(self.fig.bbox)
            finally:
                self.ax.set_visible(True)
                self._capturing_frame = False
        canvas.restore_region(self._frame_background)
        self.ax.draw(canvas.get_renderer())
        self._background = canvas.copy_from_bbox(self.ax.bbox)
        if self.brush is not None and self.brush.get_visible():
            self.ax.draw_artist(self.brush)
        canvas.blit(self.fig.bbox)

    def _draw_brush(self):
        if self._background is not None and self.fig.canvas.supports_blit:
            self.fig.canvas.restore_region(self._background)
            if self.brush is not None and self.brush.get_visible():
                self.ax.draw_artist(self.brush)
            self.fig.canvas.blit(self.ax.bbox)
        else:
            self.fig.canvas.draw_idle()

    def _throttled_redraw(self):
        self._queue_view()

    def _home(self):
        self.ax.set_xlim(self._home_limits[0])
        self.ax.set_ylim(self._home_limits[1])
        self._flush_view()

    def _set_mode(self, m):
        self._finish_erase()
        self._clear_current()
        self._cancel_wall_start()
        self.mode = m
        if self.brush is not None:
            self.brush.set_visible(False)
        self._set_help_text(m)
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
        for artist, _ in self.poly_artists:
            artist.remove()
        self.poly_artists.clear()
        for mode in (*self.POLYGON_KEYS, "obstacle_segments"):
            polys = self.annotations.get(mode, [])
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
                color = self.COLORS.get(mode, "gray")
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
            self._view_timer.stop()
            self._view_pending = False
            self._pan = (event.x, event.y, self.ax.get_xlim(), self.ax.get_ylim(),
                         self.ax.transData.inverted().frozen())
            self._pan_shift = (0., 0.)
            if self.fig.canvas.supports_blit:
                buffer = np.asarray(self.fig.canvas.buffer_rgba())
                x0, y0, x1, y1 = np.round(self.ax.bbox.extents).astype(int)
                top, bottom = buffer.shape[0]-y1, buffer.shape[0]-y0
                self._pan_rect = (top, bottom, x0, x1)
                self._pan_image = buffer[top:bottom, x0:x1].copy()
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
            if self._pan is not None:
                x0, y0, (xl0, xl1), (zl0, zl1), inverse = self._pan
                sx, sy = self._pan_shift
                dx, dz = inverse.transform((x0+sx, y0+sy)) - inverse.transform((x0, y0))
                self.ax.set_xlim(xl0-dx, xl1-dx)
                self.ax.set_ylim(zl0-dz, zl1-dz)
            self._pan = None
            self._pan_image = None
            self._flush_view()
        elif event.button == 1:
            self._finish_erase()

    def _on_motion(self, event):
        if self._pan is not None:
            self._on_motion_pan(event)
        else:
            self._on_motion_erase(event)

    def _on_motion_pan(self, event):
        if self._pan is None or event.x is None or event.y is None:
            return
        x0, y0, (xl0, xl1), (zl0, zl1), inverse = self._pan
        self._pan_shift = (event.x-x0, event.y-y0)
        if self._pan_image is not None:
            # Translate cached pixels while dragging. Newly exposed areas are
            # filled with fresh source points on release; no geometry is edited.
            buffer = np.asarray(self.fig.canvas.buffer_rgba())
            top, bottom, left, right = self._pan_rect
            target = buffer[top:bottom, left:right]
            source = self._pan_image
            if target.shape != source.shape:
                return
            target[:] = 255
            sx, sy = int(round(event.x-x0)), int(round(y0-event.y))
            h, w = source.shape[:2]
            tx, ty = max(0,sx), max(0,sy)
            ox, oy = max(0,-sx), max(0,-sy)
            width, height = w-abs(sx), h-abs(sy)
            if width > 0 and height > 0:
                target[ty:ty+height, tx:tx+width] = source[oy:oy+height, ox:ox+width]
            self._background = None
            self.fig.canvas.blit(self.ax.bbox)
            return
        dx, dz = inverse.transform((event.x, event.y)) - inverse.transform((x0, y0))
        self.ax.set_xlim(xl0-dx, xl1-dx)
        self.ax.set_ylim(zl0-dz, zl1-dz)
        self._queue_view()

    def _on_motion_erase(self, event):
        if self.mode != "erase":
            return
        from matplotlib.patches import Circle
        if event.inaxes is not self.ax or event.xdata is None:
            if self.brush is not None:
                self.brush.set_visible(False)
                self._draw_brush()
            self._stroke_last = None
            return
        if self.brush is None:
            self.brush = Circle((0, 0), self.ERASE_RADIUS_M, fill=False,
                                edgecolor="#cc0000", linewidth=1.2, alpha=0.8,
                                animated=self.fig.canvas.supports_blit)
            self.ax.add_patch(self.brush)
        self.brush.center = (event.xdata, event.ydata)
        self.brush.set_radius(self.ERASE_RADIUS_M)
        self.brush.set_visible(True)
        if self._erasing:
            self._apply_erase(event.xdata, event.ydata)
        self._draw_brush()

    def _apply_erase(self, x, z):
        end = (float(x), float(z))
        start = self._stroke_last if self._stroke_last is not None else end
        self.erased.update(int(v) for v in self.index.erase_stroke(start, end, self.ERASE_RADIUS_M))
        self._stroke_last = end

    def _finish_erase(self):
        if not self._erasing:
            return
        self._erasing = False
        added = self.erased - self._erase_before
        if added:
            self._record(("erase", added))
        self._erase_before = None
        self._stroke_last = None
        self._rebuild_points()
        self._update_title()

    def _on_click_erase(self, event):
        if event.button == 1:
            self._erasing = True
            self._erase_before = set(self.erased)
            self._stroke_last = None
            self._apply_erase(event.xdata, event.ydata)
            self._on_motion_erase(event)
        elif event.button == 3:
            self._finish_erase()

    def _record(self, action):
        self._history.append(action)
        self._history = self._history[-100:]
        self._redo_history.clear()

    def _apply_action(self, action, undo):
        if action[0] == "erase":
            if undo:
                self.erased.difference_update(action[1])
            else:
                self.erased.update(action[1])
            self._rebuild_points()
        else:
            kind, key, value = action
            remove = (kind == "add") == undo
            if remove:
                self.annotations[key].pop()
            else:
                self.annotations[key].append(value)
            self._redraw_existing()
        self._update_title()

    def _undo(self):
        self._finish_erase()
        if self._history:
            action = self._history.pop()
            self._apply_action(action, True)
            self._redo_history.append(action)

    def _redo(self):
        if self._redo_history:
            action = self._redo_history.pop()
            self._apply_action(action, False)
            self._history.append(action)

    # ------------------------------------------------------------------ 多边形
    def _draw_current(self):
        if self.current_line is not None:
            self.current_line.remove()
            self.current_line = None
        for lb in self.vertex_labels:
            lb.remove()
        self.vertex_labels = []
        if not self.current:
            self.fig.canvas.draw_idle()
            return
        color = self.COLORS.get(self.mode, "gray")
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
        value = [list(p) for p in self.current]
        self.annotations[self.mode].append(value)
        self._record(("add", self.mode, value))
        arr = np.asarray(self.current)
        color = self.COLORS.get(self.mode, "gray")
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
            self._record(("add", "obstacle_segments", seg))
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
            value = self.annotations[key].pop()
            self._record(("delete", key, value))
            print(f"deleted last {key} item")
            self.fig.canvas.draw_idle()

    def _save(self):
        self._finish_erase()
        self.annotations["erased_point_ids"] = sorted(int(i) for i in self.erased)
        import os, tempfile
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        fd, path = tempfile.mkstemp(prefix=self.out_path.name+".", dir=self.out_path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(self.annotations, f, ensure_ascii=False, indent=2)
            os.replace(path, self.out_path)
        finally:
            if os.path.exists(path):
                os.unlink(path)
        print(f"saved -> {self.out_path} (erased {len(self.erased)} points)")

    # ------------------------------------------------------------------ 键盘/滚轮
    def _on_key(self, event):
        # Text input owns its keys: typing '1' must not switch to Free mode.
        if getattr(self.radius_box, "capturekeystrokes", False):
            return
        k = event.key
        if k is None:
            return
        if k in self.MODE_KEYS:
            self._set_mode(self.MODE_KEYS[k])
        elif k == "ctrl+z":
            self._undo()
        elif k in ("ctrl+y", "ctrl+shift+z"):
            self._redo()
        elif k == "h":
            self._home()
        elif k == "escape":
            self._clear_current()
            self._cancel_wall_start()
        elif k == "z":
            self._undo_point()
        elif k == "x":
            self._del_poly()
        elif k == "w":
            self._save()
        elif k in ("?", "f1"):
            self._show_help()
        elif k == "[":
            self._scale_erase_radius(1/1.5)
        elif k == "]":
            self._scale_erase_radius(1.5)
        elif k == "q":
            self._save()
            import matplotlib.pyplot as plt
            plt.close(self.fig)

    def _on_close(self, event):
        self._view_timer.stop()
        self._save()
        if self._help_fig is not None:
            import matplotlib.pyplot as plt
            plt.close(self._help_fig)

    def _on_scroll(self, event):
        if event.inaxes is not self.ax or self._pan is not None:
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
    p.add_argument("--erase-radius-m", type=float, default=0.3,
                   help="初始擦除半径，单位米，范围 0.05–3.00，默认 0.30；界面可随时调整")
    p.add_argument("--display-mode", choices=("auto", "points", "density"), default="auto",
                   help="auto：概览像素聚合，放大后原始点；points：强制全点；density：始终聚合")
    args = p.parse_args(argv)
    if not np.isfinite(args.erase_radius_m) or not 0.05 <= args.erase_radius_m <= 3.0:
        p.error("--erase-radius-m 必须在 0.05–3.00 米之间")

    data = json.loads(Path(args.source_json).read_text(encoding="utf-8"))
    import map2d_data as m2d
    m2d._validate_source_json(data)
    pts = np.array([[lm["pose"]["x"], lm["pose"]["y"], lm["pose"]["z"]]
                    for lm in data["landmarks"]], dtype=np.float64)
    ids = np.array([lm["id"] for lm in data["landmarks"]], dtype=np.int64)
    candidates = None
    cand_ids = None
    if args.ground_y is not None:
        if not np.all(np.isfinite([args.ground_y, args.height_min, args.height_max])) or args.height_min > args.height_max:
            p.error("地面/高度区间非法")
        h = args.ground_y - pts[:, 1]
        mask = (h >= args.height_min) & (h <= args.height_max)
        candidates = pts[mask]
        cand_ids = ids[mask]

    annotations = {"free": [], "obstacle": [], "unknown": [], "candidate_remove": [],
                   "obstacle_segments": []}
    existing = Path(args.annotations) if args.annotations else Path(args.out)
    if existing.is_file():
        loaded = json.loads(existing.read_text(encoding="utf-8"))
        print(f"恢复标注: {existing}")
    elif args.annotations:
        p.error(f"标注文件不存在: {existing}")
    else:
        loaded = {}
    if loaded:
        for k in annotations:
            annotations[k] = list(loaded.get(k, []))
        if "erased_point_ids" in loaded:
            annotations["erased_point_ids"] = list(loaded["erased_point_ids"])

    editor = Annotator(pts, candidates, annotations, Path(args.out),
                       landmark_ids=ids, cand_ids=cand_ids, display_mode=args.display_mode,
                       erase_radius_m=args.erase_radius_m)
    import matplotlib.pyplot as plt
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
