"""Headless editor regression checks; no camera, GPU or desktop required."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'orbbec'))
from map2d_display import PointCloudIndex
from annotate_map2d import Annotator


class EditorTests(unittest.TestCase):
    def test_capsule_erase_and_pixel_projection(self):
        pts = np.array([[0,0,0],[1,0,0],[2,0,0],[1,0,.5]])
        index = PointCloudIndex(pts, [10,11,12,13], [12])
        self.assertEqual(set(index.erase_stroke((0,0),(2,0),.1)), {10,11,12})
        index.set_erased({11})
        visible = index.visible((0,2),(0,1))
        self.assertEqual(set(index.ids[visible]),{10,12,13})
        rgb = index.raster(visible,(0,2),(0,1),10,10)
        self.assertEqual(tuple(rgb[0,9]),(215,45,35))
        self.assertEqual(tuple(rgb[0,0]),(90,90,90))

    def test_edit_erase_undo_reload_and_view_preservation(self):
        import matplotlib.pyplot as plt
        pts = np.array([[0,0,0],[1,0,1],[2,0,2],[3,0,3]], dtype=float)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'annotations.json'
            editor = Annotator(pts, None, {'erased_point_ids':[13]}, out,
                               landmark_ids=np.array([10,11,12,13]), backend='Agg')
            editor.current = [[.2,.2],[2.8,.2],[2.8,2.8],[.2,2.8]]
            editor._draw_current()  # Previously failed with dict(MODES).
            editor._close_current()
            self.assertEqual(len(editor.annotations['free']),1)
            editor._undo(); self.assertEqual(editor.annotations['free'],[])
            editor._redo(); self.assertEqual(len(editor.annotations['free']),1)
            editor._set_mode('erase')
            editor.ax.set_xlim(.5,2.5); editor.ax.set_ylim(.5,2.5)
            editor._flush_view(); editor.fig.canvas.draw()
            click = SimpleNamespace(button=1,inaxes=editor.ax,xdata=1.,ydata=1.)
            editor._on_click_erase(click)
            editor._on_motion_erase(SimpleNamespace(inaxes=editor.ax,xdata=2.,ydata=2.))
            editor._on_release(SimpleNamespace(button=1))
            self.assertEqual(editor.erased,{11,12,13})
            np.testing.assert_allclose(editor.ax.get_xlim(),(.5,2.5))
            np.testing.assert_allclose(editor.ax.get_ylim(),(.5,2.5))
            editor._undo(); self.assertEqual(editor.erased,{13})
            editor._redo(); self.assertEqual(editor.erased,{11,12,13})
            editor._save()
            loaded = json.loads(out.read_text())
            editor2 = Annotator(pts,None,loaded,out,landmark_ids=np.array([10,11,12,13]),backend='Agg')
            self.assertEqual(editor2.erased,{11,12,13})
            self.assertEqual(len(editor2.poly_artists),1)
            np.testing.assert_array_equal(editor2.pts,pts)
            plt.close(editor.fig); plt.close(editor2.fig)

    def test_overview_switches_to_exact_points_on_zoom(self):
        import matplotlib.pyplot as plt
        rng = np.random.default_rng(7)
        pts = rng.uniform(0,100,(20000,3))
        with tempfile.TemporaryDirectory() as tmp:
            editor = Annotator(pts,None,{},Path(tmp)/'ann.json',backend='Agg')
            self.assertTrue(editor.display_aggregated)
            editor.ax.set_xlim(0,2); editor.ax.set_ylim(0,2); editor._flush_view()
            self.assertFalse(editor.display_aggregated)
            idx = editor.index.visible((0,2),(0,2))
            np.testing.assert_allclose(editor.sc_pts.get_offsets(),pts[idx][:,[0,2]])
            plt.close(editor.fig)

    def test_drag_preview_commits_exact_limits_on_release(self):
        import matplotlib.pyplot as plt
        pts = np.array([[0,0,0],[10,0,10]],dtype=float)
        with tempfile.TemporaryDirectory() as tmp:
            editor = Annotator(pts,None,{},Path(tmp)/'ann.json',backend='Agg')
            editor.fig.canvas.draw()
            xl, zl = editor.ax.get_xlim(), editor.ax.get_ylim()
            pixel = editor.ax.transData.transform((5,5))
            inv = editor.ax.transData.inverted().frozen()
            event = SimpleNamespace(button=2,inaxes=editor.ax,xdata=5.,ydata=5.,x=pixel[0],y=pixel[1])
            editor._on_click(event)
            event.x += 50; event.y -= 30
            editor._on_motion_pan(event)
            delta = inv.transform((event.x,event.y))-inv.transform(pixel)
            editor._on_release(event)
            np.testing.assert_allclose(editor.ax.get_xlim(),np.array(xl)-delta[0])
            np.testing.assert_allclose(editor.ax.get_ylim(),np.array(zl)-delta[1])
            np.testing.assert_array_equal(editor.pts,pts)
            plt.close(editor.fig)

    def test_radius_controls_change_actual_erasure_and_ignore_invalid_input(self):
        import matplotlib.pyplot as plt
        pts = np.array([[0,0,0],[.2,0,0],[.6,0,0],[2,0,2]],dtype=float)
        with tempfile.TemporaryDirectory() as tmp:
            editor = Annotator(pts,None,{},Path(tmp)/'ann.json',backend='Agg',erase_radius_m=.2)
            editor._set_mode('erase')
            event = SimpleNamespace(button=1,inaxes=editor.ax,xdata=0.,ydata=0.)
            editor._on_motion_erase(event)
            editor.radius_box.set_val('0.75')
            self.assertAlmostEqual(editor.brush.get_radius(), .75)
            self.assertAlmostEqual(editor.radius_slider.val, .75)
            editor._on_click_erase(event); editor._on_release(event)
            self.assertEqual(editor.erased,{0,1,2})
            editor._undo()
            editor.radius_slider.set_val(.25)
            self.assertEqual(editor.radius_box.text,'0.25')
            editor._on_click_erase(event); editor._on_release(event)
            self.assertEqual(editor.erased,{0,1})
            for bad in ('nan','inf','-1','0','10','abc'):
                editor.radius_box.set_val(bad)
                self.assertEqual(editor.ERASE_RADIUS_M,.25)
                self.assertEqual(editor.radius_box.text,'0.25')
            editor._on_key(SimpleNamespace(key=']'))
            self.assertAlmostEqual(editor.brush.get_radius(),editor.ERASE_RADIUS_M)
            self.assertAlmostEqual(editor.radius_slider.val,editor.ERASE_RADIUS_M)
            editor._scale_erase_radius(100)
            self.assertEqual(editor.ERASE_RADIUS_M,3.)
            editor._scale_erase_radius(.0001)
            self.assertEqual(editor.ERASE_RADIUS_M,.05)
            editor.radius_box.begin_typing()
            editor._on_key(SimpleNamespace(key='1'))
            self.assertEqual(editor.mode,'erase')
            editor.radius_box.stop_typing()
            plt.close(editor.fig)

    def test_button_help_and_hover_do_not_modify_annotations(self):
        import copy
        import matplotlib.pyplot as plt
        with tempfile.TemporaryDirectory() as tmp:
            editor = Annotator(np.array([[0,0,0],[1,0,1.]]),None,{},Path(tmp)/'ann.json',backend='Agg')
            before = copy.deepcopy(editor.annotations)
            for key in [m[1] for m in editor.MODES]+['UndoPt','DelPoly','Undo','Redo','Home','Save','Help']:
                self.assertIn(key,editor.HELP_ZH)
                editor._on_control_hover(SimpleNamespace(inaxes=editor.buttons[key].ax))
                self.assertTrue(editor.help_text.get_text())
            editor._show_help()
            self.assertTrue(plt.fignum_exists(editor._help_fig.number))
            self.assertEqual(editor.annotations,before)
            plt.close(editor._help_fig)
            plt.close(editor.fig)


if __name__ == '__main__':
    unittest.main()
