"""Headless editor regression checks; no camera, GPU or desktop required."""
import json
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'orbbec'))
from map2d_display import PointCloudIndex
from annotate_map2d import Annotator
import annotate_map2d


class EditorTests(unittest.TestCase):
    def test_selected_middle_obstacle_and_wall_delete_undo_redo_and_reload(self):
        import copy
        import matplotlib.pyplot as plt
        ann = {
            'free': [[[0,0],[12,0],[12,12],[0,12]]],
            'obstacle': [[[1,1],[2,1],[2,2],[1,2]],
                         [[3,3],[5,3],[5,5],[3,5]],
                         [[8,8],[9,8],[9,9],[8,9]]],
            'obstacle_segments': [[[1,6],[2,6]], [[3,6],[5,6]], [[7,6],[9,6]]],
        }
        pts = np.array([[0.,0,0],[12,0,12]])
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'annotations.json'
            out.write_text(json.dumps(ann))
            editor = Annotator(pts,None,json.loads(out.read_text()),out,backend='Agg')
            editor._set_mode('select')
            def click(x,z):
                editor._on_click(SimpleNamespace(button=1,inaxes=editor.ax,xdata=x,ydata=z))
            click(4,4)
            self.assertEqual(editor.selected,('obstacle',1))
            self.assertIsNotNone(editor.selection_artist)
            editor._on_key(SimpleNamespace(key='delete'))
            self.assertEqual(editor.annotations['obstacle'],[ann['obstacle'][0],ann['obstacle'][2]])
            self.assertIsNone(editor.selected)
            editor._on_key(SimpleNamespace(key='ctrl+z'))
            self.assertEqual(editor.annotations['obstacle'],ann['obstacle'])
            editor._on_key(SimpleNamespace(key='ctrl+y'))
            self.assertEqual(len(editor.annotations['obstacle']),2)
            editor._undo()
            # The free polygon also contains this point, but the wall must win.
            click(4,6)
            self.assertEqual(editor.selected,('obstacle_segments',1))
            editor._delete_selected()
            self.assertEqual(editor.annotations['obstacle_segments'],[ann['obstacle_segments'][0],ann['obstacle_segments'][2]])
            editor._undo()
            self.assertEqual(editor.annotations['obstacle_segments'],ann['obstacle_segments'])
            editor._redo()
            editor._save()
            saved = json.loads(out.read_text())
            self.assertEqual(saved['obstacle'],ann['obstacle'])
            self.assertEqual(saved['free'],ann['free'])
            self.assertEqual(saved['obstacle_segments'],[ann['obstacle_segments'][0],ann['obstacle_segments'][2]])
            loaded = Annotator(pts,None,saved,out,backend='Agg')
            loaded._set_mode('select')
            loaded._on_click(SimpleNamespace(button=1,inaxes=loaded.ax,xdata=8,ydata=6))
            self.assertEqual(loaded.selected,('obstacle_segments',1))
            loaded._delete_selected(); loaded._undo()
            self.assertEqual(loaded.annotations['obstacle_segments'],[ann['obstacle_segments'][0],ann['obstacle_segments'][2]])
            # Escape clears selection; pressing Delete afterward is a no-op.
            click(4,4)
            before = copy.deepcopy(editor.annotations)
            editor._on_key(SimpleNamespace(key='escape'))
            editor._on_key(SimpleNamespace(key='delete'))
            self.assertEqual(editor.annotations,before)
            np.testing.assert_array_equal(editor.pts,pts)
            plt.close(editor.fig); plt.close(loaded.fig)

    def test_wall_start_and_completed_shapes_undo_and_type_specific_delete(self):
        import matplotlib.pyplot as plt
        with tempfile.TemporaryDirectory() as tmp:
            editor = Annotator(np.array([[0.,0,0],[10,0,10]]),None,{},Path(tmp)/'ann.json',backend='Agg')
            editor._set_mode('obstacle')
            editor.current = [[1,1],[3,1],[3,3],[1,3]]
            editor._draw_current()
            editor._on_key(SimpleNamespace(key='backspace'))
            self.assertEqual(len(editor.current),3)
            editor._close_current()
            obstacle = editor.annotations['obstacle'][0]
            editor._undo(); self.assertEqual(editor.annotations['obstacle'],[])
            editor._redo(); self.assertEqual(editor.annotations['obstacle'],[obstacle])
            editor._set_mode('wallseg')
            def click(x,z):
                editor._on_click(SimpleNamespace(button=1,inaxes=editor.ax,xdata=x,ydata=z))
            click(2,5)
            editor._on_key(SimpleNamespace(key='z'))
            self.assertIsNone(editor.wall_start)
            self.assertIsNone(editor.wall_start_marker)
            self.assertEqual(editor.annotations['obstacle'],[obstacle])
            click(2,5); click(4,5)
            self.assertEqual(len(editor.annotations['obstacle_segments']),1)
            editor._undo(); self.assertEqual(editor.annotations['obstacle_segments'],[])
            editor._redo(); self.assertEqual(len(editor.annotations['obstacle_segments']),1)
            editor._set_mode('obstacle')
            editor._del_poly()  # Must delete the obstacle, even though the wall was added last.
            self.assertEqual(editor.annotations['obstacle'],[])
            self.assertEqual(len(editor.annotations['obstacle_segments']),1)
            editor._undo(); self.assertEqual(editor.annotations['obstacle'],[obstacle])
            editor._set_mode('wallseg'); editor._del_poly()
            self.assertEqual(editor.annotations['obstacle_segments'],[])
            editor._undo(); self.assertEqual(len(editor.annotations['obstacle_segments']),1)
            # A new deletion branches history and must invalidate previous redo.
            editor._undo()
            editor._set_mode('obstacle'); editor._del_poly()
            self.assertEqual(editor._redo_history,[])
            plt.close(editor.fig)

    def test_selection_cycles_overlaps_and_uses_pixel_tolerance_at_zoom(self):
        import matplotlib.pyplot as plt
        ann = {'obstacle': [[[1,1],[4,1],[4,4],[1,4]]]*2,
               'obstacle_segments': [[[1,6],[4,6]]]}
        with tempfile.TemporaryDirectory() as tmp:
            editor = Annotator(np.array([[0.,0,0],[10,0,10]]),None,ann,Path(tmp)/'ann.json',backend='Agg')
            editor._set_mode('select')
            event = SimpleNamespace(button=1,inaxes=editor.ax,xdata=2.,ydata=2.)
            editor._on_click(event); self.assertEqual(editor.selected,('obstacle',1))
            editor._on_click(event); self.assertEqual(editor.selected,('obstacle',0))
            editor._on_click(event); self.assertEqual(editor.selected,('obstacle',1))
            for limits in ((0,10), (1.5,2.5)):
                editor.ax.set_xlim(*limits); editor.ax.set_ylim(5,7); editor._flush_view()
                pixel = editor.ax.transData.transform((2,6)) + [0,6]
                x,z = editor.ax.transData.inverted().transform(pixel)
                editor._on_click(SimpleNamespace(button=1,inaxes=editor.ax,xdata=x,ydata=z))
                self.assertEqual(editor.selected,('obstacle_segments',0))
            editor.radius_box.begin_typing()
            editor._on_key(SimpleNamespace(key='delete'))
            self.assertEqual(len(editor.annotations['obstacle_segments']),1)
            editor.radius_box.stop_typing()
            editor._on_click(SimpleNamespace(button=3,inaxes=editor.ax,xdata=2.,ydata=6.))
            self.assertIsNone(editor.selected)
            plt.close(editor.fig)

    def test_height_slice_preserves_hidden_points_even_at_same_xz(self):
        pts = np.array([[0,1,0], [0,.875,0], [0,1.125,0], [0,0,0], [2,1,2]], float)
        original = pts.copy()
        index = PointCloudIndex(pts, [10,11,12,13,14], [11,13])
        index.set_height_filter(1, -.125, .125)
        index.set_erased({14})
        visible = index.visible((-1,3),(-1,3))
        self.assertEqual(set(index.ids[visible]), {10,11,12})
        self.assertEqual(set(index.erase_stroke((0,0),(2,2),.05)), {10,11,12})
        mask = index.display_mask.copy()
        for ground, low, high in [(float('nan'),0,1), (1,1,0), (1,0,float('inf'))]:
            with self.assertRaises(ValueError):
                index.set_height_filter(ground,low,high)
            np.testing.assert_array_equal(index.display_mask,mask)
        index.set_height_filter()
        self.assertEqual(set(index.erase_stroke((0,0),(2,2),.05)), {10,11,12,13})
        np.testing.assert_array_equal(pts,original)

    def test_height_controls_render_erase_save_and_restore_without_moving_points(self):
        import matplotlib.pyplot as plt
        pts = np.array([[0,1,0],[1,.875,1],[2,1.125,2],[0,0,0],[3,1,3]],float)
        annotations = {'obstacle_segments': [[[10.,10.],[11.,10.]]]}
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'ann.json'
            editor = Annotator(pts,pts[[0,3]],annotations,out,landmark_ids=np.arange(10,15),
                               cand_ids=[10,13],backend='Agg',display_ground_y=1,
                               display_height_min=-.125,display_height_max=.125)
            self.assertEqual(len(editor.sc_pts.get_offsets()),4)
            self.assertEqual(len(editor.sc_cand.get_offsets()),1)
            editor.candidate_check.set_active(0)
            self.assertEqual(len(editor.sc_cand.get_offsets()),0)
            editor.display_mode = 'density'
            editor._flush_view()
            rgb = np.asarray(editor.point_image.get_array())
            self.assertFalse(np.any(np.all(rgb == [215,45,35],axis=-1)))
            editor.candidate_check.set_active(0)
            rgb = np.asarray(editor.point_image.get_array())
            self.assertTrue(np.any(np.all(rgb == [215,45,35],axis=-1)))
            editor._set_mode('erase')
            event = SimpleNamespace(button=1,inaxes=editor.ax,xdata=0.,ydata=0.)
            editor._on_click_erase(event); editor._on_release(event)
            self.assertEqual(editor.erased,{10})  # ID 13 at the same XZ is hidden, not erased.
            editor._save()
            saved = json.loads(out.read_text())
            self.assertEqual(saved['erased_point_ids'],[10])
            self.assertEqual(saved['obstacle_segments'],annotations['obstacle_segments'])
            self.assertNotIn('display_ground_y',saved)
            editor.height_check.set_active(0)
            self.assertFalse(editor.height_filter_enabled)
            editor.height_check.set_active(0)
            self.assertTrue(editor.height_filter_enabled)
            editor.height_boxes['ground'].set_val('nan')
            self.assertEqual(editor.display_ground_y,1)
            self.assertTrue(editor.height_check.get_status()[0])
            editor.ax.set_xlim(-1,4); editor.ax.set_ylim(-1,4)
            editor.height_boxes['ground'].set_val('0')
            self.assertEqual(editor.display_ground_y,0)
            np.testing.assert_allclose(editor.ax.get_xlim(),(-1,4))
            self.assertEqual(set(editor.index.ids[editor.index.visible((-1,4),(-1,4))]),{13})
            editor.height_boxes['ground'].begin_typing()
            editor._on_key(SimpleNamespace(key='1'))
            self.assertEqual(editor.mode,'erase')
            editor.height_boxes['ground'].stop_typing()
            editor._home()
            self.assertGreater(editor.ax.get_xlim()[1],11)  # Keep existing walls in view.
            editor.height_boxes['ground'].set_val('100')
            self.assertFalse(np.any(editor.index.display_mask))
            editor._home()  # Empty slice must still support editing and navigation.
            editor._undo()
            self.assertEqual(editor.erased,set())
            np.testing.assert_array_equal(editor.pts,pts)
            plt.close(editor.fig)

    def test_cli_display_filter_is_independent_of_candidate_policy(self):
        import contextlib
        import io
        for extra in (['--display-height-min','0'], ['--display-ground-y','nan'],
                      ['--display-ground-y','0','--display-height-min','1','--display-height-max','0']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
                annotate_map2d.main(['--source-json','not-read.json',*extra])
            self.assertEqual(caught.exception.code,2)
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp)/'source.json'
            src.write_text(json.dumps({'landmarks': [
                {'id':10,'pose':{'x':0,'y':1,'z':0}},
                {'id':11,'pose':{'x':1,'y':-1,'z':1}}], 'poses':[], 'edges':[]}))
            with mock.patch.object(annotate_map2d,'Annotator') as ctor, mock.patch('matplotlib.pyplot.show'):
                annotate_map2d.main(['--source-json',str(src),'--out',str(Path(tmp)/'ann.json'),
                                    '--ground-y','0','--height-min','.5','--height-max','5',
                                    '--display-ground-y','1','--hide-candidates'])
            args, kwargs = ctor.call_args
            self.assertEqual(len(args[0]),2)  # Source is never prefiltered or renumbered.
            np.testing.assert_array_equal(kwargs['cand_ids'],[11])
            self.assertEqual(kwargs['display_ground_y'],1)
            self.assertEqual(kwargs['display_height_min'],-.15)
            self.assertFalse(kwargs['show_candidates'])

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
        pts[:,1] = np.where(np.arange(len(pts)) % 3, 1., 0.)
        with tempfile.TemporaryDirectory() as tmp:
            ids = np.arange(len(pts))
            editor = Annotator(pts,pts[::4],{},Path(tmp)/'ann.json',backend='Agg',
                               cand_ids=ids[::4],display_ground_y=1.)
            self.assertTrue(editor.display_aggregated)
            idx = editor.index.visible(editor.ax.get_xlim(),editor.ax.get_ylim())
            self.assertEqual(len(idx),13333)
            self.assertGreater(len(editor.pixel_cand.get_xdata()),0)
            editor.candidate_check.set_active(0)
            self.assertEqual(len(editor.pixel_cand.get_xdata()),0)
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
            for key in [m[1] for m in editor.MODES]+['UndoPt','DeleteSel','DelPoly','Undo','Redo','Home','Save','Help']:
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
