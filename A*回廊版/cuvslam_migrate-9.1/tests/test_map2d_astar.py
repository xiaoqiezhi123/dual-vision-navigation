"""Reference polyline safety, coordinate fidelity, output protocol and UI flow."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use('Agg')
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'orbbec'))
from map2d_data import PlanningMap
from map2d_astar import (PlanningError, astar_search, world_segment_is_free,
                         plan_reference_path, sample_references, export_coordinates,
                         center_preference_map, simplify_path, world_segment_cells)


def make_map(mask, origin=(-10., -20.), res=.1):
    mask = np.asarray(mask, bool)
    meta = dict(origin_xz=list(origin), resolution_m=res, robot_radius_m=0.,
                safety_margin_m=0., map_version='test', map_id='test')
    return PlanningMap(Path('/tmp/test'), meta, np.where(mask, 0, 100).astype(np.int8),
                       mask, np.ones(mask.shape), np.where(mask, 0., np.inf), True)


def rectangle_intersects_segment(a, b, lo, hi):
    """Independent slab intersection, including touching rectangle boundaries."""
    first, last = 0., 1.
    for i in range(2):
        d = b[i]-a[i]
        if abs(d) < 1e-12:
            if a[i] < lo[i]-1e-10 or a[i] > hi[i]+1e-10:
                return False
        else:
            t0, t1 = sorted(((lo[i]-a[i])/d, (hi[i]-a[i])/d))
            first, last = max(first, t0), min(last, t1)
            if first > last+1e-10:
                return False
    return True


class ReferenceTests(unittest.TestCase):
    def test_center_cost_prefers_corridor_interior_without_changing_obstacles(self):
        from map2d_data import compute_inflation
        occupancy=np.full((42,95),100,dtype=np.int8)
        occupancy[2:40,2:93]=0
        clearance,mask,cost=compute_inflation(occupancy,.1,.2,0.,.3)
        pm=make_map(mask,origin=(0.,0.),res=.1)
        pm.occupancy=occupancy;pm.clearance_m=clearance;pm.cost=cost
        pm.meta.update(robot_radius_m=.2,safety_margin_m=0.)
        old_cost=cost.copy();preferred=center_preference_map(pm,.8)
        a,b=pm.grid_to_world(10,10),pm.grid_to_world(10,84)
        old=plan_reference_path(pm,a,b,count=13)
        new=plan_reference_path(preferred,a,b,count=13,cost_weight=4.,
            simplification_tolerance_m=.1,max_clearance_loss_m=.05)
        old_mid=np.asarray(old['reference_polyline_xy'])[3:-3,1].mean()
        new_mid=np.asarray(new['reference_polyline_xy'])[3:-3,1].mean()
        self.assertGreater(new_mid,old_mid+.3)
        self.assertIs(preferred.traversable,pm.traversable)
        np.testing.assert_array_equal(cost,old_cost)
        self.assertTrue(np.isposinf(preferred.cost[~mask]).all())
        self.assertEqual(new['reference_polyline_xy'][0],list(a))
        self.assertEqual(new['reference_polyline_xy'][-1],list(b))
        self.assertEqual(len(new['reference_polyline_xy']),15)

    def test_simplification_rejects_collision_free_but_low_clearance_shortcut(self):
        pm=make_map(np.ones((12,12),bool),origin=(0.,0.),res=1.)
        pm.clearance_m[:]=2.
        pm.clearance_m[5,5]=.55
        path=np.array([[2.5,5.5],[5.5,6.5],[8.5,5.5]])
        ordinary=simplify_path(pm,path,tolerance_m=2.)
        guarded=simplify_path(pm,path,tolerance_m=2.,max_clearance_loss_m=.05)
        self.assertEqual(len(ordinary),2)
        self.assertEqual(len(guarded),3)
        self.assertTrue(world_segment_is_free(pm,*ordinary))
        self.assertTrue(all(pm.clearance_m[c]>=1.95 for a,b in zip(guarded,guarded[1:])
                            for c in world_segment_cells(pm,a,b)))

    def test_bend_retention_and_independent_collision_check(self):
        mask = np.zeros((28, 28), bool)
        mask[3:9, 3:24] = True
        mask[3:24, 18:24] = True
        pm = make_map(mask)
        a, b = pm.grid_to_world(5, 5), pm.grid_to_world(21, 21)
        self.assertFalse(world_segment_is_free(pm, a, b))
        result = plan_reference_path(pm, a, b)
        refs = result['references_xy']
        self.assertEqual(len(refs), 15)
        self.assertNotEqual(refs[0], list(a))
        self.assertNotEqual(refs[-1], list(b))
        self.assertGreater(len(result['anchors_xy']), 2)
        full = result['reference_polyline_xy']
        origin = np.array(pm.meta['origin_xz'])
        for start, end in zip(full, full[1:]):
            for row, col in np.argwhere(~mask):
                lo = origin+np.array([col, row])*.1
                self.assertFalse(rectangle_intersects_segment(start, end, lo, lo+.1))
        for anchor in result['anchors_xy'][1:-1]:
            self.assertTrue(any(np.allclose(anchor, p, atol=1e-12) for p in refs))

    def test_protocol_and_unsnapped_endpoints_and_export(self):
        pm = make_map(np.ones((30, 35), bool))
        a, b = (-9.876, -19.843), (-7.054, -17.123)
        for include in [False, True]:
            result = plan_reference_path(pm, a, b, include_goal=include)
            self.assertEqual(result['start_xy'], list(a))
            self.assertEqual(result['goal_xy'], list(b))
            refs = result['references_xy']
            self.assertEqual(len(refs), 15)
            self.assertEqual(len({tuple(p) for p in refs}), 15)
            self.assertEqual(np.allclose(refs[-1], b), include)
            self.assertEqual(result['reference_polyline_xy'][0], list(a))
            self.assertTrue(np.allclose(result['reference_polyline_xy'][-1], b))
            self.assertTrue(np.allclose(refs[0], np.array(a)+(np.array(b)-a)/(15 if include else 16)))
            with tempfile.TemporaryDirectory() as tmp:
                out = export_coordinates(result, Path(tmp)/'result')
                saved = json.loads((out/'route.json').read_text())
                self.assertEqual(saved, result)
                with (out/'reference_points.csv').open() as f:
                    rows = list(csv.reader(f))
                self.assertEqual(len(rows), 16)
                self.assertEqual([float(x) for x in rows[1][1:]], refs[0])
                with self.assertRaises(FileExistsError):
                    export_coordinates(result, out)

    def test_same_cell_and_same_point(self):
        pm = make_map(np.ones((8, 8), bool), origin=(0., 0.))
        r = plan_reference_path(pm, (.351, .354), (.374, .378))
        self.assertEqual(len({tuple(p) for p in r['references_xy']}), 15)
        self.assertEqual(len(r['anchors_xy']), 2)
        with self.assertRaisesRegex(PlanningError, '重合'):
            plan_reference_path(pm, (.35, .35), (.35, .35))

    def test_forbidden_space_boundary_and_no_path(self):
        mask = np.ones((12, 12), bool)
        mask[:, 6] = False
        pm = make_map(mask, origin=(0., 0.))
        # A line on the blocked-cell boundary must not pass by rounding to the free side.
        self.assertFalse(world_segment_is_free(pm, (.7, .2), (.7, .9)))
        self.assertFalse(world_segment_is_free(pm, (0., .2), (0., .9)))
        self.assertFalse(world_segment_is_free(pm, (np.nan, 0), (.3, .3)))
        self.assertFalse(world_segment_is_free(pm, (.3, .3), (2., .3)))
        for a, b, code in [((.35, .35), (.95, .95), 'no_path'),
                            ((.65, .35), (.95, .95), 'blocked_endpoint'),
                            ((-.1, .3), (.95, .95), 'out_of_bounds')]:
            with self.assertRaises(PlanningError) as error:
                plan_reference_path(pm, a, b)
            self.assertEqual(error.exception.code, code)
        pm.reviewed = False
        with self.assertRaises(PlanningError) as error:
            plan_reference_path(pm, (.35, .35), (.35, .95))
        self.assertEqual(error.exception.code, 'unreviewed_map')

    def test_point_budget_and_cancellation(self):
        with self.assertRaises(PlanningError) as error:
            sample_references([[0, 0], [1, 0], [1, 1], [2, 1]], count=1)
        self.assertEqual(error.exception.code, 'reference_budget')
        event = threading.Event()
        event.set()
        pm = make_map(np.ones((8, 8), bool))
        self.assertEqual(astar_search(pm, (1, 1), (6, 6), cancel=event)['status'], 'cancelled')

    def test_gui_two_clicks_export_swap_reset_and_stale_results(self):
        from plan_map2d import PlannerWindow
        import matplotlib.pyplot as plt
        pm = make_map(np.ones((16, 16), bool))
        with tempfile.TemporaryDirectory() as tmp:
            ui = PlannerWindow(pm, output_root=tmp)
            try:
                ui.select(pm.grid_to_world(2, 2))
                self.assertEqual(ui.mode, 'goal')
                self.assertIsNone(ui.result)
                ui.select(pm.grid_to_world(12, 12))
                until = time.monotonic()+10
                while ui.result is None and time.monotonic() < until:
                    time.sleep(.02)
                    ui.poll()
                self.assertIsNotNone(ui.result)
                self.assertEqual(len(ui.result['references_xy']), 15)
                self.assertTrue((ui.saved_dir/'route.json').is_file())
                old_result = ui.result
                # Swapping schedules a new plan and invalidates the displayed route.
                with patch.object(ui, 'request_plan') as request:
                    ui.swap()
                    request.assert_called_once()
                self.assertEqual(list(ui.start), old_result['goal_xy'])
                self.assertEqual(list(ui.goal), old_result['start_xy'])
                generation = ui.generation
                ui.reset()
                ui.mailbox.put((generation, old_result))
                ui.poll()
                self.assertIsNone(ui.result)
                self.assertIsNone(ui.start)
                self.assertIsNone(ui.goal)
                self.assertIsNone(ui.saved_dir)
            finally:
                ui.on_close()
                plt.close(ui.fig)


if __name__ == '__main__':
    unittest.main()
