"""Offline A* probe checks against an independent shortest-path solver."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.sparse.csgraph import floyd_warshall

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'orbbec'))
import map2d_data as m2d
from validate_map2d import astar_probe


def planning_map(trav,cost=None):
    trav=np.asarray(trav,dtype=bool)
    cost=np.where(trav,0.,np.inf) if cost is None else np.where(trav,cost,np.inf)
    return m2d.PlanningMap(Path('/tmp/test-map'),{'resolution_m':.1,'origin_xz':[0,0]},
                          np.where(trav,0,100).astype(np.int8),trav,np.ones(trav.shape),cost,False)


class ProbeTests(unittest.TestCase):
    def test_no_corner_cut_and_blocked_endpoints(self):
        pm=planning_map([[True,False],[False,True]])
        self.assertEqual(astar_probe(pm,(0,0),(1,1))['status'],'no_path')
        self.assertEqual(astar_probe(pm,(0,0),(0,1))['status'],'blocked_endpoint')
        self.assertEqual(astar_probe(pm,None,(1,1))['status'],'blocked_endpoint')
        self.assertEqual(astar_probe(pm,(-1,0),(1,1))['status'],'blocked_endpoint')

    def test_weighted_optimum_matches_floyd_warshall(self):
        rng=np.random.default_rng(24)
        for _ in range(5):
            trav=rng.random((5,6))>.2
            trav[0,0]=trav[-1,-1]=True
            pm=planning_map(trav,rng.random(trav.shape))
            h,w=trav.shape
            matrix=np.full((h*w,h*w),np.inf)
            np.fill_diagonal(matrix,0)
            # Independent graph construction and all-pairs dynamic programming.
            for r,c in np.ndindex(trav.shape):
                if not trav[r,c]: continue
                for rr,cc in np.ndindex(trav.shape):
                    dr,dc=abs(rr-r),abs(cc-c)
                    if not trav[rr,cc] or max(dr,dc)!=1: continue
                    if dr and dc and not (trav[r,cc] and trav[rr,c]): continue
                    matrix[r*w+c,rr*w+cc]=.1*math.hypot(dr,dc)*(1+2*(pm.cost[r,c]+pm.cost[rr,cc])/2)
            expected=floyd_warshall(matrix,directed=False)[0,-1]
            result=astar_probe(pm,(0,0),(h-1,w-1))
            if math.isinf(expected):
                self.assertEqual(result['status'],'no_path')
            else:
                self.assertEqual(result['status'],'ok')
                self.assertAlmostEqual(result['total_cost'],expected,places=10)
                self.assertTrue(all(pm.edge_is_free(a,b) for a,b in zip(result['path_cells'],result['path_cells'][1:])))

    def test_limits_and_same_cell(self):
        pm=planning_map(np.ones((8,8),bool))
        self.assertEqual(astar_probe(pm,(0,0),(7,7),max_expansions=1)['status'],'expansion_limit')
        result=astar_probe(pm,(2,2),(2,2))
        self.assertEqual(result['path_cells'],[(2,2)])
        self.assertEqual(result['total_cost'],0)
        with self.assertRaises(ValueError):
            astar_probe(pm,(0,0),(7,7),cost_weight=-1)


if __name__=='__main__':
    unittest.main()
