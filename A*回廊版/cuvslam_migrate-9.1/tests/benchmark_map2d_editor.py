"""Measure CPU-side rendering; this is not a desktop FPS measurement."""
import argparse
import json
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'orbbec'))
from annotate_map2d import Annotator
import map2d_data as m2d


def measure(fn, iterations=30):
    fn()
    samples = []
    for _ in range(iterations):
        t0 = time.perf_counter()
        fn()
        samples.append((time.perf_counter()-t0)*1000)
    return {'median_ms': float(np.median(samples)), 'p95_ms': float(np.percentile(samples,95))}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-json',required=True)
    p.add_argument('--ground-y',type=float)
    p.add_argument('--height-min',type=float,default=.5)
    p.add_argument('--height-max',type=float,default=5.)
    args = p.parse_args()
    source = json.loads(Path(args.source_json).read_text())
    m2d._validate_source_json(source)
    pts = m2d.landmark_xyz(source)
    ids = np.array([lm['id'] for lm in source['landmarks']])
    cand = np.zeros(len(pts),bool) if args.ground_y is None else (
        (args.ground_y-pts[:,1] >= args.height_min) & (args.ground_y-pts[:,1] <= args.height_max))
    import matplotlib.pyplot as plt
    with tempfile.TemporaryDirectory() as tmp:
        editor = Annotator(pts,pts[cand],{},Path(tmp)/'ann.json',ids,ids[cand],backend='Agg')
        editor.fig.canvas.draw()
        result = {'points':len(pts), 'backend':'Agg (no desktop window)',
                  'view_refresh':measure(editor._refresh_view)}
        pixel = editor.ax.transData.transform(np.mean(editor.index.xz,axis=0))
        x,z = editor.ax.transData.inverted().transform(pixel)
        event = SimpleNamespace(button=2,inaxes=editor.ax,xdata=x,ydata=z,x=pixel[0],y=pixel[1])
        editor._on_click(event)
        event.x += 30; event.y += 15
        result['pan_preview'] = measure(lambda: editor._on_motion_pan(event),100)
        editor._on_release(event)
        editor._set_mode('erase'); editor.fig.canvas.draw()
        result['brush_refresh'] = measure(lambda: editor._on_motion_erase(event),100)
        result['erase_query'] = measure(lambda: editor.index.erase_stroke((x,z),(x+.2,z+.2),.3),100)
        print(json.dumps(result,indent=2))
        plt.close(editor.fig)


if __name__ == '__main__':
    main()
