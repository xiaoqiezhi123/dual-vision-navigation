import sys
sys.path.insert(0, "/cuvslam/examples/orbbec")

import numpy as np
from pyorbbecsdk import Config, Pipeline, OBFormat, OBSensorType, OBFrameType
from camera_utils import process_ir_frame

ir_pipe = Pipeline()
ir_cfg = Config()
for st in [OBSensorType.LEFT_IR_SENSOR, OBSensorType.RIGHT_IR_SENSOR]:
    pl = ir_pipe.get_stream_profile_list(st)
    prof = pl.get_video_stream_profile(848, 480, OBFormat.Y8, 60)
    ir_cfg.enable_stream(prof)
ir_pipe.start(ir_cfg)

print("=== IR 图像像素统计（连续 5 帧） ===")
for i in range(5):
    frames = ir_pipe.wait_for_frames(500)
    if frames is None:
        print(f"[{i}] no frames")
        continue
    left = frames.get_frame(OBFrameType.LEFT_IR_FRAME)
    right = frames.get_frame(OBFrameType.RIGHT_IR_FRAME)
    if left is None or right is None:
        print(f"[{i}] missing left/right")
        continue
    li = process_ir_frame(left)
    ri = process_ir_frame(right)
    if li is None or ri is None:
        print(f"[{i}] process failed")
        continue
    print(f"[{i}] left  shape={li.shape} dtype={li.dtype} min={li.min()} max={li.max()} mean={li.mean():.2f} std={li.std():.2f} uniq={len(np.unique(li))}")
    print(f"[{i}] right shape={ri.shape} dtype={ri.dtype} min={ri.min()} max={ri.max()} mean={ri.mean():.2f} std={ri.std():.2f} uniq={len(np.unique(ri))}")

ir_pipe.stop()
print("done")
