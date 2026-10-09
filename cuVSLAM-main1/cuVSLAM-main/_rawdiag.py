import sys
sys.path.insert(0, "/cuvslam/examples/orbbec")

import numpy as np
from pyorbbecsdk import Config, Pipeline, OBFormat, OBSensorType, OBFrameType

ir_pipe = Pipeline()
ir_cfg = Config()
for st in [OBSensorType.LEFT_IR_SENSOR, OBSensorType.RIGHT_IR_SENSOR]:
    pl = ir_pipe.get_stream_profile_list(st)
    prof = pl.get_video_stream_profile(848, 480, OBFormat.Y8, 60)
    ir_cfg.enable_stream(prof)
ir_pipe.start(ir_cfg)

frames = ir_pipe.wait_for_frames(2000)
if frames is None:
    print("no frames"); raise SystemExit

left = frames.get_frame(OBFrameType.LEFT_IR_FRAME)
vf = left.as_video_frame()
print("format =", vf.get_format())
print("width x height =", vf.get_width(), vf.get_height())
print("data_size =", vf.get_data_size())

data = vf.get_data()
print("type(get_data()) =", type(data))
try:
    print("len(data) =", len(data))
except TypeError as e:
    print("len error:", e)

arr = np.asanyarray(data)
print("np.asanyarray(data) shape =", arr.shape, "dtype =", arr.dtype)

# 打印原始字节
if hasattr(data, "__getitem__"):
    raw = bytes(data[:64])
    print("first 64 raw bytes =", list(raw))

# 用 np.frombuffer 直接读（假设是连续字节缓冲区）
try:
    flat = np.frombuffer(data, dtype=np.uint8)
    print("frombuffer shape =", flat.shape)
    print("frombuffer first 32 =", list(flat[:32]))
    print("frombuffer min/max/mean/std =", flat.min(), flat.max(), float(flat.mean()), float(flat.std()))
    print("frombuffer uniq =", len(np.unique(flat)))
except Exception as e:
    print("frombuffer error:", e)

ir_pipe.stop()
print("done")
