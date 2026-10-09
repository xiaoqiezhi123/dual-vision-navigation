import sys
sys.path.insert(0, "/cuvslam/examples/orbbec")
import time
import numpy as np
from pyorbbecsdk import Config, Pipeline, OBFormat, OBSensorType, OBFrameType, OBPropertyID, OBPermissionType
from camera_utils import process_ir_frame


def stats(li):
    return f"min={li.min()} max={li.max()} mean={li.mean():.1f} std={li.std():.2f} uniq={len(np.unique(li))}"


def grab(ir_pipe):
    frames = ir_pipe.wait_for_frames(500)
    if frames is None:
        return None
    left = frames.get_frame(OBFrameType.LEFT_IR_FRAME)
    if left is None:
        return None
    return process_ir_frame(left)


def set_prop(dev, pid, val):
    try:
        dev.set_int_property(pid, val)
        return dev.get_int_property(pid)
    except Exception as e:
        return f"ERR {e}"


def set_bool(dev, pid, val):
    try:
        dev.set_bool_property(pid, val)
        return dev.get_bool_property(pid)
    except Exception as e:
        return f"ERR {e}"


ir_pipe = Pipeline()
ir_cfg = Config()
for st in [OBSensorType.LEFT_IR_SENSOR, OBSensorType.RIGHT_IR_SENSOR]:
    pl = ir_pipe.get_stream_profile_list(st)
    prof = pl.get_video_stream_profile(848, 480, OBFormat.Y8, 60)
    ir_cfg.enable_stream(prof)
ir_pipe.start(ir_cfg)
dev = ir_pipe.get_device()

print("baseline (laser off):", stats(grab(ir_pipe)))

print("=== 尝试强制开启激光 ===")
print("LDP_BOOL=True ->", set_bool(dev, OBPropertyID.OB_PROP_LDP_BOOL, True))
print("LASER_ALWAYS_ON_BOOL=True ->", set_bool(dev, OBPropertyID.OB_PROP_LASER_ALWAYS_ON_BOOL, True))
print("LASER_CONTROL_INT=1 ->", set_prop(dev, OBPropertyID.OB_PROP_LASER_CONTROL_INT, 1))
time.sleep(2.0)
print("after laser-on:", stats(grab(ir_pipe)))

# 也尝试把 IR 曝光/增益手动拉高
print("=== 手动调 IR 曝光/增益 ===")
print("IR_AUTO_EXPOSURE_BOOL=False ->", set_bool(dev, OBPropertyID.OB_PROP_IR_AUTO_EXPOSURE_BOOL, False))
print("IR_EXPOSURE_INT=8000 ->", set_prop(dev, OBPropertyID.OB_PROP_IR_EXPOSURE_INT, 8000))
print("IR_GAIN_INT=64 ->", set_prop(dev, OBPropertyID.OB_PROP_IR_GAIN_INT, 64))
time.sleep(1.0)
print("after exposure/gain:", stats(grab(ir_pipe)))

ir_pipe.stop()
print("done")
