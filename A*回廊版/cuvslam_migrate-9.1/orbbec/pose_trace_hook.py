"""Optional shared diagnostic module supplied by the prepare script."""
import importlib.util
import os

class NullTrace:
    enabled = False
    stream_id = ''
    def emit(self, stage, **fields):
        pass
    def close(self):
        pass
    def encode(self, data, **context):
        return data, {}

_trace = None
def get_trace():
    global _trace
    if _trace is None:
        _trace = NullTrace()
        if os.environ.get('NAV_POSE_DIAG_DIR'):
            try:
                spec = importlib.util.spec_from_file_location('_nav_pose_trace', os.environ['NAV_POSE_DIAG_MODULE'])
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                _trace = module.get_trace('slam')
            except Exception as exc:
                print(f'[POSE-DIAG] diagnostic hook unavailable: {exc}', flush=True)
    return _trace
