from pyorbbecsdk import Config, Pipeline, OBPropertyID, OBPermissionType

pipe = Pipeline()
dev = pipe.get_device()
info = dev.get_device_info()
print("device =", info.get_name(), "SN:", info.get_serial_number())

props = [
    ("LASER_BOOL", OBPropertyID.OB_PROP_LASER_BOOL),
    ("LASER_CONTROL_INT", OBPropertyID.OB_PROP_LASER_CONTROL_INT),
    ("LASER_ALWAYS_ON_BOOL", OBPropertyID.OB_PROP_LASER_ALWAYS_ON_BOOL),
    ("LASER_MODE_INT", OBPropertyID.OB_PROP_LASER_MODE_INT),
    ("LASER_ON_OFF_PATTERN_INT", OBPropertyID.OB_PROP_LASER_ON_OFF_PATTERN_INT),
    ("LASER_POWER_LEVEL_CONTROL_INT", OBPropertyID.OB_PROP_LASER_POWER_LEVEL_CONTROL_INT),
    ("LASER_ENERGY_LEVEL_INT", OBPropertyID.OB_PROP_LASER_ENERGY_LEVEL_INT),
    ("LDP_BOOL", OBPropertyID.OB_PROP_LDP_BOOL),
    ("SWITCH_IR_MODE_INT", OBPropertyID.OB_PROP_SWITCH_IR_MODE_INT),
    ("LOW_EXPOSURE_LASER_CONTROL_BOOL", OBPropertyID.OB_PROP_LOW_EXPOSURE_LASER_CONTROL_BOOL),
    ("IR_AUTO_EXPOSURE_BOOL", OBPropertyID.OB_PROP_IR_AUTO_EXPOSURE_BOOL),
    ("IR_EXPOSURE_INT", OBPropertyID.OB_PROP_IR_EXPOSURE_INT),
    ("IR_GAIN_INT", OBPropertyID.OB_PROP_IR_GAIN_INT),
    ("IR_BRIGHTNESS_INT", OBPropertyID.OB_PROP_IR_BRIGHTNESS_INT),
    ("IR_AE_MAX_EXPOSURE_INT", OBPropertyID.OB_PROP_IR_AE_MAX_EXPOSURE_INT),
]

for name, pid in props:
    try:
        for perm in (OBPermissionType.PERMISSION_READ_WRITE, OBPermissionType.PERMISSION_READ):
            if dev.is_property_supported(pid, perm):
                break
        else:
            print(f"{name}: NOT supported")
            continue
        try:
            rng = dev.get_int_property_range(pid)
            cur = dev.get_int_property(pid)
            print(f"{name}: current={cur} range=[{rng.min},{rng.max}] step={rng.step} def={rng.default}")
        except Exception:
            try:
                cur = dev.get_int_property(pid)
                print(f"{name}: current={cur} (no range)")
            except Exception:
                try:
                    cur = dev.get_bool_property(pid)
                    print(f"{name}: bool current={cur}")
                except Exception as e3:
                    print(f"{name}: read error {e3}")
    except Exception as e:
        print(f"{name}: {e}")
