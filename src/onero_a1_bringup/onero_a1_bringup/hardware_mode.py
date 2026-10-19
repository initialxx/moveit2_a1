"""Resolve the public mode before launch defaults or hardware actions run."""

MODE_FLAGS = {
    "fake": (True, True),
    "sim": (False, True),
    "real": (False, False),
}

# Gains are unused by the fake/sim plants. Defaults can be overridden explicitly;
# mounting and payload remain physical properties supplied by the user.
MODE_DEFAULTS = {
    "fake": {"kp": "50.0", "control_rate": "200.0", "gravity_compensation": "false",
             "velocity_feedforward": "false"},
    "sim": {"kp": "50.0", "control_rate": "200.0", "gravity_compensation": "false",
            "velocity_feedforward": "false"},
    "real": {"kp": "50.0", "control_rate": "100.0", "gravity_compensation": "true",
             "velocity_feedforward": "true"},
}


def mode_default(name):
    """Lazy launch default, evaluated after hardware_mode has been resolved.

    DeclareLaunchArgument preserves explicit user values and values forwarded
    by the parent launch. Both entry points share this same profile table.
    """
    from launch.substitutions import LaunchConfiguration, PythonExpression

    values = {mode: defaults[name] for mode, defaults in MODE_DEFAULTS.items()}
    return PythonExpression([repr(values), "['", LaunchConfiguration("hardware_mode"), "']"])


def _boolean(value, name):
    normalized = str(value).strip().lower()
    if normalized not in ("true", "false", "1", "0"):
        raise ValueError(f"{name} 必须为 true 或 false")
    return normalized in ("true", "1")


def resolve_hardware_settings(mode=None, use_fake_hardware=None, simulate=None):
    legacy = (use_fake_hardware, simulate)
    names = ("use_fake_hardware", "simulate")
    if mode is None:
        fake, simulated = (True if value is None else _boolean(value, name)
                           for value, name in zip(legacy, names))
        mode = "fake" if fake else "sim" if simulated else "real"
    else:
        mode = str(mode).strip().lower()
        if mode not in MODE_FLAGS:
            raise ValueError("hardware_mode 必须为 fake、sim 或 real")
        for value, name, expected in zip(legacy, names, MODE_FLAGS[mode]):
            if value is not None and _boolean(value, name) != expected:
                raise ValueError(f"hardware_mode={mode} 与 {name}={value} 冲突；请只使用 hardware_mode 选择模式")
    fake, simulated = MODE_FLAGS[mode]
    return {"hardware_mode": mode,
            "use_fake_hardware": str(fake).lower(),
            "simulate": str(simulated).lower()}


def configure_hardware_mode(context):
    from launch.actions import SetLaunchConfiguration

    # Must precede DeclareLaunchArgument: only explicitly supplied values are
    # present here. This preserves old commands without letting their defaults
    # conflict with hardware_mode:=real or mask an explicit fake selection.
    values = context.launch_configurations
    settings = resolve_hardware_settings(
        values.get("hardware_mode"), values.get("use_fake_hardware"), values.get("simulate"))
    return [SetLaunchConfiguration(key, value) for key, value in settings.items()]
