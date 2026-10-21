"""Resolve the public mode before launch defaults or hardware actions run."""

# Gains are unused by the simulated plant. Defaults can be overridden explicitly;
# mounting and payload remain physical properties supplied by the user.
MODE_DEFAULTS = {
    "sim": {"kp": "50.0", "control_rate": "200.0", "gravity_compensation": "false",
            "velocity_feedforward": "false"},
    "real": {"kp": "50.0", "control_rate": "100.0", "gravity_compensation": "true",
             "velocity_feedforward": "true"},
}

ARM_DEFAULTS = {'right': {'model': 'A1R', 'mounting': 'humanoid_right', 'can_channel': 'can0'},
                'left': {'model': 'A1L', 'mounting': 'humanoid_left', 'can_channel': 'can1'}}


def arm_default(name):
    from launch.substitutions import LaunchConfiguration, PythonExpression
    values = {side: fields[name] for side, fields in ARM_DEFAULTS.items()}
    return PythonExpression([repr(values), "['", LaunchConfiguration('arm'), "']"])


def configure_arm(context):
    from launch.actions import SetLaunchConfiguration
    values = context.launch_configurations
    side = values.get('arm', 'right')
    if side not in ARM_DEFAULTS or values.get('model') != ARM_DEFAULTS[side]['model']:
        raise ValueError('arm=left 对应 A1L，arm=right 对应 A1R')
    suffix = 'l' if side == 'left' else 'r'
    names = [f'joint{i}-a1_{suffix}' for i in range(1, 8)]
    return [SetLaunchConfiguration('onero_joint_names', repr(names)),
            SetLaunchConfiguration('onero_projection_evaluator', 'joints('+','.join(names[:2])+')'),
            SetLaunchConfiguration('onero_left_arm', str(side == 'left').lower())]


def mode_default(name):
    """Lazy launch default, evaluated after hardware_mode has been resolved.

    DeclareLaunchArgument preserves explicit user values and values forwarded
    by the parent launch. Both entry points share this same profile table.
    """
    from launch.substitutions import LaunchConfiguration, PythonExpression

    values = {mode: defaults[name] for mode, defaults in MODE_DEFAULTS.items()}
    return PythonExpression([repr(values), "['", LaunchConfiguration("hardware_mode"), "']"])


def resolve_hardware_settings(mode=None):
    mode = "sim" if mode is None else str(mode).strip().lower()
    if mode not in MODE_DEFAULTS:
        raise ValueError("hardware_mode 必须为 sim 或 real")
    return {"hardware_mode": mode,
            "onero_simulate": str(mode == "sim").lower()}


def configure_hardware_mode(context):
    from launch.actions import SetLaunchConfiguration

    # Resolve before hardware actions. Reject ambiguous mode switches instead
    # of silently ignoring them or allowing them to open CAN unexpectedly.
    values = context.launch_configurations
    for name in ("use_fake_hardware", "simulate"):
        if name in values:
            raise ValueError(
                f"不支持启动参数 {name}；请使用 hardware_mode:=sim 或 hardware_mode:=real")
    settings = resolve_hardware_settings(values.get("hardware_mode"))
    return [SetLaunchConfiguration(key, value) for key, value in settings.items()]
