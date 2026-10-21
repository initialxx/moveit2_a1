"""Synchronize Cartesian edges without increasing either arm's speed or acceleration."""
import math

from builtin_interfaces.msg import Duration
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from rcl_interfaces.srv import GetParameters
from scipy.interpolate import BPoly
from trajectory_msgs.msg import JointTrajectoryPoint

from .constants import JOINTS, LEFT_JOINTS


def _seconds(duration):
    return duration.sec + duration.nanosec*1e-9


def _duration(seconds):
    nanoseconds = round(seconds*1e9)
    return Duration(sec=nanoseconds//10**9, nanosec=nanoseconds % 10**9)


def _scaled_limits(node, names, error):
    fields = ('has_velocity_limits', 'max_velocity', 'has_acceleration_limits', 'max_acceleration')
    values = node.call(GetParameters, '/move_group/get_parameters', GetParameters.Request(names=[
        f'robot_description_planning.joint_limits.{name}.{field}' for name in names for field in fields
    ])).values
    limits = []
    if len(values) != len(names)*len(fields):
        raise error('无法读取双臂规划速度和加速度限位')
    for index, name in enumerate(names):
        has_v, velocity, has_a, acceleration = values[index*4:index*4+4]
        if not (has_v.type == has_a.type == 1 and has_v.bool_value and has_a.bool_value
                and velocity.type == acceleration.type == 3
                and math.isfinite(velocity.double_value) and velocity.double_value > 0
                and math.isfinite(acceleration.double_value) and acceleration.double_value > 0):
            raise error(f'{name} 缺少有效的规划速度或加速度限位')
        limits.append((velocity.double_value*node.args.velocity,
                       acceleration.double_value*node.args.acceleration))
    return limits


def _curve(trajectory, names, limits, error):
    """Match the controller's position/velocity/acceleration interpolation."""
    joint_path = trajectory.joint_trajectory
    if set(joint_path.joint_names) != set(names) or len(joint_path.points) < 2:
        raise error('笛卡尔轨迹缺少完整的单臂关节路径')
    indices = [joint_path.joint_names.index(name) for name in names]
    times, derivatives = [], []
    for point in joint_path.points:
        values = (point.positions, point.velocities, point.accelerations)
        if any(len(values_) != len(names) or not all(map(math.isfinite, values_)) for values_ in values):
            raise error('笛卡尔轨迹缺少有效的位置、速度或加速度')
        times.append(_seconds(point.time_from_start))
        derivatives.append([[values_[index] for index in indices] for values_ in values])
    if times[0] != 0. or any(b <= a for a, b in zip(times, times[1:])):
        raise error('笛卡尔轨迹时间戳无效')
    curve = BPoly.from_derivatives(times, derivatives)
    # TOTG can return finite-difference derivatives at its endpoints. Make
    # corners stationary, confining this adjustment to the first/last 10 ms.
    times = sorted(set(times + [min(.01, times[-1]/2), max(times[-1]-.01, times[-1]/2)]))
    derivatives = [[curve(time, nu=order).tolist() for order in range(3)] for time in times]
    for boundary in (derivatives[0], derivatives[-1]):
        boundary[1:] = [[0.]*len(names), [0.]*len(names)]
    curve = BPoly.from_derivatives(times, derivatives)
    # Bernstein coefficients bound the entire spline, including between
    # samples. Stretch enough to respect the actual scaled MoveIt limits.
    velocities = abs(curve.derivative().c).max(axis=(0, 1))
    accelerations = abs(curve.derivative(2).c).max(axis=(0, 1))
    scale = max(1., *(max(v/v_limit, math.sqrt(a/a_limit))
                       for v, a, (v_limit, a_limit) in zip(velocities, accelerations, limits)))
    return times, curve, scale


def plan_dual_cartesian_path(node, start, left_waypoints, right_waypoints, error):
    if not node.dual or node.arm_side != 'both':
        raise error('双臂笛卡尔路径需要双臂服务并选择 both')
    if not left_waypoints or len(left_waypoints) != len(right_waypoints):
        raise error('左右臂必须提供相同数量的对应路径点')
    names = LEFT_JOINTS + JOINTS
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(names)
    limits = _scaled_limits(node, names, error)
    current, elapsed = dict(start), 0.
    for edge, corners in enumerate(zip(left_waypoints, right_waypoints), 1):
        curves = []
        for side, joints, corner, bounds in zip(('left', 'right'), (LEFT_JOINTS, JOINTS), corners,
                                                (limits[:7], limits[7:])):
            response = node.cartesian_path(current, [corner], group=side+'_arm', link=side+'_Link7')
            if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 1.-1e-6:
                raise error(f'第 {edge} 条边 {side} 笛卡尔路径不完整：'
                            f'fraction={response.fraction}，code={response.error_code.val}')
            curves.append(_curve(response.solution, joints, bounds, error))
        # Stretch the faster arm to the slower arm's duration. Both arms stop
        # at each corresponding corner; scaling never increases derivatives.
        duration = max(times[-1]*scale for times, _, scale in curves)
        scales = [duration/times[-1] for times, _, _ in curves]
        knots = {0., duration}
        for (times, _, _), scale in zip(curves, scales):
            knots.update(time*scale for time in times)
        # Check the combined state, including both arms and the environment.
        # Include all source knots so resampling preserves each quintic segment.
        samples = max(1, math.ceil(duration/.02))
        knots.update(duration*i/samples for i in range(samples+1))
        last = None
        for time in sorted(knots):
            if last is not None and time-last < 1e-8:
                continue
            point = JointTrajectoryPoint(time_from_start=_duration(elapsed+time))
            for (_, curve, _), scale in zip(curves, scales):
                point.positions.extend(curve(time/scale).tolist())
                point.velocities.extend((curve(time/scale, nu=1)/scale).tolist())
                point.accelerations.extend((curve(time/scale, nu=2)/scale**2).tolist())
            # Also bound the joint-space distance between collision samples.
            previous = trajectory.joint_trajectory.points[-1] if trajectory.joint_trajectory.points else point
            steps = max(1, math.ceil(max(abs(a-b) for a, b in zip(previous.positions, point.positions))/.02))
            for step in range(1, steps+1):
                sample_time = (last or 0.)+(time-(last or 0.))*step/steps
                positions = [value for (_, curve, _), scale in zip(curves, scales)
                             for value in curve(sample_time/scale).tolist()]
                state = dict(current)
                state.update(zip(names, positions))
                valid = node.validity(state, group='both_arms')
                if not valid.valid:
                    pairs = [(c.contact_body_1, c.contact_body_2) for c in valid.contacts]
                    raise error(f'双臂第 {edge} 条边存在碰撞或无效状态：{pairs}；请调整起点或减小 --size')
            if last is not None or not trajectory.joint_trajectory.points:
                trajectory.joint_trajectory.points.append(point)
            last = time
        current.update(zip(names, trajectory.joint_trajectory.points[-1].positions))
        elapsed += duration
    node.planned_start_state = node.planning_start_state(start)
    return trajectory
