"""Small CLI and lifecycle helpers shared by the runnable examples."""
import argparse
from copy import deepcopy
import math
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from onero_a1_common.constants import JOINTS
from onero_a1_common.moveit_client import MoveItClient, MoveItError


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("必须是有限数值")
    return value


def positive(value):
    value = finite(value)
    if value <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return value


def scaling(value):
    value = positive(value)
    if value > 1:
        raise argparse.ArgumentTypeError("缩放比例必须在 (0, 1] 内")
    return value


def translation_arguments(parser):
    parser.add_argument("--dx", type=finite, default=0., help="X 方向位移，米")
    parser.add_argument("--dy", type=finite, default=0., help="Y 方向位移，米")
    parser.add_argument("--dz", type=finite, default=.01, help="Z 方向位移，米")


def translated_target(node):
    current = node.fk(node.start)
    target = deepcopy(current)
    for axis in "xyz":
        setattr(target.position, axis, getattr(target.position, axis)+getattr(node.args, "d"+axis))
    return current, target


def finish_arm_plan(node, trajectory):
    node.show_plan(trajectory)
    if node.args.execute:
        node.execute(trajectory)


def run_example(name, description, configure, scenario, args=None, *, query_only=False):
    parser = argparse.ArgumentParser(description=description)
    configure(parser)
    parser.add_argument("--frame", choices=["world", "base_link"], default="world", help="参考系，默认 world")
    if query_only:
        parser.set_defaults(execute=False)
    else:
        parser.add_argument("--execute", action="store_true", help="执行轨迹；默认只规划")
        parser.add_argument("--velocity", type=scaling, default=.05, help="速度缩放，默认 0.05")
        parser.add_argument("--acceleration", type=scaling, default=.05, help="加速度缩放，默认 0.05")
        parser.add_argument("--planning-time", type=positive, default=10.)
    supplied = list(args) if args is not None else sys.argv[1:]
    split = supplied.index("--ros-args") if "--ros-args" in supplied else len(supplied)
    options = parser.parse_args(supplied[:split])
    options.case = name
    rclpy.init(args=supplied[split:], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    try:
        node = MoveItClient(options)
        node.start = {n: q for n, q in node.state().items() if n in JOINTS}
        scenario(node)
    except (Exception, KeyboardInterrupt) as exc:
        print(f"示例失败：{str(exc) or 'interrupted'}", file=sys.stderr)
        code = 1
    finally:
        if node is not None:
            if not node.cleanup():
                code = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
