"""夹爪示例：规划 gripper 组，通过 MoveIt 的 GripperCommand 映射执行。"""
import argparse
from .support import finite, run_example

JOINT = "woan_picker_left_finger_joint"


def opening(value):
    value = finite(value)
    if not 0 <= value <= 1:
        raise argparse.ArgumentTypeError("开度必须在 [0, 1] 内")
    return value


def configure(parser):
    parser.add_argument("--opening", type=opening, default=1., help="0=全闭，0.5=半开，1=全开")


def example(node):
    # Model opening -> master angle. The adapter applies device calibration.
    target = {JOINT: node.args.opening*1.1}
    trajectory = node.plan(node.start, node.joint_constraints(target), group="gripper")
    node.show_plan(trajectory)
    if node.args.execute:
        node.execute(trajectory, controller="gripper_controller")


def main(args=None):
    return run_example("gripper", __doc__, configure, example, args)
