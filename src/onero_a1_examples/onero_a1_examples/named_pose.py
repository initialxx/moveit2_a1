"""命名姿态：读取 SRDF 中的 ready/home，通过 MoveIt 规划并可选执行。"""
from .support import finish_arm_plan, run_example


def configure(parser):
    parser.add_argument("--target", default="ready", help="SRDF arm 组内的姿态名")


def example(node):
    target = node.named_target(node.args.target)
    trajectory = node.plan(node.start, node.joint_constraints(target))
    finish_arm_plan(node, trajectory)


def main(args=None):
    return run_example("named_pose", __doc__, configure, example, args)
