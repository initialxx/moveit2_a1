"""关节目标：读取当前位置，修改一个关节，经 MoveIt 规划并可选执行。"""
from .support import JOINTS, finite, finish_arm_plan, run_example
from onero_a1_common.constants import LEFT_JOINTS
from onero_a1_common.moveit_client import MoveItError


def configure(parser):
    parser.add_argument("--joint", choices=LEFT_JOINTS+JOINTS, help="默认所选手臂的第1关节")
    parser.add_argument("--delta", type=finite, default=.1, help="相对关节角度，rad")


def example(node):
    target = dict(node.start)
    joint = node.args.joint or node.joints[0]
    if joint not in node.joints:
        raise MoveItError("--joint 不属于 --arm 选择的手臂")
    target[joint] += node.args.delta
    # JointConstraint describes the goal; MoveGroup chooses a collision-free path.
    goal = node.joint_constraints(target)
    trajectory = node.plan(node.start, goal)
    finish_arm_plan(node, trajectory)


def main(args=None):
    return run_example("joint_goal", __doc__, configure, example, args)
