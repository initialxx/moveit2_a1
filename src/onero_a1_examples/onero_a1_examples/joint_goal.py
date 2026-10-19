"""关节目标：读取当前位置，修改一个关节，经 MoveIt 规划并可选执行。"""
from .support import JOINTS, finite, finish_arm_plan, run_example


def configure(parser):
    parser.add_argument("--joint", choices=JOINTS, default=JOINTS[0])
    parser.add_argument("--delta", type=finite, default=.1, help="相对关节角度，rad")


def example(node):
    target = dict(node.start)
    target[node.args.joint] += node.args.delta
    # JointConstraint describes the goal; MoveGroup chooses a collision-free path.
    goal = node.joint_constraints(target)
    trajectory = node.plan(node.start, goal)
    finish_arm_plan(node, trajectory)


def main(args=None):
    return run_example("joint_goal", __doc__, configure, example, args)
