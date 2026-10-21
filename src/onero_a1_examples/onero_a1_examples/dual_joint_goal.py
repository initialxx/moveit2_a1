"""双臂联合运动：将左右关节目标放入同一条十四关节轨迹。"""
from .support import finite, finish_arm_plan, run_example


def configure(parser):
    parser.add_argument('--joint-index', type=int, choices=range(1, 8), default=1)
    parser.add_argument('--left-delta', type=finite, default=-.2, help='左臂相对转角，rad')
    parser.add_argument('--right-delta', type=finite, default=.2, help='右臂相对转角，rad')


def example(node):
    target = dict(node.start)
    for suffix, delta in (('l', node.args.left_delta), ('r', node.args.right_delta)):
        target[f'joint{node.args.joint_index}-a1_{suffix}'] += delta
    finish_arm_plan(node, node.plan(node.start, node.joint_constraints(target)))


def main(args=None):
    return run_example('dual_joint_goal', __doc__, configure, example, args, dual=True)
