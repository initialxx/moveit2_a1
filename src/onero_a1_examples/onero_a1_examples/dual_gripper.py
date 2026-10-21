"""双夹爪控制：分别设置左右开度，由 MoveIt 协调两个 GripperCommand 接口。"""
from .gripper import opening
from .support import run_example


def configure(parser):
    parser.add_argument('--left-opening', type=opening, default=1.)
    parser.add_argument('--right-opening', type=opening, default=1.)


def example(node):
    target = {'left_woan_picker_left_finger_joint': node.args.left_opening*1.1,
              'right_woan_picker_left_finger_joint': node.args.right_opening*1.1}
    trajectory = node.plan(node.start, node.joint_constraints(target), group='both_grippers')
    node.show_plan(trajectory)
    if node.args.execute:
        node.execute(trajectory, controller=['left_gripper_controller', 'right_gripper_controller'])


def main(args=None):
    return run_example('dual_gripper', __doc__, configure, example, args,
                       planning_pipelines=(), dual=True)
