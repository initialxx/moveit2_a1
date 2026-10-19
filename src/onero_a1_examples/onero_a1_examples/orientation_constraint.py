"""姿态路径约束：给 MoveIt 设置法兰平移过程中的朝向约束。"""
from moveit_msgs.msg import Constraints
from .support import finish_arm_plan, positive, run_example, translated_target, translation_arguments


def configure(parser):
    translation_arguments(parser)
    parser.add_argument("--path-angle-tolerance", type=positive, default=.1, help="路径中各轴朝向容差，rad")


def example(node):
    current, target = translated_target(node)
    constraints = Constraints(orientation_constraints=[
        node.orientation_constraint(current, node.args.path_angle_tolerance)])
    # The path constraint applies throughout the trajectory, not only at its end.
    trajectory = node.plan(node.start, node.pose_constraints(target), constraints)
    finish_arm_plan(node, trajectory)


def main(args=None):
    return run_example("orientation_constraint", __doc__, configure, example, args)
