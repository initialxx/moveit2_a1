"""位姿目标：指定 Link7 法兰的平移和旋转，通过 IK 和规划管线到达。"""
from scipy.spatial.transform import Rotation
from .support import finite, finish_arm_plan, run_example, translated_target, translation_arguments


def configure(parser):
    translation_arguments(parser)
    for axis in ("roll", "pitch", "yaw"):
        parser.add_argument("--"+axis, type=finite, default=0., help="绕所选参考系轴的增量旋转，rad")


def example(node):
    _, target = translated_target(node)
    q = target.orientation
    rotation = Rotation.from_euler("xyz", [node.args.roll, node.args.pitch, node.args.yaw])
    result = (rotation * Rotation.from_quat([q.x, q.y, q.z, q.w])).as_quat()
    q.x, q.y, q.z, q.w = map(float, result)
    # PositionConstraint + OrientationConstraint define the endpoint only.
    # The planned path is not required to be a Cartesian straight line.
    trajectory = node.plan(node.start, node.pose_constraints(target))
    finish_arm_plan(node, trajectory)


def main(args=None):
    return run_example("pose_goal", __doc__, configure, example, args,
                       planning_pipelines=("ompl", "stomp"))
