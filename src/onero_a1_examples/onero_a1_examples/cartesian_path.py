"""笛卡尔路径：通过 GetCartesianPath 生成法兰直线或正方形路径。"""
from copy import deepcopy
from moveit_msgs.msg import MoveItErrorCodes
from .support import MoveItError, finish_arm_plan, positive, run_example, translated_target, translation_arguments


def configure(parser):
    translation_arguments(parser)
    parser.add_argument("--shape", choices=["line", "square"], default="line")
    parser.add_argument("--size", type=positive, default=.02, help="正方形边长，米")


def example(node):
    _, target = translated_target(node)
    waypoints = [target]
    if node.args.shape == "square":
        size = node.args.size
        for dx, dy in ((size, 0.), (size, size), (0., size), (0., 0.)):
            corner = deepcopy(target)
            corner.position.x += dx
            corner.position.y += dy
            waypoints.append(corner)
    response = node.cartesian_path(node.start, waypoints)
    # A partial path is not an acceptable replacement for the requested motion.
    if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 1.-1e-6:
        raise MoveItError(f"笛卡尔路径不完整：fraction={response.fraction}，code={response.error_code.val}")
    finish_arm_plan(node, response.solution)


def main(args=None):
    return run_example("cartesian_path", __doc__, configure, example, args)
