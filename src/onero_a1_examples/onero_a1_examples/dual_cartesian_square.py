"""双臂画正方形：两侧法兰同时沿 YZ 平面画一圈，保持各自朝向。"""
from copy import deepcopy
from .support import finish_arm_plan, positive, run_example


def configure(parser):
    parser.add_argument('--size', type=positive, default=.05, help='正方形边长，米')


def example(node):
    paths = []
    for side, direction in (('left', 1.), ('right', -1.)):
        origin = node.fk(node.start, link=side+'_Link7')
        corners = []
        for dy, dz in ((0., 1.), (1., 1.), (1., 0.), (0., 0.)):
            pose = deepcopy(origin)
            pose.position.y += direction*dy*node.args.size
            pose.position.z += dz*node.args.size
            corners.append(pose)
        paths.append(corners)
    finish_arm_plan(node, node.dual_cartesian_path(node.start, *paths))


def main(args=None):
    return run_example('dual_cartesian_square', __doc__, configure, example, args,
                       dual=True, planning_pipelines=())
