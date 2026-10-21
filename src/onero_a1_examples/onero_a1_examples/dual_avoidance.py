"""双臂靠拢：分别求解末端目标，再联合规划并检查两臂间碰撞。"""
from onero_a1_common.constants import JOINTS, LEFT_JOINTS
from .support import MoveItError, positive, finish_arm_plan, run_example


def configure(parser):
    parser.add_argument('--approach', type=positive, default=.05, help='每侧沿 Y 方向靠拢的距离，米')


def example(node):
    target = dict(node.start)
    for side, names, direction in (('left', LEFT_JOINTS, -1.), ('right', JOINTS, 1.)):
        pose = node.fk(node.start, link=side+'_Link7')
        pose.position.y += direction*node.args.approach
        solution = node.inverse_kinematics(pose, target, group=side+'_arm', link=side+'_Link7')
        target.update({name:solution[name] for name in names})
    valid = node.validity(target)
    if not valid.valid:
        pairs = [(c.contact_body_1, c.contact_body_2) for c in valid.contacts]
        raise MoveItError(f'双臂目标存在碰撞：{pairs}；请减小 --approach')
    finish_arm_plan(node, node.plan(node.start, node.joint_constraints(target)))


def main(args=None):
    return run_example('dual_avoidance', __doc__, configure, example, args, dual=True)
