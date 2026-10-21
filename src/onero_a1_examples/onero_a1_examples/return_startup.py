"""返回启动姿态：读取本次话题桥启动快照，通过 MoveIt 规划返回。"""
from .support import finish_arm_plan, run_example


def example(node):
    # The snapshot is captured by the adapter once at startup, not by this script.
    snapshot = node.startup_snapshot()
    target = snapshot["joint_positions"]
    node.startup_id = snapshot["id"]
    trajectory = node.plan(node.start, node.joint_constraints(target))
    finish_arm_plan(node, trajectory)


def main(args=None):
    return run_example("return_startup", __doc__, lambda _: None, example, args, allow_both=True)
