"""SRDF 碰撞矩阵对照实验：采样关节状态，统计 MoveIt 判定的自碰撞无效状态数。

背景
----
发现 `armsdk/ros2/onero_moveit_config/config/a1.srdf` 的 `disable_collisions`
只有 10 对，而 armsdk 的权威文件 `onero_sdk/urdf/A1/a1_r.srdf` 有 28 对。
排除数量本身不能判断碰撞配置是否正确。是否误报需要核对几何接触，
不能仅因更多排除使规划通过，就将所有连杆对标成 Never。
当前生成器只排除直接相邻连杆；功能验收使用 onero_a1_examples collision_check/planning_scene。

本脚本不修改任何配置，只**测量**当前加载的 SRDF 下的状态有效性，
把「碰撞矩阵差异」从「静态 diff」升级成「可观测的功能影响」。

用法（需先启动 move_group）：
    python3 tools/exp_srdf_validity.py [标签]
"""

import math
import random
import sys

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

JOINTS = [f"joint{i}-a1_r" for i in range(1, 8)]
GROUP = "arm"
N_SAMPLES = 200
SEED = 20261014


def build_samples():
    """构造有代表性的测试状态。

    除了均匀随机采样，还刻意加入「手臂折返」构型——
    这正是 Link7 与 base_link 等远端连杆互相靠近的场景，
    也是缺失碰撞排除对时最容易暴露问题的区域。
    """
    rng = random.Random(SEED)
    samples = []

    # 1) 均匀随机
    for _ in range(N_SAMPLES):
        samples.append([rng.uniform(-1.5, 1.5) for _ in JOINTS])

    # 2) 折返构型：让末端卷回底座附近
    folded = [
        [0.0, -1.5, 0.0, -1.5, 0.0, 0.5, 0.0],
        [0.0, -2.0, 0.0, -2.0, 0.0, 1.0, 0.0],
        [0.0, -1.8, 0.0, -1.8, 0.0, 0.0, 0.0],
        [1.0, -1.5, -1.0, -1.5, 1.0, 0.5, -1.0],
        [-1.0, -1.5, 1.0, -1.5, -1.0, 0.5, 1.0],
        [0.0, -2.2, 0.0, -2.2, 0.0, 1.5, 0.0],
    ]
    samples.extend(folded)

    # 3) 全零（home）
    samples.append([0.0] * len(JOINTS))
    return samples


class ValidityProbe(Node):
    def __init__(self):
        super().__init__("srdf_validity_probe")
        self.cli = self.create_client(GetStateValidity, "/check_state_validity")
        if not self.cli.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("/check_state_validity 不可用")
            sys.exit(2)

    def check(self, positions):
        req = GetStateValidity.Request()
        req.group_name = GROUP
        rs = RobotState()
        js = JointState()
        js.name = list(JOINTS)
        js.position = [float(p) for p in positions]
        rs.joint_state = js
        req.robot_state = rs

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            return None, []
        self_pairs = {
            (c.contact_body_1, c.contact_body_2) for c in res.contacts
        }
        return res.valid, sorted(self_pairs)


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "current"

    rclpy.init()
    node = ValidityProbe()
    samples = build_samples()

    invalid = 0
    collisions = {}
    for s in samples:
        valid, pairs = node.check(s)
        if valid is None:
            print("  ✗ 服务调用超时")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(3)
        if not valid:
            invalid += 1
            for p in pairs:
                collisions[p] = collisions.get(p, 0) + 1

    total = len(samples)
    print(f"  SRDF 变体           : {label}")
    print(f"  采样状态数          : {total}")
    print(f"  判定为无效的状态数  : {invalid}  ({100.0 * invalid / total:.1f}%)")
    print(f"  有效状态数          : {total - invalid}")
    if collisions:
        print("  触发碰撞的连杆对（次数）:")
        for pair, cnt in sorted(collisions.items(), key=lambda kv: -kv[1])[:15]:
            print(f"      {pair[0]:<12} <-> {pair[1]:<12} {cnt}")

    node.destroy_node()
    rclpy.shutdown()
    return invalid


if __name__ == "__main__":
    sys.exit(0 if main() is not None else 1)
