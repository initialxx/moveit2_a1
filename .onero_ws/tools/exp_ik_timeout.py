"""IK 求解超时实测：0.05s vs 0.2s 在 7-DOF KDL 上的成功率。

背景
----
`armsdk/ros2` 的提交里有一句结论：
    kinematics.yaml: timeout 0.05→0.2s（7-DOF KDL 需要更长时间）
本工作区当前用的是 0.05。本脚本不照抄那个结论，而是**自己测**。

做法
----
不依赖配置文件，直接用 `/compute_ik` 服务并显式指定 `ik_request.timeout`
（`GetPositionIK` 的该字段会覆盖插件默认值），因此**一次启动即可对比多个超时值**。

测试用例的构造：随机取关节角 → 用 `/compute_fk` 算出末端位姿 → 再让 IK
解回该位姿。因为位姿一定可达（由 FK 生成），失败就意味着**求解器没解出来**，
而不是目标本身不可达。

⚠️ 种子位形是本实验的关键：最初用「生成该位姿的真值位形」当种子，
KDL 一步就收敛，四个超时值全是 100%，完全测不出差别。
因此这里**故意用零位形当种子**，模拟从当前（不同）状态出发去够目标位姿的真实场景。

用法（需先启动 move_group）：
    python3 tools/exp_ik_timeout.py
"""

import random
import sys

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState, PositionIKRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

JOINTS = [f"joint{i}-a1_r" for i in range(1, 8)]
GROUP = "arm"
TIP_LINK = "Link7"
N_CASES = 60
SEED = 20261014
TIMEOUTS = [0.02, 0.05, 0.1, 0.2]


class IkProbe(Node):
    def __init__(self):
        super().__init__("ik_timeout_probe")
        self._ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self._fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        for name, cli in (("/compute_ik", self._ik_cli), ("/compute_fk", self._fk_cli)):
            if not cli.wait_for_service(timeout_sec=30.0):
                self.get_logger().error(f"{name} 不可用")
                sys.exit(2)

    def _call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        return fut.result()

    def fk_pose(self, positions, seed_state):
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [TIP_LINK]
        req.robot_state = seed_state
        res = self._call(self._fk_cli, req)
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            return None
        return res.pose_stamped[0]

    def ik(self, pose, seed_state, timeout):
        req = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = GROUP
        r.ik_link_name = TIP_LINK
        r.pose_stamped = pose
        r.timeout.sec = 0
        r.timeout.nanosec = int(timeout * 1e9)
        r.avoid_collisions = False          # 只测求解器，不掺碰撞因素
        r.robot_state = seed_state
        req.ik_request = r
        res = self._call(self._ik_cli, req)
        if res is None:
            return None
        return res.error_code.val


def main():
    rclpy.init()
    node = IkProbe()
    rng = random.Random(SEED)

    # 用「全零位形」的关节状态作为 robot_state 骨架（service 要求带上）
    skeleton = RobotState()
    js = JointState()
    js.name = list(JOINTS)
    js.position = [0.0] * len(JOINTS)
    skeleton.joint_state = js

    def make_state(vals):
        st = RobotState()
        j = JointState()
        j.name = list(JOINTS)
        j.position = [float(v) for v in vals]
        st.joint_state = j
        return st

    cases = []
    for _ in range(N_CASES):
        vals = [rng.uniform(-1.4, 1.4) for _ in JOINTS]
        truth_state = make_state(vals)
        pose = node.fk_pose(vals, truth_state)     # FK 用真值位形算
        if pose is not None:
            # 但 IK 一律从【零位形】出发 —— 见文件头说明
            cases.append((pose, make_state([0.0] * len(JOINTS))))

    print(f"  构造出 {len(cases)} / {N_CASES} 个可达测试位姿（由 FK 生成）")
    print("  IK 种子：零位形（非真值位形，否则测不出差别）")
    print()
    print(f"  {'超时(s)':<10} {'成功':<8} {'成功率':<10}")
    print(f"  {'-'*30}")

    results = {}
    for t in TIMEOUTS:
        ok = 0
        for pose, seed in cases:
            code = node.ik(pose, seed, t)
            if code == 1:
                ok += 1
        results[t] = ok
        print(f"  {t:<10} {ok:<8} {100.0*ok/len(cases):.1f}%")

    print()
    base = results[TIMEOUTS[0]]
    best = max(results.values())
    for t in TIMEOUTS:
        d = results[t] - base
        print(f"  {t}s 相对 {TIMEOUTS[0]}s: {d:+d} 次成功")
    print()
    if best > base:
        gain = best - base
        print(f"  → 结论：缩短超时确实有损失。最长超时相对最短多解出 {gain} 个"
              f"（{(100.0*gain/len(cases)):.1f} 个百分点）")
    else:
        print("  → 结论：本数据集上超时长短无差别，0.05s 已足够")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
