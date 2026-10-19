"""Read saved replay reports; never initialize ROS or send robot commands."""

import argparse
import json
import math
from pathlib import Path

from onero_a1_demos.trial_records import BRIDGE_IDENTITY


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reports", nargs="+", type=Path)
    options = parser.parse_args(args)
    if len(options.reports) < 2:
        parser.error("请提供至少两份 replay_trial 执行报告")
    reports = []
    try:
        for path in options.reports:
            report = json.loads(path.read_text())
            if (report.get("case") != "replay_trial" or not report.get("execute_requested")
                    or not report.get("trial_trajectory_sha256") or "trajectory" not in report):
                raise ValueError(f"{path} 不是完整的 replay_trial 执行报告")
            reports.append(report)
        baseline = reports[0]
        mismatches = []
        for path, report in zip(options.reports[1:], reports[1:]):
            for key in ("trial_trajectory_sha256", "trajectory_sha256", "model_fingerprint", "frame",
                        "velocity_scaling", "acceleration_scaling", "position_tolerance_m",
                        "orientation_tolerance_rad"):
                if report.get(key) != baseline.get(key):
                    mismatches.append(f"{path.name}: {key} 不一致")
            a, b = baseline.get("bridge_settings"), report.get("bridge_settings")
            if (a is None) != (b is None) or (a is not None and any(a.get(k) != b.get(k) for k in BRIDGE_IDENTITY)):
                mismatches.append(f"{path.name}: 安装/负载/硬件模式/频率不一致")
            first_start = baseline.get("execution_start", baseline["start"])
            next_start = report.get("execution_start", report["start"])
            if set(next_start) != set(first_start):
                mismatches.append(f"{path.name}: 起始关节名不一致")
                continue
            difference = max(abs(next_start[j] - first_start[j]) for j in first_start)
            if difference > 0.01:
                mismatches.append(f"{path.name}: 两轮实际起点相差 {difference:.6f} rad，超过 0.01 rad")
            first_pose = baseline.get("execution_start_pose", baseline["start_pose"])
            next_pose = report.get("execution_start_pose", report["start_pose"])
            pa, pb = first_pose["position"], next_pose["position"]
            distance = math.sqrt(sum((pa[k] - pb[k]) ** 2 for k in "xyz"))
            if distance > min(baseline["position_tolerance_m"], report["position_tolerance_m"]):
                mismatches.append(f"{path.name}: 两轮起点法兰位置相差 {distance * 1000:.2f} mm")
            qa, qb = ([pose["orientation"][axis] for axis in "xyzw"] for pose in (first_pose, next_pose))
            norm = math.sqrt(sum(v * v for v in qa) * sum(v * v for v in qb))
            if not math.isfinite(norm) or norm < 1e-12:
                raise ValueError(f"{path.name}: 起点朝向无效")
            angle = 2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(qa, qb))) / norm))
            if angle > min(baseline["orientation_tolerance_rad"], report["orientation_tolerance_rad"]):
                mismatches.append(f"{path.name}: 两轮起点朝向相差 {angle:.4f} rad")
        for path, report in zip(options.reports, reports):
            bridge = report.get("bridge_settings") or {}
            distance = report.get("fk_position_error_m")
            displacement = report.get("actual_translation_xyz_m")
            error = "未取得" if distance is None else f"{distance * 1000:.2f} mm"
            xyz = "未取得" if displacement is None else ", ".join(
                f"{k.upper()}={displacement[k] * 1000:+.2f}" for k in "xyz") + " mm"
            print(f"{path.name}\n  kp/kd 配置={bridge.get('kp', 'mock')}/{bridge.get('kd', 'mock')} "
                  f"结果={report.get('status')}\n  末端误差={error}；实际位移：{xyz}")
            print(f"  速度前馈={bridge.get('velocity_feedforward', '未记录')} "
                  f"比例={bridge.get('velocity_feedforward_scale', '未记录')}")
            if report.get("error"):
                print(f"  说明：{report['error']}")
        if mismatches:
            print("条件不可直接比较：\n  " + "\n  ".join(mismatches))
            return 2
        print("报告中的模型、轨迹、时间参数和起点满足对比条件。实物接触和外部负载仍需现场一致。")
        print("以上误差为 Link7 的反馈 FK；5 mm 实物末端精度仍需外部测量验证。")
        return 0
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
