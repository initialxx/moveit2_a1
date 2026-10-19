"""Offline data preparation and FK for the SDK/MoveIt comparison."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from onero_a1_common.constants import JOINTS
from .trial_records import fingerprint

CONTROL_KEYS = ("model", "mounting", "can_channel", "simulate", "with_gripper",
                "gravity_compensation", "control_rate", "kp", "kd",
                "velocity_feedforward", "velocity_feedforward_scale",
                "velocity_feedforward_timeout")


def file_hash(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def vector(values):
    if len(values) != 7 or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError("需要七个有限关节数值")
    return list(values)


class FlangeFK:
    def __init__(self, urdf, frame_from_base=None):
        root = ET.fromstring(urdf)
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, child = [], "Link7"
        while child != "base_link":
            joint = by_child[child]
            origin = joint.find("origin")
            fixed = np.eye(4)
            fixed[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            fixed[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ") if joint.get("type") != "fixed" else np.zeros(3)
            chain.append((joint.get("name"), fixed, axis))
            child = joint.find("parent").get("link")
        self.chain = chain[::-1]
        self.frame = np.eye(4) if frame_from_base is None else np.asarray(frame_from_base, dtype=float)

    def matrix(self, positions):
        q = dict(zip(JOINTS, vector(positions)))
        result = self.frame.copy()
        for name, fixed, axis in self.chain:
            motion = np.eye(4)
            if np.linalg.norm(axis):
                motion[:3, :3] = Rotation.from_rotvec(axis / np.linalg.norm(axis) * q[name]).as_matrix()
            result = result @ fixed @ motion
        return result

    def errors(self, positions, target):
        actual = self.matrix(positions)
        xyz = actual[:3, 3] - np.array([target["position"][a] for a in "xyz"])
        desired = Rotation.from_quat([target["orientation"][a] for a in "xyzw"])
        angle = (desired.inv() * Rotation.from_matrix(actual[:3, :3])).magnitude()
        return {"xyz_m": xyz.tolist(), "position_m": float(np.linalg.norm(xyz)), "orientation_rad": float(angle)}


def pose_matrix(pose):
    matrix = np.eye(4)
    matrix[:3, 3] = [pose["position"][a] for a in "xyz"]
    matrix[:3, :3] = Rotation.from_quat([pose["orientation"][a] for a in "xyzw"]).as_matrix()
    return matrix


def recorded_segment(report, trace_path, phase, hold_seconds):
    if report.get("execution_action_result") != {"status": 4, "error_code": 1}:
        raise ValueError("A 轮必须已由控制器正常完成；末端精度失败可以作为对照")
    if fingerprint(report.get("trajectory")) != report.get("trajectory_sha256"):
        raise ValueError("A 轮轨迹校验不通过")
    settings = report.get("bridge_settings") or {}
    if any(k not in settings for k in CONTROL_KEYS) or settings["simulate"]:
        raise ValueError("需要包含完整控制配置的真机报告")
    traces, starts, finishes, closed = [], {}, [], False
    with Path(trace_path).open() as stream:
        for line in stream:
            row = json.loads(line)
            if row["source"] == "/onero_topic_bridge/control_trace":
                traces.append(row["data"])
            elif row["source"] == "/execute_trajectory/_action/status":
                for status in row["data"]["status_list"]:
                    if status["status"] == 2:
                        info = status["goal_info"]
                        starts[bytes(info["goal_id"]["uuid"]).hex()] = info["stamp"]["sec"] * 10**9 + info["stamp"]["nanosec"]
            elif row["source"] == "/execute_trajectory/result":
                finishes.append(row)
            elif row["source"] == "summary":
                closed = True
    if not closed or len(finishes) != 1 or not traces:
        raise ValueError("记录器须已结束，且记录中只能有一轮执行")
    finish = finishes[0]
    if finish["data"]["response"]["status"] != 4:
        raise ValueError("A 轮动作没有成功完成")
    end_ns = finish["received_ros_ns"]
    start_ns = end_ns + 10**9 if phase == "hold" else starts[finish["data"]["goal_id"]]
    stop_ns = end_ns + (1 + hold_seconds) * 10**9
    if traces[-1]["ros_start_ns"] < stop_ns:
        raise ValueError(f"A 轮记录不足：动作结束后至少记录 {1 + hold_seconds:g} 秒")
    if phase == "motion":
        before = [s for s in traces if s["ros_start_ns"] <= start_ns]
        if not before:
            raise ValueError("缺少动作开始前的反馈")
        start_ns = before[-1]["ros_start_ns"]
    selected = [s for s in traces if start_ns <= s["ros_start_ns"] <= stop_ns]
    if len(selected) < 2:
        raise ValueError("对照段没有足够数据")
    period = 1 / settings["control_rate"]
    q_final = [report["planned_endpoint"][n] for n in JOINTS]
    offsets = [a-b for a,b in zip(selected[0]["raw_command_position"], selected[0]["command_position"])]
    for i, sample in enumerate(selected):
        if sample["joint_names"] != JOINTS:
            raise ValueError("记录的关节顺序不一致")
        for name in ("sdk_input_position", "sdk_input_velocity", "feedback_position", "command_position"):
            vector(sample[name])
        if (sample["exception"] or sample["missed_joint_indices"] or sample["can_faults_total"]
                or sample["command_kp"] != [settings["kp"]]*7 or sample["command_kd"] != [settings["kd"]]*7
                or sample["velocity_feedforward"] != settings["velocity_feedforward"]
                or sample["gravity_compensation"] != settings["gravity_compensation"]):
            raise ValueError("A 轮出现故障、丢帧或控制配置变化，不能作为对照")
        if max(abs(a-b) for a,b in zip(sample["sdk_input_position"], sample["command_position"])) > 1e-8:
            raise ValueError("A 轮位置指令被限幅，不能作为本次直接重放基准")
        if max(abs(a-b-o) for a,b,o in zip(sample["raw_command_position"],sample["command_position"],offsets)) > 1e-8:
            raise ValueError("A 轮零位偏移发生变化")
        if phase == "hold" and (max(map(abs, sample["sdk_input_velocity"])) > 1e-7
                               or max(abs(a-b) for a,b in zip(sample["sdk_input_position"],q_final)) > 1e-8):
            raise ValueError("A 轮保持段的目标或速度不是静止保持")
        if i and (sample["cycle"] != selected[i-1]["cycle"]+1
                  or not 0 < sample["monotonic_start"]-selected[i-1]["monotonic_start"] < 2*period):
            raise ValueError("控制记录有缺帧或过长周期")
    return settings, selected, offsets


def validate_bundle(bundle):
    candidate = deepcopy(bundle)
    expected = candidate.pop("bundle_sha256", None)
    if candidate.get("schema_version") != 1 or candidate.get("kind") != "onero_sdk_comparison" or fingerprint(candidate) != expected:
        raise ValueError("对照文件格式或校验不通过")
    if not bundle.get("collision_checked") or not bundle.get("samples"):
        raise ValueError("缺少 MoveIt 场景检查或命令序列")
    settings = bundle["settings"]
    if settings["simulate"] or settings["model"] != "A1R":
        raise ValueError("本工具只接受 A1R 真机记录")
    if not math.isfinite(settings["control_rate"]) or not 1 <= settings["control_rate"] <= 1000:
        raise ValueError("控制频率无效")
    if not 0 <= settings["kp"] <= 500 or not 0 <= settings["kd"] <= 5:
        raise ValueError("增益越界")
    previous = -1.0
    for sample in bundle["samples"]:
        vector(sample["positions"]); vector(sample["velocities"])
        if not math.isfinite(sample["time_s"]) or sample["time_s"] <= previous:
            raise ValueError("命令时间不递增")
        previous = sample["time_s"]
        for q,(lo,hi) in zip(sample["positions"],bundle["joint_bounds"]):
            if not lo <= q <= hi:
                raise ValueError("命令超出关节限位")
    if not 0 <= bundle["samples"][0]["time_s"] <= 1e-8 or previous > 120:
        raise ValueError("命令序列必须从零开始，且最长 120 秒")
    if max(map(abs,bundle["samples"][-1]["velocities"])) > 1e-7:
        raise ValueError("对照命令必须以零速度结束")
    return bundle


def check_start(bundle, actual):
    actual = vector(actual)
    reference = bundle["initial_feedback"]
    difference = max(abs(a-b) for a,b in zip(actual,reference))
    fk = FlangeFK(bundle["urdf"], bundle["frame_from_base"])
    a,b = fk.matrix(actual),fk.matrix(reference)
    distance = float(np.linalg.norm(a[:3,3]-b[:3,3]))
    angle = float(Rotation.from_matrix(b[:3,:3].T@a[:3,:3]).magnitude())
    if difference > .01 or distance > .005 or angle > .05:
        raise ValueError(f"B 轮起点不符：最大关节差 {difference:.5f} rad，法兰差 {distance*1000:.2f} mm；不发送运动目标")
    return {"max_joint_difference_rad":difference,"flange_difference_m":distance,"orientation_difference_rad":angle}


def metrics(bundle, samples):
    if not samples:
        return None
    fk = FlangeFK(bundle["urdf"],bundle["frame_from_base"])
    tail = [s for s in samples if s["time_s"] >= samples[-1]["time_s"]-3]
    cache = {}
    for sample in tail:
        key = tuple(sample["feedback_position"])
        if key not in cache:
            cache[key] = fk.errors(key,bundle["pose_target"])
    errors = [cache[tuple(s["feedback_position"])] for s in tail]
    final = samples[-1]
    return {"final_fk":fk.errors(final["feedback_position"],bundle["pose_target"]),
            "last_3s_fk_mean_m":statistics.mean(e["position_m"] for e in errors),
            "final_joint_error_rad":[q-a for q,a in zip(bundle["samples"][-1]["positions"],final["feedback_position"])],
            "last_3s_feedback_peak_to_peak_rad":[max(s["feedback_position"][i] for s in tail)-min(s["feedback_position"][i] for s in tail) for i in range(7)]}
