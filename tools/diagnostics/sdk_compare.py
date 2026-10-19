"""SDK/MoveIt comparison. Export and preview never open CAN; run needs --execute."""

import argparse
from collections import deque
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np

from .sdk_comparison import (CONTROL_KEYS, JOINTS, FlangeFK, check_start, file_hash,
                             metrics, pose_matrix, recorded_segment, validate_bundle, vector)
from .trial_records import fingerprint


def sdk_files():
    import onero_sdk
    names = ("arm", "can_backend", "can_bus", "protocol", "safety", "safe_backend",
             "model_registry", "dynamics", "mounting", "zero_offset_backend", "zero_calibration")
    root = Path(onero_sdk.__file__).parent
    return {str(root / (name+".py")): None for name in names}


def write_new(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def check_planning_model(node, report):
    """Allow a CAN-free planning server only when geometry/SRDF/limits agree exactly."""
    if node.report["model_fingerprint"] != report["model_fingerprint"]:
        raise ValueError("当前 MoveIt 模型与 A 轮不同")


def export_bundle(args):
    from .client import DiagnosticClient
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene
    from rosidl_runtime_py.convert import message_to_ordereddict
    from ament_index_python.packages import get_package_share_directory
    import rclpy

    report = json.loads(args.from_report.read_text())
    if (report.get("bridge_settings") or {}).get("gripper_model", "none") != "none":
        raise ValueError("SDK 直连 A/B 工具尚不接管夹爪状态；夹爪模型请使用 MoveIt 测试，旧单臂对照使用 gripper_model:=none")
    settings, selected, offsets = recorded_segment(report, args.trace, args.phase, args.hold_seconds)
    options = SimpleNamespace(case="sdk_compare_export", execute=False, frame=report["frame"],
                              velocity=.05, acceleration=.05, position_tolerance=.005,
                              orientation_tolerance=.05)
    rclpy.init(args=[])
    node = DiagnosticClient(options)
    try:
        node.load_model()
        check_planning_model(node,report)
        if node.report["bridge_settings"] is not None and any(node.report["bridge_settings"].get(k) != settings[k] for k in CONTROL_KEYS):
            raise ValueError("当前桥接控制配置与 A 轮不同")
        values = node.call(GetParameters, "/move_group/get_parameters",
                          GetParameters.Request(names=["robot_description"])).values
        urdf = values[0].string_value
        base_fk = FlangeFK(urdf)
        zero = dict.fromkeys(JOINTS, 0.0)
        transform = pose_matrix(message_to_ordereddict(node.fk(zero))) @ np.linalg.inv(base_fk.matrix([0.0]*7))
        fk = FlangeFK(urdf, transform)
        for key, pose in (("actual_endpoint", "actual_pose"), ("planned_endpoint", "planned_pose")):
            error = fk.errors([report[key][n] for n in JOINTS], report[pose])
            if error["position_m"] > 1e-8 or error["orientation_rad"] > 1e-7:
                raise ValueError("离线 FK 与 A 轮 MoveIt FK 不一致")
        commands = {tuple(s["sdk_input_position"]) for s in selected}
        initial = selected[0]["feedback_position"]
        # Check the small transition from measured A start to its first setpoint.
        for fraction in np.linspace(0, 1, 21):
            commands.add(tuple(a+(b-a)*fraction for a,b in zip(initial,selected[0]["sdk_input_position"])))
        for q in commands:
            node.require_valid(dict(zip(JOINTS, q)))
        parts = PlanningSceneComponents
        scene = node.call(GetPlanningScene, "/get_planning_scene", GetPlanningScene.Request(
            components=parts(components=parts.WORLD_OBJECT_GEOMETRY | parts.ROBOT_STATE_ATTACHED_OBJECTS
                             | parts.ALLOWED_COLLISION_MATRIX | parts.TRANSFORMS)))
        watched = sdk_files()
        sdk_code = {Path(p).name:file_hash(p) for p in watched}
        workspace = Path(__file__).resolve().parents[3]
        for relative in ("src/onero_a1_description/urdf/onero_a1_body.urdf",
                         "src/onero_a1_description/urdf/onero_a1.ros2_control.xacro",
                         "src/onero_a1_moveit_config/config/onero_a1.srdf",
                         "src/onero_a1_moveit_config/config/joint_limits.yaml"):
            watched[str(workspace / relative)] = None
        watched[str(args.environment_file.resolve())] = None
        # Also pin the installed SDK's own dynamics URDF, independent of ROS meshes.
        import onero_sdk
        for path in (Path(onero_sdk.__file__).parent / "urdf" / "A1").glob("*.urdf"):
            watched[str(path)] = None
        watched = {str(Path(p).resolve()): file_hash(p) for p in watched}
        t0 = selected[0]["monotonic_start"]
        samples = [{"time_s": s["monotonic_start"]-t0, "positions": s["sdk_input_position"],
                    "velocities": s["sdk_input_velocity"], "feedback_position":s["feedback_position"],
                    "source_torque_ff":s["command_torque_ff"]} for s in selected]
        bundle = {"schema_version":1,"kind":"onero_sdk_comparison","phase":args.phase,
                  "source_report":str(args.from_report.resolve()),"source_trace":str(args.trace.resolve()),
                  "source_report_sha256":file_hash(args.from_report),"source_trace_sha256":file_hash(args.trace),
                  "settings":settings,"zero_offsets":offsets,"joint_names":JOINTS,
                  "initial_feedback":initial,"samples":samples,"urdf":urdf,
                  "frame":report["frame"],"tip_link":"Link7","frame_from_base":transform.tolist(),
                  "pose_target":report["pose_target"],"joint_bounds":[node.bounds[n] for n in JOINTS],
                  "position_tolerance_m":report["position_tolerance_m"],
                  "orientation_tolerance_rad":report["orientation_tolerance_rad"],
                  "collision_checked":True,"collision_checked_points":len(commands),
                  "scene_snapshot":message_to_ordereddict(scene.scene),
                  "validated_at_unix_s":time.time(),"watched_files":watched,
                  "ros_domain_id":int(os.environ.get("ROS_DOMAIN_ID","0")),
                  "sdk_code":sdk_code,
                  "model_fingerprint":report["model_fingerprint"]}
        bundle["source_metrics"] = metrics(bundle,samples)
        bundle["bundle_sha256"] = fingerprint(bundle)
        validate_bundle(bundle)
        write_new(args.output,bundle)
        show_bundle(bundle)
        print(f"已导出：{args.output.resolve()}；此步骤没有发送运动目标。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def show_bundle(bundle):
    s = bundle["settings"]
    print(f"对照段={bundle['phase']} | {len(bundle['samples'])} 帧 | "
          f"{bundle['samples'][-1]['time_s']:.3f}s | {s['control_rate']}Hz | kp/kd={s['kp']}/{s['kd']}")
    print(f"CAN={s['can_channel']} | mounting={s['mounting']} | 速度前馈={s['velocity_feedforward']}")
    print("B 轮必须与 A 轮起点相差不超过 0.01 rad / 5 mm；参数及场景须保持一致。")
    print(f"A 轮保持误差={bundle['source_metrics']['last_3s_fk_mean_m']*1000:.2f} mm（Link7 反馈 FK）")


def prepare_bundle(args):
    """Use MoveIt to return to the measured A start, rather than a startup snapshot."""
    from .client import DiagnosticClient
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rclpy.signals import SignalHandlerOptions
    import rclpy

    bundle = validate_bundle(json.loads(args.bundle.read_text()))
    check_files(bundle)
    output = args.output or Path(".logs/diagnostics") / f"{time.time_ns()}-sdk_compare_prepare.json"
    if output.exists():
        raise ValueError(f"输出文件已存在：{output}")
    options = SimpleNamespace(case="sdk_compare_prepare", execute=args.execute,
                              frame=bundle["frame"], velocity=.05, acceleration=.05,
                              planning_time=10.0, goal_tolerance=.01,
                              position_tolerance=.005, orientation_tolerance=.05)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = DiagnosticClient(options)
    code = 0
    try:
        node.load_model()
        if node.report["model_fingerprint"] != bundle["model_fingerprint"]:
            raise ValueError("当前 MoveIt 模型与 A 轮不同；请使用原真机配置启动服务")
        settings = node.report["bridge_settings"]
        if settings is None or any(settings.get(k) != bundle["settings"][k] for k in CONTROL_KEYS):
            raise ValueError("当前真机控制配置与 A 轮不同")
        target = dict(zip(JOINTS, vector(bundle["initial_feedback"])))
        start = {n: q for n, q in node.state().items() if n in JOINTS}
        node.report.update(start=start, source_bundle=str(args.bundle.resolve()),
                           bundle_sha256=bundle["bundle_sha256"], requested_joint_target=target)
        node.require_valid(start)
        node.require_valid(target)
        target_pose = message_to_ordereddict(node.fk(target))
        expected = FlangeFK(bundle["urdf"], bundle["frame_from_base"]).matrix(bundle["initial_feedback"])
        if not np.allclose(pose_matrix(target_pose), expected, atol=1e-8, rtol=0):
            raise ValueError("当前安装坐标变换与 A 轮不同")
        node.report["pose_target"] = target_pose
        print(f"返回目标：对照包内 A 轮实测起点，{target}", flush=True)
        try:
            ready = check_start(bundle, [start[n] for n in JOINTS])
        except ValueError:
            ready = None
        if ready is None:
            trajectory = node.plan(start, node.joint_constraints(target, tolerance=.001))
            node.validate_plan(trajectory)
            if args.execute:
                node.execute(trajectory)
                ready = check_start(bundle, [node.report["actual_endpoint"][n] for n in JOINTS])
        if ready is not None:
            node.report.update(status="start_ready", start_check=ready)
            print(f"A 轮起点检查通过：最大关节差 {ready['max_joint_difference_rad']:.6f} rad，"
                  f"法兰差 {ready['flange_difference_m']*1000:.2f} mm", flush=True)
        else:
            node.report["status"] = "planning_checks_passed"
            print("仅规划；确认轨迹后添加 --execute 返回 A 轮起点。", flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        code = 1
        node.report.update(status="failed", error=str(exc) or "interrupted")
        print(f"返回 A 轮起点失败：{node.report['error']}", file=sys.stderr)
    finally:
        if not node.cleanup():
            code = 1
            node.report["status"] = "failed"
        write_new(output, node.report)
        print(f"结果：{node.report['status']}；报告：{output.resolve()}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


def check_files(bundle):
    for path, digest in bundle["watched_files"].items():
        if file_hash(path) != digest:
            raise ValueError(f"模型、SDK 或场景配置已改变，请重新导出：{path}")
    if {Path(p).name:file_hash(p) for p in sdk_files()} != bundle["sdk_code"]:
        raise ValueError("当前解释器加载的 SDK 与导出对照时不同")


def refuse_running_bridge(domain_id):
    """An old bridge may predate the cooperative CAN lock; also check ROS graph."""
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=[],domain_id=domain_id)
    node = Node("onero_sdk_comparison_guard")
    try:
        deadline = time.monotonic()+1.5
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        if any("topic_bridge" in name for name in node.get_node_names()):
            raise ValueError("ROS 话题桥仍在运行；先结束真机 launch，SDK 直连不能同时占用 CAN")
    finally:
        node.destroy_node()
        rclpy.shutdown()


class ObservedBus:
    """Transparent SDK CanTransport decorator: record wire frames, never rewrite."""
    def __init__(self, bus, model):
        from onero_sdk.protocol import MotorLimits
        self.bus = bus
        self.ids = {j.can_id:i for i,j in enumerate(model.joints)}
        self.responses = {j.response_id:i for i,j in enumerate(model.joints)}
        self.limits = [MotorLimits(p_max=j.encoding_p_max,v_max=j.codec_v_max,t_max=j.codec_t_max)
                       for j in model.joints]
        self.reset()

    @property
    def channel(self):
        return self.bus.channel

    def reset(self):
        self.sent = {}
        self.feedback = {}

    def send(self, can_id, data):
        from onero_sdk.protocol import decode_mit
        self.bus.send(can_id,data)
        if can_id in self.ids and len(data)==8 and bytes(data[:7]) != b"\xff"*7:
            index = self.ids[can_id]
            q,v,kp,kd,tau = decode_mit(data,self.limits[index])
            self.sent[index] = {"position_raw":q,"velocity":v,"kp":kp,"kd":kd,
                                "torque_ff":tau,"data_hex":bytes(data).hex()}

    def observe(self, frame):
        from onero_sdk.protocol import decode_feedback
        if frame is not None and frame.arbitration_id in self.responses:
            index = self.responses[frame.arbitration_id]
            self.feedback[index] = decode_feedback(frame.data,self.limits[index])
        return frame

    def recv(self, timeout=.01):
        return self.observe(self.bus.recv(timeout=timeout))

    def recv_until(self, target_id, timeout=.05):
        return self.observe(self.bus.recv_until(target_id,timeout=timeout))

    def recv_batch(self, target_ids, timeout=.05):
        frames = self.bus.recv_batch(target_ids,timeout=timeout)
        for frame in frames.values(): self.observe(frame)
        return frames

    def require_feedback(self, *, allow_missing=False):
        faults = {i+1:state.error for i,state in self.feedback.items() if state.error != 1}
        if faults:
            detail = ", ".join(f"J{i}=0x{code:X}" for i,code in faults.items())
            raise RuntimeError(f"电机反馈状态异常：{detail}；先按原流程恢复机械臂状态")
        missing = sorted(i+1 for i in set(range(7))-set(self.feedback))
        if missing and not allow_missing:
            raise RuntimeError(f"未取得七关节新反馈，缺少关节 {missing}；本轮对照停止推进")
        return missing

    def flush_rx(self): self.bus.flush_rx()
    def close(self): self.bus.close()


def execute_bundle(bundle,args):
    from onero_sdk import Arm
    from onero_sdk.can_bus import CanBus
    from onero_sdk.model_registry import get_model
    from onero_sdk.safety import SafetyConfig
    from onero_sdk.protocol import encode_mit, decode_mit
    from onero_a1_bringup.can_ownership import ChannelOwnership

    for domain in {bundle["ros_domain_id"],int(os.environ.get("ROS_DOMAIN_ID","0"))}:
        refuse_running_bridge(domain)
    settings = bundle["settings"]
    period = 1/settings["control_rate"]
    model = get_model(settings["model"])
    report = {"kind":"onero_sdk_comparison_result","bundle_sha256":bundle["bundle_sha256"],
              "settings":settings,"phase":bundle["phase"],"samples":[],"status":"failed",
              "execution_completed":False,"source_metrics":bundle["source_metrics"]}
    arm = None
    bus = None
    hold_target = None
    finalizer = None
    holding_started = False
    hold_error = None
    hold_events = deque(maxlen=1000)
    hold_event_count = 0
    def hold_event(event, **details):
        nonlocal hold_event_count
        hold_event_count += 1
        hold_events.append({"time_unix_s":time.time(),"stage":"post_record_hold",
                            "event":event,**details})
    with ChannelOwnership(settings["can_channel"]):
        try:
            bus = ObservedBus(CanBus(settings["can_channel"]),model)
            # Public SDK path, without ROS, JTC, SDKArmSession, or private SDK hooks.
            arm = Arm(settings["model"],can_bus=bus,mounting=settings["mounting"],
                      with_gripper=settings["with_gripper"],
                      safety=SafetyConfig(control_freq=settings["control_rate"],
                                          tracking_error_hard_threshold=0.0,
                                          tracking_error_soft_threshold=0.0))
            offsets = arm.get_zero_offset()
            if max(abs(a-b) for a,b in zip(offsets,bundle["zero_offsets"]))>1e-9:
                raise ValueError("SDK 直连实际加载的零位与 A 轮不同，拒绝执行；不自动修改零位")
            if settings["gravity_compensation"]:
                vector(arm.gravity_torque([0.0]*7))
            bus.reset()
            initial = arm.get_state()
            bus.require_feedback()  # Only hand over an already enabled arm.
            if max(map(abs,vector(initial.velocities)))>.02:
                raise ValueError("起点仍在运动，拒绝接管")
            report["start_check"] = check_start(bundle,initial.joints)
            report["initial_feedback"] = list(initial.joints)
            bus.reset()
            arm.enable()  # SDK's all-enabled path preserves holding torque.
            bus.reset()
            initial = arm.get_state()
            bus.require_feedback()
            check_start(bundle,initial.joints)
            if max(map(abs,vector(initial.velocities)))>.02:
                raise ValueError("接管期间关节仍在运动")
            hold_target = list(initial.joints)
            print("起点检查通过，开始 SDK 对照。Ctrl+C 会中断记录并进入保持。",flush=True)
            anchor = time.monotonic()
            try:
                for command in bundle["samples"]:
                    deadline = anchor+command["time_s"]
                    delay = deadline-time.monotonic()
                    if delay>0: time.sleep(delay)
                    started = time.monotonic()
                    if started-deadline>2*period:
                        raise RuntimeError("控制调度落后超过两个周期，不追赶旧命令")
                    requested_gravity = vector(arm.gravity_torque()) if settings["gravity_compensation"] else [0.0]*7
                    kwargs = {} if settings["gravity_compensation"] else {"tau_ff":[0.0]*7}
                    bus.reset()
                    state = arm.servoj(command["positions"],kp=settings["kp"],kd=settings["kd"],
                                       velocities=command["velocities"],**kwargs)
                    completed = time.monotonic()
                    bus.require_feedback()
                    vector(state.joints)
                    if set(bus.sent)!=set(range(7)):
                        raise RuntimeError("未观察到完整七关节 MIT 命令")
                    wires = [deepcopy(bus.sent[i]) for i in range(7)]
                    for wire,offset in zip(wires,offsets):wire["position"] = wire["position_raw"]-offset
                    mismatch = []
                    for i,wire in enumerate(wires):
                        expected = decode_mit(encode_mit(
                            command["positions"][i]+offsets[i],command["velocities"][i],
                            settings["kp"],settings["kd"],0.0,bus.limits[i]),bus.limits[i])
                        if any(abs(wire[key]-value)>1e-10 for key,value in zip(
                                ("position_raw","velocity","kp","kd"),expected[:4])):
                            mismatch.append(i+1)
                    row = {"time_s":command["time_s"],"actual_time_s":started-anchor,
                           "lateness_s":started-deadline,"io_s":completed-started,
                           "positions":command["positions"],"velocities":command["velocities"],
                           "feedback_position":list(state.joints),"feedback_velocity":list(state.velocities),
                           "feedback_torque":list(state.torques),"requested_gravity":requested_gravity,
                           "wire_commands":wires,"wire_reference_mismatch":mismatch}
                    report["samples"].append(row)
                    hold_target = list(state.joints)
                    if mismatch:
                        raise RuntimeError(f"SDK 实际发送的参考值与 A 轮不一致，关节 {mismatch}")
                    if max(abs(a-b) for a,b in zip(command["positions"],state.joints))>.20:
                        raise RuntimeError("关节跟踪误差超过 0.20 rad，停止推进并保持")
                hold_target = list(bundle["samples"][-1]["positions"])
                report["execution_completed"] = True
            except (Exception,KeyboardInterrupt) as exc:
                report["error"] = str(exc) or "用户中断"
            def finalize():
                try:
                    report["metrics"] = metrics(bundle,report["samples"])
                    if report["execution_completed"]:
                        final = report["metrics"]["final_fk"]
                        report["status"] = ("passed" if final["position_m"]<=bundle["position_tolerance_m"]
                                            and final["orientation_rad"]<=bundle["orientation_tolerance_rad"] else "precision_failed")
                    write_new(args.output,report)
                    print(f"结果={report['status']}；报告={args.output.resolve()}",flush=True)
                    if report["metrics"]:
                        print(f"B 轮末端误差={report['metrics']['final_fk']['position_m']*1000:.2f} mm",flush=True)
                    print("程序继续刷新保持目标。按现场流程支撑好机械臂后再 Ctrl+C 退出；退出不发送失能指令。",flush=True)
                except Exception as exc:
                    report["status"] = "failed"
                    report["report_write_error"] = str(exc)
                    print(f"报告保存失败，仍在保持：{exc}",file=sys.stderr,flush=True)
            # Serialize/FK outside the control loop, without a gap in gravity refresh.
            finalizer = threading.Thread(target=finalize,daemon=True)
            finalizer.start()
            # A failed/finished test must not silently stop updating gravity.
            holding_started = True
            missing_before = []
            while True:
                tick = time.monotonic()
                # Strict fresh feedback remains mandatory for the recorded
                # comparison above. During subsequent hold, tolerate a transient
                # missing reply while refreshing the SAME target at zero speed.
                # SDK record_response() still enforces its consecutive-loss
                # watchdog; hardware faults and SDK exceptions are never hidden.
                missing = bus.require_feedback(allow_missing=True)
                if missing != missing_before:
                    hold_event("incomplete_feedback" if missing else "feedback_recovered",
                               missing_joints=missing)
                    if missing:
                        print(f"保持阶段反馈暂缺关节 {missing}，继续刷新原保持目标；"
                              "SDK 连续丢帧保护仍生效。",file=sys.stderr,flush=True)
                    missing_before = missing
                bus.reset()
                kwargs = {} if settings["gravity_compensation"] else {"tau_ff":[0.0]*7}
                arm.servoj(hold_target,kp=settings["kp"],kd=settings["kd"],velocities=[0.0]*7,**kwargs)
                time.sleep(max(0.0,period-(time.monotonic()-tick)))
        except KeyboardInterrupt:
            if holding_started:hold_event("operator_exit")
            print("已结束 SDK 对照保持。")
        except Exception as exc:
            if holding_started:
                hold_error = str(exc)
                hold_event("holding_stopped",error=hold_error,
                           missing_joints=sorted(i+1 for i in set(range(7))-set(bus.feedback)),
                           motor_errors={str(i+1):s.error for i,s in bus.feedback.items()})
                label = "对照采集已完成" if report["execution_completed"] else "对照采集已结束"
                print(f"{label}；后续保持异常停止：{exc}。原报告保留，保持事件另存。",
                      file=sys.stderr,flush=True)
            else:
                report["error"] = str(exc)
                if finalizer is not None:finalizer.join()
                if not args.output.exists():write_new(args.output,report)
                raise
        finally:
            try:
                if arm is not None: arm.close(disable=False)
                elif bus is not None: bus.close()
            finally:
                if finalizer is not None:finalizer.join()
                if holding_started:
                    event_path = args.output.with_name(args.output.stem+".hold_events.json")
                    try:
                        write_new(event_path,{"bundle_sha256":bundle["bundle_sha256"],
                                              "recording_completed":report["execution_completed"],
                                              "holding_error":hold_error,"events":list(hold_events),
                                              "discarded_events":hold_event_count-len(hold_events)})
                    except Exception as exc:
                        print(f"保持事件保存失败：{exc}",file=sys.stderr)
    if hold_error is not None:return 2
    return 0 if report["status"]=="passed" else 1


def compare(args):
    bundle = validate_bundle(json.loads(args.bundle.read_text()))
    result = json.loads(args.result.read_text())
    if result.get("bundle_sha256") != bundle["bundle_sha256"] or result.get("settings") != bundle["settings"]:
        raise ValueError("两轮不是同一组命令和配置")
    if not result.get("execution_completed") or not result.get("metrics"):
        raise ValueError("SDK 对照没有完整执行；查看结果中的 error")
    if len(result.get("samples",[])) != len(bundle["samples"]):
        raise ValueError("B 轮采样数量与 A 轮不同")
    for a,b in zip(bundle["samples"],result["samples"]):
        if (a["time_s"] != b["time_s"] or a["positions"] != b["positions"]
                or a["velocities"] != b["velocities"] or b.get("wire_reference_mismatch")):
            raise ValueError("B 轮时间/位置/速度序列或线缆参考值与 A 轮不同")
    start = check_start(bundle,result["initial_feedback"])
    a,b = bundle["source_metrics"],result["metrics"]
    print(f"同一命令序列、增益和起点检查通过，phase={bundle['phase']}")
    print(f"两轮起点最大关节差 {start['max_joint_difference_rad']:.6f} rad，法兰差 {start['flange_difference_m']*1000:.2f} mm")
    print(f"MoveIt A：最后 3 秒平均误差 {a['last_3s_fk_mean_m']*1000:.2f} mm")
    print(f"SDK    B：最后 3 秒平均误差 {b['last_3s_fk_mean_m']*1000:.2f} mm")
    for i in (0,3):
        print(f"J{i+1} 最终角度误差 A={a['final_joint_error_rad'][i]:+.6f}, B={b['final_joint_error_rad'][i]:+.6f} rad")
    if bundle["phase"] == "hold":
        print("结果为 Link7 反馈 FK；静止段对照用于定位持位问题，不评价动态轨迹精度。")
    else:
        print("以上为完整运动后的 Link7 反馈 FK 到位误差；运动过程数据保存在两轮记录中。")
    event_path = args.result.with_name(args.result.stem+".hold_events.json")
    if event_path.exists():
        events = json.loads(event_path.read_text())
        if events.get("holding_error"):
            print(f"采集完成后保持曾异常停止：{events['holding_error']}；详情：{event_path}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="operation",required=True)
    export = sub.add_parser("export",help="只读检查现有 MoveIt 场景并导出 A 轮命令")
    export.add_argument("--from-report",type=Path,required=True)
    export.add_argument("--trace",type=Path,required=True)
    export.add_argument("--output",type=Path,required=True)
    export.add_argument("--phase",choices=["hold","motion"],default="hold")
    export.add_argument("--hold-seconds",type=float,default=10)
    export.add_argument("--environment-file",type=Path,default=Path("src/onero_a1_moveit_config/config/environment.desktop.yaml"))
    prepare = sub.add_parser("prepare",help="通过 MoveIt 返回对照包内 A 轮实测起点；默认仅规划")
    prepare.add_argument("--bundle",type=Path,required=True)
    prepare.add_argument("--output",type=Path)
    prepare.add_argument("--execute",action="store_true")
    run = sub.add_parser("run",help="默认只预览；--execute 才连接 CAN")
    run.add_argument("--bundle",type=Path,required=True)
    run.add_argument("--output",type=Path)
    run.add_argument("--execute",action="store_true")
    comparison = sub.add_parser("compare")
    comparison.add_argument("--bundle",type=Path,required=True)
    comparison.add_argument("--result",type=Path,required=True)
    args = parser.parse_args(argv)
    try:
        if args.operation=="export":
            if not math.isfinite(args.hold_seconds) or not 3<=args.hold_seconds<=30:
                raise ValueError("hold-seconds 必须在 3 到 30 秒之间")
            if args.output.exists():raise ValueError("输出文件已存在")
            export_bundle(args)
        elif args.operation=="prepare":return prepare_bundle(args)
        elif args.operation=="compare":compare(args)
        else:
            bundle = validate_bundle(json.loads(args.bundle.read_text()))
            check_files(bundle)
            show_bundle(bundle)
            if args.execute:
                if (args.output is None or args.output.exists()
                        or args.output.with_name(args.output.stem+".hold_events.json").exists()):
                    raise ValueError("执行需要指定尚不存在的 --output")
                return execute_bundle(bundle,args)
            print("仅预览：没有连接 CAN。执行前停止其他 CAN 控制程序，保持场景和起点不变。")
        return 0
    except Exception as exc:
        print(f"对照失败：{exc}",file=sys.stderr)
        return 1


if __name__=="__main__":
    raise SystemExit(main())
