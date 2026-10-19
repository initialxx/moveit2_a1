"""Internal measured-pose archive, with model/hardware identity and motion checks."""
import argparse
import json
import math
from pathlib import Path
from onero_a1_common.constants import JOINTS
from .client import DiagnosticError, diagnostic_options, run_diagnostic
from .trial_records import BRIDGE_IDENTITY


def load_target(path, report):
    saved = json.loads(path.expanduser().read_text())
    if not isinstance(saved, dict) or saved.get("schema_version") != 1 or saved.get("kind") != "onero_a1_joint_pose":
        raise DiagnosticError("不是 saved_pose save 生成的姿态文件")
    if saved.get("model_fingerprint") != report["model_fingerprint"]:
        raise DiagnosticError("保存姿态的模型或规划限位与当前服务不同")
    old, new = saved.get("bridge_settings"), report["bridge_settings"]
    if (old is None) != (new is None) or (old is not None and
            (not isinstance(old, dict) or any(old.get(k) != new.get(k) for k in BRIDGE_IDENTITY))):
        raise DiagnosticError("保存姿态的硬件、安装或夹爪配置与当前服务不同")
    target = saved.get("joint_positions")
    if (not isinstance(target, dict) or set(target) != set(JOINTS)
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in target.values())):
        raise DiagnosticError("保存姿态必须包含七个有限关节角度")
    return target


def operation(node):
    path = node.args.file.expanduser().resolve()
    if node.args.operation == "save":
        path.parent.mkdir(parents=True, exist_ok=True)
        saved = {"schema_version": 1, "kind": "onero_a1_joint_pose", "name": path.stem,
                 "model_fingerprint": node.report["model_fingerprint"],
                 "bridge_settings": node.report["bridge_settings"], "joint_positions": node.report["start"]}
        with path.open("x", encoding="utf-8") as stream:
            json.dump(saved, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        node.report["saved_pose_file"] = str(path)
        print(f"已保存当前实测姿态：{path}")
        return
    target = load_target(path, node.report)
    node.require_valid(target)
    node.report.update(loaded_pose_file=str(path), requested_joint_target=target)
    if max(abs(target[n]-node.report["start"][n]) for n in JOINTS) <= .001:
        node.report["already_at_named_pose"] = True
        print("当前已在保存姿态的 0.001 rad 范围内，无需运动。")
        return
    trajectory = node.plan(node.report["start"], node.joint_constraints(target, .001))
    node.validate_plan(trajectory)
    if node.args.execute:
        node.execute(trajectory)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=["save", "load"])
    parser.add_argument("--file", required=True, type=Path)
    options = diagnostic_options(parser, argv)
    options.case = "save_pose" if options.operation == "save" else "restore_pose"
    if options.operation == "save" and options.execute:
        parser.error("save 只读保存，不接受 --execute")
    if options.output is not None and options.file.expanduser().resolve() == options.output.expanduser().resolve():
        parser.error("姿态文件与诊断报告必须使用不同路径")
    return run_diagnostic(options, operation)


if __name__ == "__main__":
    raise SystemExit(main())
