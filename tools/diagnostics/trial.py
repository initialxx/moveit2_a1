"""Record a Cartesian line for strict replay/precision checks; no CAN is opened here."""
import argparse
from copy import deepcopy
import math
from pathlib import Path
import sys
from rosidl_runtime_py.convert import message_to_ordereddict
from .client import DiagnosticError, diagnostic_options, run_diagnostic


def record_line(node):
    current = node.fk(node.report["start"])
    target = deepcopy(current)
    delta = {axis: getattr(node.args, "d"+axis) for axis in "xyz"}
    for axis, value in delta.items():
        setattr(target.position, axis, getattr(target.position, axis)+value)
    node.report.update(start_pose=message_to_ordereddict(current),
                       pose_target=message_to_ordereddict(target), cartesian_shape="line",
                       requested_translation_xyz_m=delta,
                       cartesian_waypoints=[message_to_ordereddict(target)])
    response = node.cartesian_path(node.report["start"], [target])
    node.report["cartesian_fraction"] = response.fraction
    if response.error_code.val != 1 or response.fraction < 1.-1e-6:
        raise DiagnosticError(f"笛卡尔路径不完整：fraction={response.fraction}，code={response.error_code.val}")
    node.validate_plan(response.solution)
    if node.args.execute:
        node.execute(response.solution)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=["plan", "prepare", "replay"])
    parser.add_argument("--from-report", type=Path)
    parser.add_argument("--dx", type=float, default=.01)
    parser.add_argument("--dy", type=float, default=0.)
    parser.add_argument("--dz", type=float, default=-.01)
    options = diagnostic_options(parser, argv)
    options.case = {"plan": "cartesian", "prepare": "prepare_trial", "replay": "replay_trial"}[options.operation]
    if any(not math.isfinite(getattr(options, "d"+axis)) for axis in "xyz"):
        parser.error("位移必须为有限数值")
    supplied = list(argv) if argv is not None else sys.argv[1:]
    if options.operation == "plan":
        if options.from_report is not None:
            parser.error("plan 不接受 --from-report")
        scenario = record_line
    else:
        if options.from_report is None:
            parser.error("prepare/replay 必须指定 --from-report")
        flags = ["--dx", "--dy", "--dz", "--frame"]
        if options.operation == "replay":
            flags += ["--velocity", "--acceleration"]
        if any(token == flag or token.startswith(flag+"=") for token in supplied for flag in flags):
            parser.error("此操作从报告读取目标/参考系，replay 还保留原轨迹时间参数")
        scenario = lambda node: node.saved_trial(node.report["start"])
    return run_diagnostic(options, scenario)


if __name__ == "__main__":
    raise SystemExit(main())
