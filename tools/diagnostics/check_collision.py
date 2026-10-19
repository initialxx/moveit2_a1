"""Read-only collision regression with a deliberately out-of-bounds folded probe."""
import argparse
from onero_a1_common.constants import JOINTS
from .client import DiagnosticError, diagnostic_options, run_diagnostic


def check(node):
    # This deliberately out-of-bounds folded geometry is a read-only test
    # of the collision detector, NEVER a plan or a hardware motion target.
    safe = dict.fromkeys(JOINTS, 0.0)
    folded = {**safe, "joint4-a1_r": 3.14}
    node.require_valid(safe)
    response = node.validity(folded)
    contacts = [(c.contact_body_1, c.contact_body_2) for c in response.contacts]
    body = {"base_link", *[f"Link{i}" for i in range(1, 8)]}
    self_contacts = [p for p in contacts if p[0] in body and p[1] in body]
    node.report["collision_geometry_query"] = {
        "configuration": folded, "out_of_bounds_test_only": True,
        "valid": response.valid, "self_contacts": self_contacts,
    }
    if response.valid or not self_contacts:
        raise DiagnosticError("未检出折叠几何的自碰撞；检查 SRDF 是否仍屏蔽非相邻连杆")
    print(f"自碰撞查询通过：正常姿态有效，折叠几何返回 {self_contacts}", flush=True)

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    options = diagnostic_options(parser, argv)
    options.case = "collision"
    if options.execute:
        parser.error("只读几何回归禁止 --execute")
    return run_diagnostic(options, check)


if __name__ == "__main__":
    raise SystemExit(main())
