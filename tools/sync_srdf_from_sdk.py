#!/usr/bin/env python3
"""
从 armsdk 的权威 SRDF 生成 MoveIt 用的 SRDF。

碰撞矩阵（disable_collisions）直接取自 armsdk，避免手工抄写出错；
本脚本只【追加】MoveIt 需要的语义部分：planning group、命名位姿、virtual joint。

用法：
    python3 sync_srdf_from_sdk.py \
        --sdk-srdf /path/to/armsdk/onero_sdk/urdf/A1/a1_r.srdf \
        --out      /path/to/onero_a1_moveit_config/config/onero_a1.srdf
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# 命名位姿（关节角，单位 rad）
NAMED_POSES = {
    "home": {
        "joint1-a1_r": 0.0, "joint2-a1_r": 0.0, "joint3-a1_r": 0.0,
        "joint4-a1_r": 0.0, "joint5-a1_r": 0.0, "joint6-a1_r": 0.0,
        "joint7-a1_r": 0.0,
    },
    "ready": {
        "joint1-a1_r": 0.0, "joint2-a1_r": -1.0, "joint3-a1_r": 0.0,
        "joint4-a1_r": -1.5, "joint5-a1_r": 0.0, "joint6-a1_r": 0.5,
        "joint7-a1_r": 0.0,
    },
}

HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<!--
  Onero A1 语义机器人描述（SRDF）—— 自动生成，请勿手工编辑

  生成方式: tools/sync_srdf_from_sdk.py
  碰撞矩阵来源: armsdk/onero_sdk/urdf/A1/a1_r.srdf（权威）
  追加内容: planning group / 命名位姿 / virtual joint
-->
"""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sdk-srdf", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--robot-name", default="onero_a1")
    ap.add_argument("--base-link", default="base_link")
    ap.add_argument("--tip-link", default="Link7")
    args = ap.parse_args()

    if not args.sdk_srdf.is_file():
        print(f"ERROR: SDK SRDF 不存在: {args.sdk_srdf}", file=sys.stderr)
        return 2

    sdk = ET.parse(args.sdk_srdf).getroot()
    pairs = [
        (e.get("link1"), e.get("link2"), e.get("reason"))
        for e in sdk.findall("disable_collisions")
    ]
    if not pairs:
        print("ERROR: SDK SRDF 中没有 disable_collisions", file=sys.stderr)
        return 1

    robot = ET.Element("robot", {"name": args.robot_name})

    # planning group：整条臂
    g = ET.SubElement(robot, "group", {"name": "arm"})
    ET.SubElement(g, "chain", {"base_link": args.base_link, "tip_link": args.tip_link})

    # 命名位姿
    for pose_name, joints in NAMED_POSES.items():
        gs = ET.SubElement(robot, "group_state", {"name": pose_name, "group": "arm"})
        for jn, val in joints.items():
            ET.SubElement(gs, "joint", {"name": jn, "value": f"{val}"})

    # virtual joint：把机器人挂到 world 下
    ET.SubElement(
        robot,
        "virtual_joint",
        {
            "name": "world_joint",
            "type": "fixed",
            "parent_frame": "world",
            "child_link": args.base_link,
        },
    )

    # 碰撞矩阵（照搬 SDK）
    for l1, l2, reason in sorted(pairs):
        ET.SubElement(
            robot,
            "disable_collisions",
            {"link1": l1, "link2": l2, "reason": reason or "Never"},
        )

    ET.indent(ET.ElementTree(robot), space="  ")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(HEADER)
        f.write(ET.tostring(robot, encoding="unicode"))
        f.write("\n")

    print(f"✓ 已生成: {args.out}")
    print(f"  - 碰撞矩阵:   {len(pairs)} 条（照搬 SDK）")
    print(f"  - planning group: arm  ({args.base_link} -> {args.tip_link})")
    print(f"  - 命名位姿:   {', '.join(NAMED_POSES)}")
    print(f"  - virtual joint: world -> {args.base_link}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
