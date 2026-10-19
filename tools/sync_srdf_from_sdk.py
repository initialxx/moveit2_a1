#!/usr/bin/env python3
"""
从 armsdk 的权威 SRDF 生成 MoveIt 用的 SRDF。

机器人名称/关节来自 SDK；MoveIt 的自碰撞策略仅排除 URDF 中的直接相邻连杆。
SDK SRDF 的 Never 排除项不能未经验证直接沿用：A1 源文件曾排除全部连杆对。
非相邻连杆保留检查；几何和关节限位仍以 SDK URDF 为准。

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
  机器人来源: armsdk/onero_sdk/urdf/A1/a1_r.urdf 与 a1_r.srdf
  碰撞策略: 仅排除 URDF 直接相邻连杆，不沿用未验证的 Never 条目
  MoveIt 语义: planning group / 命名位姿 / virtual joint
-->
"""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sdk-srdf", required=True, type=Path)
    ap.add_argument("--sdk-urdf", type=Path, help="默认使用 SDK SRDF 同名 .urdf")
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--robot-name", default="onero_a1")
    ap.add_argument("--base-link", default="base_link")
    ap.add_argument("--tip-link", default="Link7")
    args = ap.parse_args()

    if not args.sdk_srdf.is_file():
        print(f"ERROR: SDK SRDF 不存在: {args.sdk_srdf}", file=sys.stderr)
        return 2

    sdk = ET.parse(args.sdk_srdf).getroot()
    original_pairs = [
        (e.get("link1"), e.get("link2"), e.get("reason"))
        for e in sdk.findall("disable_collisions")
    ]
    if not original_pairs:
        print("ERROR: SDK SRDF 中没有 disable_collisions", file=sys.stderr)
        return 1

    urdf_path = args.sdk_urdf or args.sdk_srdf.with_suffix(".urdf")
    if not urdf_path.is_file():
        print(f"ERROR: SDK URDF 不存在: {urdf_path}", file=sys.stderr)
        return 2
    urdf = ET.parse(urdf_path).getroot()
    pairs = [
        (*sorted((joint.find("parent").get("link"), joint.find("child").get("link"))),
         "Adjacent")
        for joint in urdf.findall("joint")
    ]

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

    # 保守的 MoveIt 碰撞策略：只排除直接装配相邻的连杆。
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
    print(f"  - 碰撞矩阵:   {len(pairs)} 条（URDF 相邻连杆；SDK 原表 {len(original_pairs)} 条）")
    print(f"  - planning group: arm  ({args.base_link} -> {args.tip_link})")
    print(f"  - 命名位姿:   {', '.join(NAMED_POSES)}")
    print(f"  - virtual joint: world -> {args.base_link}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
