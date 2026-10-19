"""碰撞查询：检查当前姿态或 SRDF 命名姿态，打印有效性和碰撞接触对。"""
from .support import run_example


def configure(parser):
    parser.add_argument("--target", help="可选 SRDF 姿态名；默认查询当前实测姿态")


def example(node):
    q = node.named_target(node.args.target) if node.args.target else node.start
    result = node.validity(q)
    print(f"状态有效：{result.valid}")
    for contact in result.contacts:
        print(f"碰撞接触：{contact.contact_body_1} ↔ {contact.contact_body_2}")


def main(args=None):
    return run_example("collision_check", __doc__, configure, example, args, query_only=True)
