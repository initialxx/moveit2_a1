"""记录起始位置：保存所选手臂当前关节位置，供后续回位使用。"""
from .support import run_example


def example(node):
    node.record_startup()
    label = {'left': '左臂', 'right': '右臂', 'both': '双臂'}[node.arm_side]
    print(f'已将{label}当前位置记录为起始位置。')


def main(args=None):
    return run_example('record_startup', __doc__, lambda _: None, example, args,
                       query_only=True, allow_both=True)
