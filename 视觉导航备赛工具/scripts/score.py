#!/usr/bin/env python3
"""Calculate recorded time plus explicitly supplied penalties; no assumed rules."""
import argparse
from decimal import Decimal, InvalidOperation

def nonnegative(value):
    try:
        n = Decimal(value)
        if not n.is_finite() or n < 0:
            raise ValueError
        return n
    except (InvalidOperation, ValueError):
        raise argparse.ArgumentTypeError('必须是非负数字')

def main():
    p = argparse.ArgumentParser(description='计算自测有效时间；罚时请按最新官方任务二自测表逐项核实后输入。')
    p.add_argument('--time', required=True, type=nonnegative, help='两圈实际用时，单位秒')
    p.add_argument('--penalty', type=nonnegative, action='append', default=[], help='每项确认后的罚时秒数，可重复使用')
    a = p.parse_args()
    total = sum(a.penalty, Decimal(0))
    print(f'比赛用时: {a.time} 秒\n罚时合计: {total} 秒\n有效时间: {a.time + total} 秒')
    print('请对照比赛视频和官方自测表复核；本工具不判断失败或罚时是否叠加。')
if __name__ == '__main__':
    main()
