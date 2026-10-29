#!/usr/bin/env python3
"""Prepare a dataset if needed, then train locally into a new directory."""
import argparse
import csv
import json
from pathlib import Path

import torch

from training.prepare_recorded_dataset import prepare as prepare_recorded
from training.prepare_vendor_dataset import prepare as prepare_vendor
from training.train_steering import run


def main():
    parser = argparse.ArgumentParser(description='本机数据整理与模型训练')
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--epochs', type=int, default=20)
    args = parser.parse_args()
    source, output = args.source.expanduser().resolve(), args.output.expanduser().absolute()
    if not source.is_dir() or not 1 <= args.epochs <= 200:
        parser.exit(2, '训练失败：数据目录或轮次无效\n')
    if output.exists():
        parser.exit(2, '训练失败：输出目录已存在\n')
    try:
        output.mkdir(parents=True, exist_ok=False)
        print('输出目录：' + str(output), flush=True)
        if (source / 'train.csv').is_file() and (source / 'validation.csv').is_file():
            dataset = source
            print('使用已整理的数据集：' + str(dataset), flush=True)
        elif (source / 'data.csv').is_file():
            with (source / 'data.csv').open(encoding='utf-8-sig', newline='') as handle:
                fields = set(csv.DictReader(handle).fieldnames or [])
            dataset = output / 'dataset'
            if {'image', 'angular_z', 'image_stamp_ns'}.issubset(fields):
                metadata = prepare_recorded(source, dataset)
            elif {'image_path', 'steering'}.issubset(fields):
                metadata = prepare_vendor(source, dataset, test_fraction=0.2)
            else:
                raise ValueError('无法识别采集数据格式')
            print('数据整理完成：' + json.dumps(metadata, ensure_ascii=False), flush=True)
        else:
            raise ValueError('数据目录中没有受支持的 CSV')
        device = 'mps' if torch.backends.mps.is_available() else 'cpu'
        print('训练设备：' + ('Apple GPU' if device == 'mps' else 'CPU'), flush=True)
        run(argparse.Namespace(dataset=dataset, output=output / 'model', epochs=args.epochs,
                               batch_size=64, lr=.001, workers=0, seed=2026, device=device))
        print('模型：' + str(output / 'model' / 'autopilot.pth'), flush=True)
    except (ValueError, OSError, csv.Error) as error:
        parser.exit(2, '训练失败：' + str(error) + '\n')


if __name__ == '__main__':
    main()
