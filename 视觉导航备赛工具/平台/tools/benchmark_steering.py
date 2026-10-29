#!/usr/bin/env python3
"""Replay a labelled image sequence without ROS or motor access."""
import argparse
import csv
import importlib.util
import json
import math
import statistics
import time
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('steering_preview', ROOT / 'ros2' / 'steering_preview.py')
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def percentile(values, quantile):
    ordered = sorted(values)
    return ordered[max(0, math.ceil(quantile * len(ordered)) - 1)]


def main():
    parser = argparse.ArgumentParser(description='离线回放并统计循迹模型误差与处理耗时，不连接小车')
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--manifest', default='test.csv', choices=('train.csv', 'validation.csv', 'test.csv'))
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.threads <= 8:
        parser.error('线程数须为 1–8')
    root = args.dataset.expanduser().resolve()
    model_path = args.model.expanduser().resolve()
    output = args.output.expanduser().absolute() if args.output else None
    if output and output.exists():
        parser.error('输出文件已存在')
    with (root / args.manifest).open(encoding='utf-8-sig', newline='') as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames or not {'image_path', 'steering'}.issubset(reader.fieldnames):
            parser.error('清单缺少 image_path 或 steering 字段')
        rows = list(reader)
    if not rows:
        parser.error('清单没有样本')
    torch.set_num_threads(args.threads)
    model = preview.load_model(model_path)
    first = (root / rows[0]['image_path']).resolve()
    if root not in first.parents or not first.is_file():
        parser.error('图片路径无效')
    sample = first.read_bytes()
    for _ in range(5):
        preview.predict_jpeg(model, sample)
    model_times, processing_times, errors = [], [], []
    bands = {'abs_lt_0.2': [], 'abs_0.2_to_0.5': [], 'abs_ge_0.5': []}
    strong_count = strong_disagreement = 0
    for row in rows:
        image = (root / row['image_path']).resolve()
        label = float(row['steering'])
        if root not in image.parents or not image.is_file() or not math.isfinite(label):
            raise ValueError('数据集包含无效图片或标签')
        data = image.read_bytes()
        started = time.perf_counter()
        predicted, inference_ms = preview.predict_jpeg(model, data)
        processing_ms = (time.perf_counter() - started) * 1000
        error = abs(predicted - label)
        errors.append(error)
        model_times.append(inference_ms)
        processing_times.append(processing_ms)
        band = 'abs_lt_0.2' if abs(label) < .2 else ('abs_0.2_to_0.5' if abs(label) < .5 else 'abs_ge_0.5')
        bands[band].append(error)
        if abs(label) >= .2:
            strong_count += 1
            strong_disagreement += int(label * predicted <= 0)
    result = {
        'samples': len(rows), 'dataset': str(root), 'manifest': args.manifest,
        'model': str(model_path), 'device': 'local CPU', 'threads': args.threads,
        'mean_absolute_error': statistics.mean(errors),
        'mean_squared_error': statistics.mean(e * e for e in errors),
        'absolute_error_p95': percentile(errors, .95),
        'label_bands': {name: {'samples': len(values), 'mae': statistics.mean(values) if values else None}
                        for name, values in bands.items()},
        'strong_label_samples': strong_count, 'strong_label_sign_disagreements': strong_disagreement,
        'model_only_ms': {'p50': percentile(model_times, .5), 'p95': percentile(model_times, .95)},
        'jpeg_decode_preprocess_and_infer_ms': {'p50': percentile(processing_times, .5),
                                                'p95': percentile(processing_times, .95)},
        'note': 'Offline CPU replay; excludes camera, ROS, network and vehicle response. Same-recording tests do not prove track performance.'
    }
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as handle:
            handle.write(text)
    print(text, end='')


if __name__ == '__main__':
    main()
