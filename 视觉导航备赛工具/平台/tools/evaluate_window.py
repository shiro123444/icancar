#!/usr/bin/env python3
"""Compare steering windows on timestamped historical frames; no hardware."""
import argparse
import csv
import importlib.util
import json
import math
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import torch
from automation.steering_candidate import SteeringCandidate

spec = importlib.util.spec_from_file_location('steering_preview', ROOT / 'ros2' / 'steering_preview.py')
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def evaluate(model_path, dataset, step, window_ms):
    root = dataset.resolve()
    with (root / 'test.csv').open(encoding='utf-8-sig', newline='') as handle:
        rows = list(csv.DictReader(handle))[::step]
    if len(rows) < 2:
        raise ValueError('测试样本不足')
    torch.set_num_threads(2)
    model = preview.load_model(model_path.resolve())
    names = [('unfiltered', 0), ('window', window_ms / 1000)]
    controllers = {name:SteeringCandidate(.05, 100, .5, enabled=True, max_frame_age=.4,
                                          window_seconds=window, window_samples=3)
                   for name, window in names}
    values = {name:[] for name, _ in names}
    labels, stamps = [], []
    for row in rows:
        image = (root / row['image_path']).resolve()
        if root not in image.parents or not image.is_file() or not image.stem.isdigit():
            raise ValueError('图片路径无效或名称不是毫秒时间戳')
        stamp = int(image.stem) * 1_000_000
        if stamps and stamp <= stamps[-1]:
            raise ValueError('图片时间戳未严格递增')
        label = float(row['steering'])
        if not math.isfinite(label):
            raise ValueError('转向标签无效')
        steering, _ = preview.predict_jpeg(model, image.read_bytes())
        stamps.append(stamp)
        labels.append(label)
        now = (stamp - stamps[0]) / 1e9
        prediction = {'source':'steering_cnn','valid':True,'steering':steering,'image_stamp_ns':stamp}
        for name, controller in controllers.items():
            controller.update(prediction, now, stamp)
            values[name].append(controller.output(now, stamp)[2])
    results = {}
    for name, outputs in values.items():
        errors = sorted(abs(x-y) for x,y in zip(outputs, labels))
        strong = [(x,y) for x,y in zip(outputs, labels) if abs(y) >= .2]
        results[name] = {
            'mse':statistics.mean((x-y)**2 for x,y in zip(outputs, labels)),
            'mae':statistics.mean(errors), 'absolute_error_p95':errors[math.ceil(.95*len(errors))-1],
            'mean_abs_step':statistics.mean(abs(b-a) for a,b in zip(outputs, outputs[1:])),
            'strong_label_samples':len(strong),
            'strong_label_sign_disagreements':sum(x*y <= 0 for x,y in strong),
        }
    intervals = [(b-a)/1e6 for a,b in zip(stamps, stamps[1:])]
    return {'samples':len(rows),'sample_step':step,'median_interval_ms':statistics.median(intervals),
            'window_ms':window_ms,'max_angular_for_analysis':100,'results':results,
            'note':'Same historical recording; label units are unverified. Angular limit 100 isolates smoothing and is not a vehicle setting.'}


def main():
    parser = argparse.ArgumentParser(description='比较历史画面上平滑窗口与原始预测的误差和变化量')
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--step', type=int, default=2)
    parser.add_argument('--window-ms', type=float, default=250)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.step <= 100 or not math.isfinite(args.window_ms) or not 0 <= args.window_ms <= 500:
        parser.error('采样间隔或窗口无效')
    if args.output and args.output.exists():
        parser.error('输出文件已存在')
    result = evaluate(args.model, args.dataset, args.step, args.window_ms)
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as target:
            target.write(text)
    print(text, end='')


if __name__ == '__main__':
    main()
