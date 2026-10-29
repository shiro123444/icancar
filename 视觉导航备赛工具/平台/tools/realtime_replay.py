#!/usr/bin/env python3
"""Offline threaded replay of perception and supervised control; no ROS/hardware."""
import argparse
import csv
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import torch
from automation.command import CommandGate
from automation.feedback import EmergencyLatch, FeedbackGuard
from automation.realtime import LatestWorker
from automation.steering_candidate import SteeringCandidate, fresh_frame
from automation.supervisor import MotionSupervisor

spec = importlib.util.spec_from_file_location('steering_preview', ROOT / 'ros2' / 'steering_preview.py')
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def run(model_path, dataset):
    root = dataset.resolve()
    with (root / 'test.csv').open(encoding='utf-8-sig', newline='') as source:
        rows = list(csv.DictReader(source))[:16]
    if not rows:
        raise ValueError('测试集为空')
    images = []
    for row in rows:
        path = (root / row['image_path']).resolve()
        if root not in path.parents or not path.is_file():
            raise ValueError('图片路径无效')
        images.append(path.read_bytes())
    torch.set_num_threads(2)
    model = preview.load_model(model_path.resolve())
    preview.predict_jpeg(model, images[0])  # Warm up before the timed run.

    def infer(frame):
        stamp, image = frame
        steering, inference_ms = preview.predict_jpeg(model, image)
        return {'source':'steering_cnn','valid':True,'steering':steering,
                'image_stamp_ns':stamp,'inference_ms':inference_ms}

    worker = LatestWorker(infer, 5.0)
    candidate = SteeringCandidate(.05, .4, .5, enabled=True, max_frame_age=.4,
                                  window_seconds=.25, window_samples=3)
    gate = CommandGate(.05, .4, .5, enabled=True, max_frame_age=.4)
    feedback, emergency = FeedbackGuard(), EmergencyLatch(.5)
    system = MotionSupervisor(gate, feedback, emergency)
    checks = {'started':False,'safety_stop':True,'safety_resumed':False,
              'estop_stop':True,'estop_held':True,'reset_resumed':False,'stale_stop':True}
    seen = {key:0 for key in checks}
    prediction_age_ms = []
    frame_index = 0
    reset_after_estop = False
    start = next_frame = next_command = time.monotonic()
    worker.start()
    try:
        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= 4.4:
                break
            ros_now = time.time_ns()
            safety_ok = not (1.2 <= elapsed < 1.6)
            estop_active = 2.0 <= elapsed < 2.3
            gate.set_state({'calibrated':True,'phase':'running','action':'follow_lane'}, now)
            gate.set_safety(safety_ok, now)
            emergency.update(estop_active, now)
            feedback.update(0, 0, 0, ros_now, now, ros_now)
            if elapsed < 3.2 and now >= next_frame:
                worker.submit((ros_now, images[frame_index % len(images)]))
                frame_index += 1
                next_frame = now + .1
            result = worker.take()
            if result:
                if result['ok'] and fresh_frame(result['value']['image_stamp_ns'], ros_now, .4):
                    prediction_age_ms.append((ros_now - result['value']['image_stamp_ns']) / 1e6)
                    candidate.update(result['value'], now, ros_now)
                else:
                    candidate.update(None, now, ros_now)
            if 3.5 <= elapsed < 4.4:
                # A stream of recently received but old predictions must not move the car.
                candidate.update({'source':'steering_cnn','valid':True,'steering':.2,
                                  'image_stamp_ns':ros_now - 2_000_000_000}, now, ros_now)
            if elapsed < .2 and emergency.latched:
                system.reset(now, ros_now)
            if 2.7 <= elapsed and not reset_after_estop:
                reset_after_estop = system.reset(now, ros_now)[0]
            if now >= next_command:
                command = candidate.output(now, ros_now)
                if command:
                    gate.set_command(*command, now, candidate.frame_stamp_ns, ros_now)
                next_command = now + .1
            output = system.output(now, ros_now)
            moving = output is not None and math.hypot(output[0], output[1]) > .001
            if .35 <= elapsed < 1.2:
                seen['started'] += 1; checks['started'] |= moving
            elif 1.25 <= elapsed < 1.6:
                seen['safety_stop'] += 1; checks['safety_stop'] &= not moving
            elif 1.7 <= elapsed < 2.0:
                seen['safety_resumed'] += 1; checks['safety_resumed'] |= moving
            elif 2.05 <= elapsed < 2.3:
                seen['estop_stop'] += 1; checks['estop_stop'] &= not moving
            elif 2.35 <= elapsed < 2.7:
                seen['estop_held'] += 1; checks['estop_held'] &= not moving
            elif 2.8 <= elapsed < 3.2:
                seen['reset_resumed'] += 1; checks['reset_resumed'] |= moving
            elif 3.8 <= elapsed < 4.4:
                seen['stale_stop'] += 1; checks['stale_stop'] &= not moving
            time.sleep(.02)
    finally:
        closed = worker.close()
    checks = {key: bool(value and seen[key]) for key, value in checks.items()}
    ages = sorted(prediction_age_ms)
    return {'checks':checks,'all_passed':all(checks.values()),'observations':seen,
            'prediction_age_ms_p50':ages[math.ceil(.5*len(ages))-1] if ages else None,
            'prediction_age_ms_p95':ages[math.ceil(.95*len(ages))-1] if ages else None,
            'worker':worker.stats(),'worker_closed':closed,
            'physical_motor_commands':False,'source':'historical images and simulated feedback',
            'note':'In-process replay, not a ROS 2 or physical-vehicle test.'}


def main():
    p = argparse.ArgumentParser(description='本机异步感知、窗口控制和安全状态回放')
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    if args.output and args.output.exists():
        p.error('输出文件已存在')
    result = run(args.model, args.dataset)
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as target:
            target.write(text)
    print(text, end='')
    if not result['all_passed'] or not result['worker_closed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
