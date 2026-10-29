#!/usr/bin/env python3
"""Bounded, supervised in-place yaw check; never starts lane following."""
import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from automation.base_protocol import FrameDecoder, measured_velocity, velocity_frame


def run_check(port, yaw_rate=.3, duration=1.5, clock=time.monotonic, pause=time.sleep):
    if (not math.isfinite(yaw_rate) or not .1 <= abs(yaw_rate) <= .3
            or not math.isfinite(duration) or not 0 < duration <= 1.5):
        raise ValueError('turn check exceeds the yaw or duration limit')
    decoder = FrameDecoder()
    latest = received = previous = None
    yaw = peak = 0.
    samples = commands = 0
    result = {'physical_motor_test': True, 'test': 'supervised_in_place_turn',
              'yaw_rate_rps': yaw_rate, 'duration_s': duration, 'stop_feedback_confirmed': False}

    def send(w):
        nonlocal commands
        data = velocity_frame(0., 0., w)
        if port.write(data) != len(data):
            raise RuntimeError('short serial write')
        if w: commands += 1

    def receive():
        nonlocal latest, received, previous, yaw, peak, samples
        changed = False
        for frame in decoder.feed(port.read(min(port.in_waiting, 8192))):
            try: value = measured_velocity(frame)
            except ValueError: continue
            now = clock()
            if previous is not None and 0 <= now-previous <= .25:
                yaw += value[2] * (now-previous)
            previous = received = now
            latest = value
            peak = max(peak, abs(value[2]))
            samples += 1
            changed = True
            if math.hypot(value[0], value[1]) > .04 or abs(value[2]) > .7:
                raise RuntimeError('turn feedback exceeded the supervised limit')
        return changed

    def fresh(): return received is not None and 0 <= clock()-received <= .25
    def stopped(): return latest is not None and math.hypot(*latest[:2]) <= .01 and abs(latest[2]) <= .04

    try:
        started = clock(); sent = -float('inf'); stable = None
        while clock()-started < 4:
            if clock()-sent >= .1: send(0.); sent = clock()
            receive()
            if samples >= 5 and fresh() and stopped():
                if stable is None: stable = clock()
                if clock()-stable >= .5: break
            else: stable = None
            pause(.005)
        if stable is None or clock()-stable < .5:
            raise RuntimeError('fresh stationary feedback required')
        yaw = peak = 0.
        started = clock(); sent = -float('inf')
        while clock()-started < duration:
            receive()
            if not fresh(): raise RuntimeError('turn feedback lost')
            if latest[2] * (1 if yaw_rate > 0 else -1) < -.05:
                raise RuntimeError('turn feedback has the opposite sign')
            if clock()-sent >= .1: send(yaw_rate); sent = clock()
            pause(.005)
        result['motion_elapsed_s'] = round(clock()-started, 3)
        started = clock(); sent = -float('inf'); clear = 0
        while clock()-started < 1.2:
            if clock()-sent >= .1: send(0.); sent = clock()
            changed = receive()
            if not fresh(): raise RuntimeError('feedback lost while stopping turn')
            if changed: clear = clear+1 if stopped() else 0
            if clear >= 3:
                result.update(stop_feedback_confirmed=True, stop_feedback_delay_s=round(clock()-started, 3))
                break
            pause(.005)
        if not result['stop_feedback_confirmed']: raise RuntimeError('turn stop feedback not confirmed')
        if peak < .05: result['inconclusive'] = 'yaw movement not observed in wheel feedback'
    except Exception as error:
        result['error'] = str(error)
    finally:
        for _ in range(3):
            try: send(0.)
            except Exception: pass
            pause(.03)
        result.update(samples=samples, nonzero_commands=commands,
                      peak_yaw_feedback_rps=round(peak, 4), integrated_yaw_rad=round(yaw, 4))
    return result


def main():
    p = argparse.ArgumentParser(description='现场看护的原地小角度转向和停车检查')
    p.add_argument('--device', required=True)
    p.add_argument('--yaw-rate', type=float, default=.3)
    p.add_argument('--duration', type=float, default=1.5)
    p.add_argument('--operator-at-power-switch', action='store_true')
    p.add_argument('--clear-rotation-area', action='store_true')
    p.add_argument('--run-motion-test', action='store_true')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not (args.operator_at_power_switch and args.clear_rotation_area and args.run_motion_test):
        p.error('需现场看护、车身转动范围留空并明确启用运动检查')
    if args.output.exists(): p.error('输出记录已经存在')
    import serial
    with serial.Serial(args.device, 115200, timeout=0, write_timeout=.2, exclusive=True) as port:
        result = run_check(port, args.yaw_rate, args.duration)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as out: json.dump(result, out, ensure_ascii=False, indent=2); out.write('\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)
    if 'error' in result: raise SystemExit(1)


if __name__ == '__main__': main()
