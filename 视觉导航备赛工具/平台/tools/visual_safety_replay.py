#!/usr/bin/env python3
"""Synthetic end-to-end perception/control replay; no ROS or motor devices."""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from automation.barrier_avoidance import BarrierAvoider, ZERO
from automation.barrier_detection import detect_barriers
from automation.command import CommandGate
from automation.feedback import EmergencyLatch, FeedbackGuard
from automation.model_crosswalk import ModelCrosswalk
from automation.race import Race
from automation.range_safety import RangeSafety
from automation.supervisor import MotionSupervisor


def config(name):
    value = json.loads((ROOT/'data'/name).read_text(encoding='utf-8'))
    value['calibrated'] = True  # Simulation only; example files are never changed.
    return value


def image(board=None):
    pixels = np.full((240,320,3),130,dtype=np.uint8)
    pixels[:,:60] = (0,60,220)
    pixels[:,260:] = (0,60,220)
    pixels[:,60:68] = (250,220,0)
    pixels[:,252:260] = (250,220,0)
    if board == 'avoid': pixels[70:170,75:150] = (0,55,210)
    if board == 'near': pixels[70:230,75:150] = (0,55,210)
    return pixels


def run():
    barrier_config, avoid_config = config('barrier.example.json'), config('avoidance.example.json')
    model_config, range_config = config('crosswalk-model.example.json'), config('range-safety.example.json')
    race, model = Race(frames=3), ModelCrosswalk(model_config)
    avoider, distance = BarrierAvoider(avoid_config), RangeSafety(range_config)
    gate = CommandGate(.05,.4,.5,enabled=True,max_frame_age=.4)
    feedback, emergency = FeedbackGuard(), EmergencyLatch(.5)
    supervisor = MotionSupervisor(gate, feedback, emergency)
    checks = {}
    tick = 0

    def step(line=False, crossing=False, board=None, range_m=1.0):
        nonlocal tick
        tick += 1
        now, stamp = tick/10, 10_000_000_000 + tick*100_000_000
        race.frame(now,line,None)
        race.odom(now,0,0,0)
        detections = ([{'label':'crossing','score':.9,'box':[100,100,500,400]}]
                      if crossing else [])
        msg = {'source':'openvino_race','valid':True,'stamp_ns':stamp,'width':640,'height':480,
               'inference_ms':10,'detections':detections}
        seen = model.update(msg,now,stamp)
        if seen is not None:
            race.crosswalk_frame(now,seen)
        result = detect_barriers(image(board),barrier_config)
        result.update({'source':'barrier_detector','stamp_ns':stamp})
        avoider.set_observation(result,now,stamp)
        avoider.set_lane(.05,0,0,stamp,now,stamp)
        candidate, candidate_stamp, why = avoider.output(now,stamp)
        distance.update(range_m,.05,2,stamp,now,stamp)
        gate.set_state(dict(race.status(now),calibrated=True),now)
        gate.set_safety(distance.allowed(now,stamp),now)
        gate.set_command(*candidate,now,candidate_stamp,stamp)
        emergency.update(False,now)
        feedback.update(0,0,0,stamp,now,stamp)
        return supervisor.output(now,stamp),why,now,stamp

    for _ in range(3): step()
    checks['initial_reset'] = supervisor.reset(tick/10,10_000_000_000+tick*100_000_000)[0]
    race.arm(tick/10)
    for _ in range(3): out, _, _, _ = step(line=True)
    checks['start_and_motion'] = race.phase == 'running' and out[0] > 0
    for _ in range(3): out, _, _, _ = step(crossing=True)
    checks['model_crosswalk_stop'] = race.phase == 'yield' and out == ZERO
    race.acknowledge(tick/10,race.announcement_id,True)
    for _ in range(33): out, _, _, _ = step(crossing=True)
    checks['three_second_stop_complete'] = race.phase == 'running' and race.yields == 1
    for _ in range(3): step()
    out, why, _, _ = step(board='avoid')
    checks['barrier_candidate'] = why == 'avoid_barrier' and 0 < out[0] <= .04 and out[2] < 0
    out, why, _, _ = step(board='near')
    checks['barrier_near_stop'] = why == 'barrier_too_close' and out == ZERO
    for _ in range(3): step()
    out, _, _, _ = step(range_m=.2)
    checks['independent_range_stop'] = out == ZERO
    for _ in range(3): step()
    for _ in range(3): step(line=True)
    checks['first_lap'] = race.laps == 1
    for _ in range(3): step()
    for _ in range(3): out, _, _, _ = step(crossing=True)
    checks['second_crosswalk_stop'] = race.phase == 'yield' and out == ZERO
    race.acknowledge(tick/10,race.announcement_id,True)
    for _ in range(33): step(crossing=True)
    for _ in range(3): step()
    for _ in range(3): out, _, now, stamp = step(line=True)
    checks['finish_two_laps'] = race.phase == 'finished' and race.laps == 2 and race.yields == 2 and out == ZERO
    return {'all_passed':all(checks.values()), 'checks':checks,
            'scope':'synthetic images, model results, range, odometry and acknowledgements',
            'ros_or_physical_motor_commands':False}


if __name__ == '__main__':
    result = run()
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if not result['all_passed']:
        raise SystemExit(1)
