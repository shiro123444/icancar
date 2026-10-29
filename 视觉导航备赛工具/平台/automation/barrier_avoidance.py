"""Image-space avoidance candidate; never drives a motor directly."""
import math

from automation.steering_candidate import finite, fresh_frame


ZERO = (0.0, 0.0, 0.0)


def validate_config(config):
    if not isinstance(config, dict) or not isinstance(config.get('calibrated'), bool):
        raise ValueError('avoidance calibrated must be true or false')
    for name in ('vehicle_center_x', 'min_gap_width', 'avoid_bottom', 'stop_bottom'):
        if not finite(config.get(name)) or not 0 < config[name] <= 1:
            raise ValueError(name + ' must be in (0,1]')
    if config['avoid_bottom'] >= config['stop_bottom']:
        raise ValueError('stop threshold must be nearer than avoidance threshold')
    if not finite(config.get('padding')) or not 0 <= config['padding'] < .25:
        raise ValueError('padding must be in [0,.25)')
    for name in ('turn_gain', 'max_angular', 'max_speed', 'avoid_speed', 'timeout', 'max_source_age'):
        if not finite(config.get(name)) or config[name] <= 0:
            raise ValueError(name + ' must be positive')
    if (config['max_speed'] > .15 or config['avoid_speed'] > config['max_speed']
            or config['max_angular'] > 1):
        raise ValueError('avoidance limits exceed low-speed test range')
    if config.get('turn_sign') not in (-1, 1) or isinstance(config.get('turn_sign'), bool):
        raise ValueError('turn sign must be -1 or 1')
    if (not isinstance(config.get('clear_frames'), int) or isinstance(config['clear_frames'], bool)
            or not 1 <= config['clear_frames'] <= 20):
        raise ValueError('clear frames must be in 1..20')
    return config


class BarrierAvoider:
    def __init__(self, config):
        self.config = validate_config(config)
        self.lane, self.lane_at, self.lane_stamp = None, None, None
        self.lane_latest_stamp = None
        self.observation, self.observation_at, self.observation_stamp = None, None, None
        self.observation_latest_stamp = None
        self.side, self.clear_count = None, 0

    def set_lane(self, vx, vy, angular, stamp, now, ros_now_ns):
        if (not all(finite(v) for v in (vx, vy, angular, now)) or vx < 0 or abs(vy) > 1e-6
                or not fresh_frame(stamp, ros_now_ns, self.config['max_source_age'])):
            self.lane, self.lane_at, self.lane_stamp = None, None, None
            return False
        if self.lane_latest_stamp is not None and stamp <= self.lane_latest_stamp:
            if vx == vy == angular == 0:
                self.lane, self.lane_at, self.lane_stamp = None, None, None
            return False
        self.lane = (min(float(vx), self.config['max_speed']), 0.0,
                     max(-self.config['max_angular'], min(self.config['max_angular'], float(angular))))
        self.lane_at, self.lane_stamp, self.lane_latest_stamp = float(now), stamp, stamp
        return True

    def set_observation(self, value, now, ros_now_ns):
        if (not finite(now) or not isinstance(value, dict) or value.get('source') != 'barrier_detector'
                or value.get('valid') is not True or value.get('calibrated') is not True
                or not fresh_frame(value.get('stamp_ns'), ros_now_ns, self.config['max_source_age'])):
            self.observation, self.observation_at, self.observation_stamp = None, None, None
            return False
        stamp, lane, boxes = value['stamp_ns'], value.get('lane'), value.get('boxes')
        if self.observation_latest_stamp is not None and stamp <= self.observation_latest_stamp:
            return False
        if (not isinstance(lane, list) or len(lane) != 2 or not all(finite(x) for x in lane)
                or not 0 <= lane[0] < lane[1] <= 1 or not isinstance(boxes, list) or len(boxes) > 100):
            self.observation, self.observation_at, self.observation_stamp = None, None, None
            return False
        for item in boxes:
            box = item.get('box') if isinstance(item, dict) else None
            if (not isinstance(box, list) or len(box) != 4 or not all(finite(x) for x in box)
                    or not (0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1)):
                self.observation, self.observation_at, self.observation_stamp = None, None, None
                return False
        self.observation, self.observation_at, self.observation_stamp = value, float(now), stamp
        self.observation_latest_stamp = stamp
        relevant = [x for x in boxes if x['box'][3] >= self.config['avoid_bottom']]
        if relevant:
            self.clear_count = 0
        else:
            self.clear_count += 1
            if self.clear_count >= self.config['clear_frames']:
                self.side = None
        return True

    def output(self, now, ros_now_ns):
        c = self.config
        if not c['calibrated']:
            return ZERO, None, 'uncalibrated'
        if (not finite(now) or self.lane is None or self.lane_at is None or self.observation_at is None
                or not 0 <= now - self.lane_at <= c['timeout']
                or not 0 <= now - self.observation_at <= c['timeout']
                or not fresh_frame(self.lane_stamp, ros_now_ns, c['max_source_age'])
                or not fresh_frame(self.observation_stamp, ros_now_ns, c['max_source_age'])):
            return ZERO, None, 'input_unavailable'
        stamp = min(self.lane_stamp, self.observation_stamp)
        vx, vy, angular = self.lane
        if vx <= 0:
            return ZERO, stamp, 'lane_stop'
        lane, boxes = self.observation['lane'], self.observation['boxes']
        if not lane[0] < c['vehicle_center_x'] < lane[1]:
            return ZERO, stamp, 'lane_geometry_invalid'
        relevant = [item['box'] for item in boxes if item['box'][3] >= c['avoid_bottom']
                    and item['box'][2] > lane[0] and item['box'][0] < lane[1]]
        if not relevant:
            if self.side is not None and self.clear_count < c['clear_frames']:
                return ZERO, stamp, 'confirming_clear'
            return (vx, vy, angular), stamp, 'follow_lane'
        if any(box[3] >= c['stop_bottom'] for box in relevant):
            return ZERO, stamp, 'barrier_too_close'
        intervals = [(lane[0], lane[1])]
        for box in sorted(relevant):
            blocked_left, blocked_right = max(lane[0], box[0] - c['padding']), min(lane[1], box[2] + c['padding'])
            remaining = []
            for left, right in intervals:
                if blocked_right <= left or blocked_left >= right:
                    remaining.append((left, right))
                else:
                    if blocked_left > left: remaining.append((left, blocked_left))
                    if blocked_right < right: remaining.append((blocked_right, right))
            intervals = remaining
        center = c['vehicle_center_x']
        targets = []
        for left, right in intervals:
            if right-left < c['min_gap_width']:
                continue
            inset = c['min_gap_width'] / 2
            target = max(left + inset, min(right - inset, center))
            side = 'left' if target < center else 'right' if target > center else 'center'
            if self.side is None or side == self.side:
                targets.append((abs(target-center), target, side))
        if not targets:
            return ZERO, stamp, 'no_safe_gap'
        _, target, side = min(targets)
        self.side = side
        turn = max(-c['max_angular'], min(c['max_angular'], c['turn_sign'] * c['turn_gain'] * (target-center)))
        return (min(vx, c['avoid_speed']), 0.0, turn), stamp, 'avoid_barrier'
