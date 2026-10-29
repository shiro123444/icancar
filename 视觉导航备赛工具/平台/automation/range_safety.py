"""Fail-closed range interlock; requires a validated independent sensor."""
from automation.steering_candidate import finite, fresh_frame


class RangeSafety:
    def __init__(self, config):
        if not isinstance(config, dict) or not isinstance(config.get('calibrated'), bool):
            raise ValueError('range sensor calibrated must be true or false')
        for name in ('stop_m', 'release_m', 'timeout', 'max_source_age'):
            if not finite(config.get(name)) or config[name] <= 0:
                raise ValueError(name + ' must be positive')
        if config['release_m'] <= config['stop_m']:
            raise ValueError('release distance must exceed stop distance')
        count = config.get('clear_frames')
        if not isinstance(count, int) or isinstance(count, bool) or not 1 <= count <= 20:
            raise ValueError('clear frames must be in 1..20')
        self.config = config
        self.stamp_ns = self.received_at = self.latest_stamp_ns = None
        self.clear_count = 0
        self.clear = False

    def invalidate(self):
        self.stamp_ns, self.received_at, self.clear_count, self.clear = None, None, 0, False

    def update(self, distance, minimum, maximum, stamp, now, ros_now_ns):
        c = self.config
        if (not all(finite(v) for v in (distance, minimum, maximum, now))
                or not 0 <= minimum <= c['stop_m'] < c['release_m'] <= maximum
                or not minimum <= distance <= maximum
                or not fresh_frame(stamp, ros_now_ns, c['max_source_age'])):
            self.invalidate()
            return False
        if self.latest_stamp_ns is not None and stamp <= self.latest_stamp_ns:
            if distance <= c['stop_m']:
                self.clear, self.clear_count = False, 0
            return False
        self.stamp_ns, self.latest_stamp_ns, self.received_at = stamp, stamp, float(now)
        if distance <= c['stop_m']:
            self.clear, self.clear_count = False, 0
        elif distance >= c['release_m']:
            self.clear_count += 1
            if self.clear_count >= c['clear_frames']:
                self.clear = True
        else:
            self.clear_count = 0
        return True

    def allowed(self, now, ros_now_ns):
        c = self.config
        if (not c['calibrated'] or not finite(now) or self.received_at is None
                or not 0 <= now - self.received_at <= c['timeout']
                or not fresh_frame(self.stamp_ns, ros_now_ns, c['max_source_age'])):
            self.clear, self.clear_count = False, 0
            return False
        return self.clear
