"""Translate fresh preview values into a bounded candidate, without ROS or I/O."""
import math
from collections import deque


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def fresh_frame(stamp_ns, now_ns, max_age_s):
    """Check an image's ROS timestamp, including at the point of use."""
    if (not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or not 0 < stamp_ns < 2**63
            or not isinstance(now_ns, int) or isinstance(now_ns, bool) or not 0 < now_ns < 2**63
            or not finite(max_age_s) or max_age_s <= 0):
        return False
    age = (now_ns - stamp_ns) / 1_000_000_000
    return -.2 <= age <= max_age_s


class SteeringCandidate:
    def __init__(self, speed, max_angular, timeout, sign=1, enabled=False, max_frame_age=None,
                 window_seconds=0.0, window_samples=1):
        if (not all(finite(v) and v > 0 for v in (speed, max_angular, timeout))
                or sign not in (-1, 1) or (max_frame_age is not None and
                    (not finite(max_frame_age) or max_frame_age <= 0))
                or not finite(window_seconds) or not 0 <= window_seconds <= .5
                or not isinstance(window_samples, int) or isinstance(window_samples, bool)
                or not 1 <= window_samples <= 5):
            raise ValueError('candidate limits must be positive and sign must be -1 or 1')
        self.speed, self.max_angular, self.timeout = float(speed), float(max_angular), float(timeout)
        self.sign, self.enabled = sign, enabled is True
        self.max_frame_age = None if max_frame_age is None else float(max_frame_age)
        self.window_seconds = float(window_seconds)
        self.samples = deque(maxlen=window_samples)
        self.steering, self.received_at = None, None
        self.frame_stamp_ns = None
        self.latest_stamp_ns = None

    def _clear(self):
        self.steering, self.received_at, self.frame_stamp_ns = None, None, None
        self.samples.clear()

    def update(self, value, now, ros_now_ns=None):
        if (not finite(now) or not isinstance(value, dict) or value.get('source') != 'steering_cnn'
                or value.get('valid') is not True or not finite(value.get('steering'))
                or (self.max_frame_age is not None and
                    (not isinstance(value.get('image_stamp_ns'), int) or isinstance(value.get('image_stamp_ns'), bool)
                     or not 0 < value.get('image_stamp_ns') < 2**63))):
            self._clear()
            return
        stamp = value.get('image_stamp_ns')
        # Replayed or reordered predictions cannot refresh the watchdog.
        if self.max_frame_age is not None and self.latest_stamp_ns is not None and stamp <= self.latest_stamp_ns:
            return
        if (self.max_frame_age is not None and ros_now_ns is not None
                and not fresh_frame(stamp, ros_now_ns, self.max_frame_age)):
            self._clear()
            return
        if self.received_at is not None and now < self.received_at:
            self._clear()
            return
        self.steering, self.received_at = float(value['steering']), float(now)
        self.frame_stamp_ns = stamp
        if self.max_frame_age is not None:
            self.latest_stamp_ns = stamp
        self.samples.append((float(now), stamp, self.steering))

    def output(self, now, ros_now_ns=None):
        if not self.enabled:
            return None
        if not finite(now) or self.received_at is None or not 0 <= now - self.received_at <= self.timeout:
            self._clear()
            return (0.0, 0.0, 0.0)
        if self.max_frame_age is not None and not fresh_frame(self.frame_stamp_ns, ros_now_ns, self.max_frame_age):
            self._clear()
            return (0.0, 0.0, 0.0)
        relevant = [item for item in self.samples
                    if 0 <= now - item[0] <= self.window_seconds
                    and (self.max_frame_age is None or fresh_frame(item[1], ros_now_ns, self.max_frame_age))]
        if not relevant:
            relevant = [(self.received_at, self.frame_stamp_ns, self.steering)]
        weight = sum(range(1, len(relevant) + 1))
        smoothed = sum(i * item[2] for i, item in enumerate(relevant, 1)) / weight
        angular = max(-self.max_angular, min(self.max_angular, self.sign * smoothed))
        return (self.speed, 0.0, angular)
