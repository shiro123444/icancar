"""Fresh odometry supervision and a separate, explicitly reset stop latch."""
import math
from automation.race import number
from automation.steering_candidate import fresh_frame


class EmergencyLatch:
    def __init__(self, timeout):
        if not number(timeout) or timeout <= 0:
            raise ValueError('emergency timeout must be positive')
        self.timeout = float(timeout)
        self.active = None
        self.received_at = None
        self.latched = True

    def update(self, active, now):
        if not isinstance(active, bool) or not number(now):
            self.active, self.received_at, self.latched = None, None, True
            return
        self.active, self.received_at = active, float(now)
        if active:
            self.latched = True

    def allowed(self, now):
        if (not number(now) or self.received_at is None or
                not 0 <= now - self.received_at <= self.timeout or self.active is not False):
            self.latched = True
        return not self.latched

    def can_reset(self, now):
        return (number(now) and self.received_at is not None and self.active is False
                and 0 <= now - self.received_at <= self.timeout)

    def reset(self, now):
        if not self.can_reset(now):
            return False
        self.latched = False
        return True


class FeedbackGuard:
    def __init__(self, timeout=.5, max_source_age=.5, max_linear=.25, max_angular=1.0,
                 stopped_linear=.02, stopped_angular=.05, stop_grace=1.0):
        values = (timeout, max_source_age, max_linear, max_angular, stopped_linear, stopped_angular, stop_grace)
        if not all(number(x) and x > 0 for x in values) or stopped_linear >= max_linear or stopped_angular >= max_angular:
            raise ValueError('odometry limits must be positive and consistent')
        (self.timeout, self.max_source_age, self.max_linear, self.max_angular,
         self.stopped_linear, self.stopped_angular, self.stop_grace) = map(float, values)
        self.measurement = None
        self.at = None
        self.stamp_ns = None
        self.latest_stamp_ns = None
        self.stopping_since = None
        self.latched_reason = None

    def update(self, vx, vy, angular, stamp_ns, now, ros_now_ns):
        if (not all(number(v) for v in (vx, vy, angular, now)) or
                not fresh_frame(stamp_ns, ros_now_ns, self.max_source_age)):
            self.measurement, self.at, self.stamp_ns = None, None, None
            return
        if self.latest_stamp_ns is not None and stamp_ns <= self.latest_stamp_ns:
            return
        self.latest_stamp_ns = stamp_ns
        self.measurement = (float(vx), float(vy), float(angular))
        self.at, self.stamp_ns = float(now), stamp_ns
        if math.hypot(vx, vy) > self.max_linear or abs(angular) > self.max_angular:
            self.latched_reason = 'odometry_limit'

    def _fresh(self, now, ros_now_ns):
        return (number(now) and self.at is not None and 0 <= now-self.at <= self.timeout
                and fresh_frame(self.stamp_ns, ros_now_ns, self.max_source_age))

    def stopped(self):
        if self.measurement is None:
            return False
        vx, vy, angular = self.measurement
        return math.hypot(vx, vy) <= self.stopped_linear and abs(angular) <= self.stopped_angular

    def evaluate(self, now, ros_now_ns, stop_requested):
        if self.latched_reason:
            return False, self.latched_reason
        if not self._fresh(now, ros_now_ns):
            self.stopping_since = None
            return False, 'odometry_stale'
        if stop_requested and not self.stopped():
            if self.stopping_since is None:
                self.stopping_since = now
            elif now - self.stopping_since >= self.stop_grace:
                self.latched_reason = 'stop_not_confirmed'
                return False, self.latched_reason
        else:
            self.stopping_since = None
        return True, ''

    def can_reset(self, now, ros_now_ns):
        return self._fresh(now, ros_now_ns) and self.stopped()

    def reset(self, now, ros_now_ns):
        if not self.can_reset(now, ros_now_ns):
            return False
        self.latched_reason, self.stopping_since = None, None
        return True
