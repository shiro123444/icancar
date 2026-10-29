"""Velocity arbitration logic, independent of ROS and physical hardware."""
import math
from automation.race import number
from automation.steering_candidate import fresh_frame


class VelocityBuffer:
    """Bounded, expiring input for the vehicle-side serial adapter.

    Stamped commands must use the same ROS clock as the consumer. Receiving an
    old or repeated command never extends the watchdog. This does not replace a
    watchdog in the motor controller or a physical emergency stop.
    """
    def __init__(self, max_linear, max_angular, timeout, max_source_age=.3, stamped=True):
        if (not all(number(x) and x > 0 for x in (max_linear, max_angular, timeout, max_source_age))
                or not isinstance(stamped, bool)):
            raise ValueError('velocity buffer limits must be positive and finite')
        self.max_linear, self.max_angular = float(max_linear), float(max_angular)
        self.timeout, self.max_source_age, self.stamped = float(timeout), float(max_source_age), stamped
        self.values, self.received_at, self.stamp_ns = None, None, None
        self.latest_stamp_ns = None

    def clear(self):
        self.values, self.received_at, self.stamp_ns = None, None, None

    def source_fresh(self, stamp_ns, ros_now_ns):
        if (not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or not 0 < stamp_ns < 2**63
                or not isinstance(ros_now_ns, int) or isinstance(ros_now_ns, bool)
                or not 0 < ros_now_ns < 2**63):
            return False
        age = (ros_now_ns - stamp_ns) / 1_000_000_000
        return -.02 <= age <= self.max_source_age

    def update(self, vx, vy, angular, now, stamp_ns=None, ros_now_ns=None):
        if not all(number(v) for v in (vx, vy, angular, now)):
            self.clear()
            return False
        if self.stamped:
            if not self.source_fresh(stamp_ns, ros_now_ns):
                self.clear()
                return False
            if self.latest_stamp_ns is not None and stamp_ns <= self.latest_stamp_ns:
                # A late stop is still safe to honor; no replay refreshes the timer.
                if vx == vy == angular == 0:
                    self.clear()
                return False
        speed = math.hypot(vx, vy)
        if not math.isfinite(speed):
            self.clear()
            return False
        if speed > self.max_linear:
            scale = self.max_linear / speed
            vx, vy = vx * scale, vy * scale
        angular = max(-self.max_angular, min(self.max_angular, angular))
        self.values, self.received_at, self.stamp_ns = (float(vx), float(vy), float(angular)), float(now), stamp_ns
        if self.stamped:
            self.latest_stamp_ns = stamp_ns
        return True

    def output(self, now, ros_now_ns=None):
        if (not number(now) or self.received_at is None or not 0 <= now - self.received_at <= self.timeout
                or (self.stamped and not self.source_fresh(self.stamp_ns, ros_now_ns))):
            self.clear()
            return (0.0, 0.0, 0.0)
        return self.values


class CommandGate:
    def __init__(self, max_linear, max_angular, timeout=0.5, enabled=False, max_frame_age=None):
        if (not all(number(v) and v > 0 for v in (max_linear, max_angular, timeout))
                or (max_frame_age is not None and (not number(max_frame_age) or max_frame_age <= 0))):
            raise ValueError('limits and timeout must be positive and finite')
        self.max_linear, self.max_angular, self.timeout = max_linear, max_angular, timeout
        self.enabled = enabled is True
        self.max_frame_age = max_frame_age
        self.command, self.command_at = None, None
        self.frame_stamp_ns, self.latest_stamp_ns = None, None
        self.state, self.state_at = None, None
        self.safety, self.safety_at = None, None

    def set_state(self, state, now):
        if not isinstance(state, dict) or not number(now):
            self.state, self.state_at = None, None
            return
        self.state, self.state_at = state, float(now)

    def set_command(self, vx, vy, angular, now, frame_stamp_ns=None, ros_now_ns=None):
        if (not all(number(v) for v in (vx, vy, angular, now)) or not math.isfinite(math.hypot(vx, vy)) or
                (self.max_frame_age is not None and not fresh_frame(frame_stamp_ns, ros_now_ns, self.max_frame_age))):
            self.command, self.command_at = None, None
            self.frame_stamp_ns = None
            return
        if self.max_frame_age is not None and self.latest_stamp_ns is not None and frame_stamp_ns < self.latest_stamp_ns:
            return
        self.command, self.command_at = (float(vx), float(vy), float(angular)), float(now)
        self.frame_stamp_ns = frame_stamp_ns
        if self.max_frame_age is not None:self.latest_stamp_ns = frame_stamp_ns

    def clear_command(self):
        self.command, self.command_at, self.frame_stamp_ns = None, None, None

    def set_safety(self, clear, now):
        if not isinstance(clear, bool) or not number(now):
            self.safety, self.safety_at = None, None
            return
        self.safety, self.safety_at = clear, float(now)

    def output(self, now, ros_now_ns=None):
        if not self.enabled:
            return None
        zero = (0.0, 0.0, 0.0)
        if not number(now) or self.command_at is None or self.state_at is None or self.safety_at is None:
            return zero
        if not (0 <= now - self.command_at <= self.timeout and 0 <= now - self.state_at <= self.timeout
                and 0 <= now - self.safety_at <= self.timeout and self.safety is True):
            return zero
        if self.max_frame_age is not None and not fresh_frame(self.frame_stamp_ns, ros_now_ns, self.max_frame_age):
            return zero
        state = self.state
        if (state.get('calibrated') is not True or state.get('action') != 'follow_lane'
                or state.get('phase') not in ('armed', 'running')):
            return zero
        vx, vy, angular = self.command
        speed = math.hypot(vx, vy)
        if speed > self.max_linear:
            scale = self.max_linear / speed
            vx, vy = vx * scale, vy * scale
        angular = max(-self.max_angular, min(self.max_angular, angular))
        return vx, vy, angular
