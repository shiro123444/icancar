"""Two-lap event controller. Input times share one monotonic clock.

The camera line is a calibrated proxy for the front-wheel crossing, not an
official timing sensor. This controller does not itself publish motor commands.
"""
import math


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class Edge:
    """Emit once on clear-to-present, after consecutive observations."""
    def __init__(self, frames):
        if not isinstance(frames, int) or isinstance(frames, bool) or frames < 1:
            raise ValueError('frames must be a positive integer')
        self.frames = frames
        self.state = None
        self.candidate = None
        self.count = 0
        self.cleared = False

    def observe(self, present):
        if not isinstance(present, bool):
            raise ValueError('detection must be boolean')
        if self.candidate != present:
            self.candidate, self.count = present, 1
        else:
            self.count += 1
        if self.count < self.frames or self.state == present:
            return False
        self.state = present
        if not present:
            self.cleared = True
            return False
        if self.cleared:
            self.cleared = False
            return True
        return False


class Race:
    def __init__(self, frames=3, timeout=0.6, stop_seconds=3.0,
                 max_linear=0.02, max_angular=0.05):
        if not all(number(x) and x > 0 for x in (timeout, stop_seconds, max_linear, max_angular)):
            raise ValueError('parameters must be positive finite numbers')
        if stop_seconds < 3:
            raise ValueError('stop duration must be at least 3 seconds')
        self.line, self.crosswalk = Edge(frames), Edge(frames)
        self.timeout, self.stop_seconds = float(timeout), float(stop_seconds)
        self.max_linear, self.max_angular = float(max_linear), float(max_angular)
        self.phase, self.reason = 'idle', ''
        self.laps, self.yields = 0, 0
        self.started, self.finished = None, None
        self.frame_at, self.odom_at, self.last_time = None, None, None
        self.stationary_since = None
        self.announcement_id, self.announcement_ok = 0, False
        self.events = []

    def _now(self, now):
        if not number(now) or (self.last_time is not None and now < self.last_time):
            raise ValueError('time must be a monotonic finite number')
        self.last_time = float(now)

    def _fresh(self, at, now):
        return at is not None and 0 <= now - at <= self.timeout

    def _check(self, now):
        if self.phase in ('armed', 'running', 'yield'):
            if not self._fresh(self.frame_at, now):
                self.phase, self.reason = 'fault', 'camera_timeout'
                self.events.append({'type': 'fault', 'reason': self.reason, 'at': now})
            elif not self._fresh(self.odom_at, now):
                self.phase, self.reason = 'fault', 'odometry_timeout'
                self.events.append({'type': 'fault', 'reason': self.reason, 'at': now})

    def arm(self, now):
        self._now(now)
        if self.phase != 'idle':
            raise ValueError('reset before arming')
        if not self._fresh(self.frame_at, now) or not self._fresh(self.odom_at, now):
            raise ValueError('camera and odometry must be receiving data')
        if self.line.state is not False or self.crosswalk.state is not False:
            raise ValueError('both detection areas must initially be clear')
        self.phase, self.reason = 'armed', ''
        self.events.append({'type': 'armed', 'at': now})
        return self.status(now)

    def frame(self, now, line, crosswalk):
        self._now(now)
        self._check(now)
        if not isinstance(line, bool) or (crosswalk is not None and not isinstance(crosswalk, bool)):
            raise ValueError('detection must be boolean or absent')
        self.frame_at = now
        line_edge = self.line.observe(line)
        crosswalk_edge = self.crosswalk.observe(crosswalk) if crosswalk is not None else False
        if line_edge and self.phase == 'armed':
            self.phase, self.started = 'running', now
            self.events.append({'type': 'start', 'at': now})
        elif line_edge and self.phase in ('running', 'yield'):
            self.laps += 1
            self.events.append({'type': 'lap', 'lap': self.laps, 'at': now})
            if self.laps == 2:
                self.phase, self.finished = 'finished', now
                self.events.append({'type': 'finish', 'at': now})
        if crosswalk_edge and self.phase == 'running':
            self._start_yield(now)
        return self.status(now)

    def crosswalk_frame(self, now, present):
        """Observe one independent crosswalk-model result, not a repeated camera frame."""
        self._now(now)
        self._check(now)
        if not isinstance(present, bool):
            raise ValueError('crosswalk detection must be boolean')
        if self.crosswalk.observe(present) and self.phase == 'running':
            self._start_yield(now)
        return self.status(now)

    def _start_yield(self, now):
        self.phase, self.stationary_since = 'yield', None
        self.announcement_id += 1
        self.announcement_ok = False
        self.events.append({'type': 'announce', 'id': self.announcement_id,
                            'text': '人行道前停车礼让行人', 'at': now})

    def fail(self, now, reason):
        self._now(now)
        if self.phase in ('armed', 'running', 'yield'):
            self.phase, self.reason = 'fault', reason
            self.events.append({'type': 'fault', 'reason': reason, 'at': now})
        return self.status(now)

    def odom(self, now, vx, vy, angular):
        self._now(now)
        self._check(now)
        if not all(number(v) for v in (vx, vy, angular)):
            raise ValueError('invalid odometry')
        self.odom_at = now
        if self.phase == 'yield':
            stopped = math.hypot(vx, vy) <= self.max_linear and abs(angular) <= self.max_angular
            if not stopped:
                self.stationary_since = None
            elif self.stationary_since is None:
                self.stationary_since = now
            self._release(now)
        return self.status(now)

    def acknowledge(self, now, identifier, success):
        self._now(now)
        self._check(now)
        if self.phase == 'yield' and identifier == self.announcement_id and success is True:
            if not self.announcement_ok:
                self.events.append({'type': 'announcement_complete', 'id': identifier, 'at': now})
            self.announcement_ok = True
            self._release(now)
        return self.status(now)

    def _release(self, now):
        if (self.phase == 'yield' and self.announcement_ok and self.stationary_since is not None
                and self._fresh(self.odom_at, now) and now - self.stationary_since >= self.stop_seconds):
            self.phase = 'running'
            self.yields += 1
            self.events.append({'type': 'yield_complete', 'stationary_seconds': round(now - self.stationary_since, 3), 'at': now})
            self.stationary_since = None

    def tick(self, now):
        self._now(now)
        self._check(now)
        self._release(now)
        return self.status(now)

    def pop_events(self):
        result, self.events = self.events, []
        return result

    def status(self, now):
        end = self.finished if self.finished is not None else now
        elapsed = max(0.0, end - self.started) if self.started is not None else 0.0
        stationary = max(0.0, now - self.stationary_since) if self.stationary_since is not None else 0.0
        return {'phase': self.phase, 'action': 'follow_lane' if self.phase in ('armed', 'running') else 'stop',
                'reason': self.reason, 'laps': self.laps, 'crosswalk_stops': self.yields,
                'elapsed_seconds': round(elapsed, 2), 'stationary_seconds': round(stationary, 2),
                'announcement_id': self.announcement_id, 'announcement_ok': self.announcement_ok,
                'line': self.line.state, 'crosswalk': self.crosswalk.state}
