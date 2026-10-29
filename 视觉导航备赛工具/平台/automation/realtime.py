"""Bounded latest-value worker for time-sensitive perception.

At most one item is being processed and one newer item is waiting. Results
also occupy one slot. No ROS publisher, vehicle command, or hardware access
is created by this module.
"""
import math
import queue
import threading
import time


class LatestWorker:
    def __init__(self, process, max_hz, clock=time.monotonic):
        if not callable(process) or not isinstance(max_hz, (int, float)) or isinstance(max_hz, bool) or not math.isfinite(max_hz) or not 0 < max_hz <= 100:
            raise ValueError('worker process or rate is invalid')
        self._process, self._clock = process, clock
        self._period = 1.0 / float(max_hz)
        self._cv = threading.Condition()
        self._pending = None
        self._result = None
        self._closed = False
        self._started = False
        self._thread = None
        self._counts = {'submitted': 0, 'processed': 0, 'input_replaced': 0,
                        'result_replaced': 0, 'errors': 0}

    def start(self):
        with self._cv:
            if self._closed:
                raise RuntimeError('worker is closed')
            if self._started:
                return
            self._started = True
            self._thread = threading.Thread(target=self._run, name='latest-perception', daemon=True)
            self._thread.start()

    def submit(self, value):
        if value is None:
            raise ValueError('None is reserved for an empty slot')
        with self._cv:
            if self._closed:
                return False
            if self._pending is not None:
                self._counts['input_replaced'] += 1
            self._pending = value
            self._counts['submitted'] += 1
            self._cv.notify()
            return True

    def take(self):
        with self._cv:
            result, self._result = self._result, None
            return result

    def stats(self):
        with self._cv:
            return dict(self._counts, pending=int(self._pending is not None),
                        result_pending=int(self._result is not None), closed=self._closed)

    def close(self, timeout=1.0):
        if not isinstance(timeout, (int, float)) or isinstance(timeout, bool) or not math.isfinite(timeout) or timeout < 0:
            raise ValueError('close timeout is invalid')
        with self._cv:
            self._closed = True
            self._pending = None
            self._result = None
            self._cv.notify_all()
            thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout)
        return thread is None or not thread.is_alive()

    def _run(self):
        next_start = 0.0
        while True:
            with self._cv:
                while not self._closed:
                    if self._pending is None:
                        self._cv.wait()
                        continue
                    delay = next_start - self._clock()
                    if delay > 0:
                        self._cv.wait(timeout=delay)
                        continue
                    item, self._pending = self._pending, None
                    next_start = self._clock() + self._period
                    break
                if self._closed:
                    return
            try:
                value = self._process(item)
                result = {'ok': True, 'value': value}
            except Exception as exc:
                result = {'ok': False, 'error': str(exc)[:300] or type(exc).__name__}
            with self._cv:
                if self._closed:
                    return
                self._counts['processed'] += 1
                if not result['ok']:
                    self._counts['errors'] += 1
                if self._result is not None:
                    self._counts['result_replaced'] += 1
                self._result = result


class EventWriter:
    """Bounded, non-blocking event sink; overflow and I/O errors are latched."""
    def __init__(self, write, capacity=256, stall_timeout=2.0):
        if (not callable(write) or not isinstance(capacity, int) or isinstance(capacity, bool) or not 1 <= capacity <= 10000
                or not isinstance(stall_timeout, (int,float)) or isinstance(stall_timeout, bool)
                or not math.isfinite(stall_timeout) or stall_timeout <= 0):
            raise ValueError('event writer configuration is invalid')
        self._write = write
        self._queue = queue.Queue(maxsize=capacity)
        self._lock = threading.Lock()
        self._closed = False
        self._error = None
        self._stall_timeout = float(stall_timeout)
        self._inflight_at = None
        self._written = 0
        self._dropped = 0
        self._thread = threading.Thread(target=self._run, name='race-event-writer', daemon=True)
        self._thread.start()

    def submit(self, value):
        with self._lock:
            if self._closed or self._error:
                self._dropped += 1
                return False
            try:
                self._queue.put_nowait(value)
                return True
            except queue.Full:
                self._error = 'event buffer full'
                self._dropped += 1
                return False

    def status(self):
        with self._lock:
            if (self._error is None and self._inflight_at is not None and
                    time.monotonic() - self._inflight_at > self._stall_timeout):
                self._error = 'event writer stalled'
            return {'pending': self._queue.qsize(), 'written': self._written,
                    'dropped': self._dropped, 'error': self._error, 'closed': self._closed}

    def close(self, timeout=2.0):
        if not isinstance(timeout, (int, float)) or isinstance(timeout, bool) or not math.isfinite(timeout) or timeout < 0:
            raise ValueError('close timeout is invalid')
        with self._lock:
            self._closed = True
        if self._thread is not threading.current_thread():
            self._thread.join(timeout)
        return not self._thread.is_alive() and self.status()['error'] is None

    def _run(self):
        while True:
            with self._lock:
                done = self._closed and self._queue.empty()
            if done:
                return
            try:
                value = self._queue.get(timeout=.05)
            except queue.Empty:
                continue
            with self._lock:
                self._inflight_at = time.monotonic()
            try:
                self._write(value)
                with self._lock:
                    self._written += 1
            except Exception as exc:
                with self._lock:
                    self._error = self._error or (str(exc)[:300] or type(exc).__name__)
                    self._dropped += 1
            finally:
                with self._lock:
                    self._inflight_at = None
                self._queue.task_done()
