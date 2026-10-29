import threading
import time
import unittest

from automation.realtime import EventWriter, LatestWorker
from automation.command import CommandGate
from automation.feedback import EmergencyLatch, FeedbackGuard
from automation.steering_candidate import SteeringCandidate
from automation.supervisor import MotionSupervisor


class LatestWorkerTests(unittest.TestCase):
    def test_slow_inference_keeps_latest_frame_and_stays_bounded(self):
        started, release, latest_done = threading.Event(), threading.Event(), threading.Event()
        seen = []
        def process(value):
            seen.append(value)
            if value == 1:
                started.set()
                if not release.wait(2):
                    raise RuntimeError('test did not release')
            if value == 4:
                latest_done.set()
            return value * 10
        worker = LatestWorker(process, 100)
        worker.start()
        try:
            worker.submit(1)
            self.assertTrue(started.wait(1))
            for value in (2, 3, 4):
                worker.submit(value)
            self.assertEqual(worker.stats()['pending'], 1)
            release.set()
            self.assertTrue(latest_done.wait(2))
            deadline = time.monotonic() + 1
            while worker.stats()['processed'] < 2 and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertEqual(seen, [1, 4])
            self.assertEqual(worker.stats()['input_replaced'], 2)
            self.assertEqual(worker.take(), {'ok': True, 'value': 40})
            self.assertIsNone(worker.take())
        finally:
            release.set()
            self.assertTrue(worker.close())
        self.assertFalse(worker.submit(5))

    def test_inference_error_does_not_kill_worker(self):
        completed = threading.Event()
        def process(value):
            if value == 'bad':
                raise ValueError('bad frame')
            completed.set()
            return value
        worker = LatestWorker(process, 100)
        worker.start()
        try:
            worker.submit('bad')
            deadline = time.monotonic() + 1
            while worker.stats()['errors'] == 0 and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertEqual(worker.take(), {'ok': False, 'error': 'bad frame'})
            worker.submit('good')
            self.assertTrue(completed.wait(1))
            deadline = time.monotonic() + 1
            while worker.stats()['processed'] < 2 and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertEqual(worker.take(), {'ok': True, 'value': 'good'})
        finally:
            worker.close()

    def test_stalled_inference_close_does_not_wait_indefinitely(self):
        started, release = threading.Event(), threading.Event()
        def blocked(value):
            started.set()
            release.wait(2)
            return value
        worker = LatestWorker(blocked, 100)
        worker.start()
        worker.submit(1)
        self.assertTrue(started.wait(1))
        self.assertFalse(worker.close(timeout=.02))
        self.assertFalse(worker.submit(2))
        release.set()
        self.assertTrue(worker.close(timeout=1))
        self.assertIsNone(worker.take())

    def test_event_buffer_overflow_and_disk_failure_are_reported(self):
        started, release = threading.Event(), threading.Event()
        def slow_writer(value):
            started.set()
            release.wait(2)
        sink = EventWriter(slow_writer, capacity=1)
        try:
            self.assertTrue(sink.submit('first'))
            self.assertTrue(started.wait(1))
            self.assertTrue(sink.submit('second'))
            self.assertFalse(sink.submit('third'))
            self.assertEqual(sink.status()['error'], 'event buffer full')
            self.assertEqual(sink.status()['dropped'], 1)
        finally:
            release.set()
            self.assertFalse(sink.close())
        def bad_writer(value):
            raise OSError('disk unavailable')
        sink = EventWriter(bad_writer, capacity=1)
        try:
            self.assertTrue(sink.submit('event'))
            deadline = time.monotonic() + 1
            while not sink.status()['error'] and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertEqual(sink.status()['error'], 'disk unavailable')
            self.assertFalse(sink.submit('another'))
        finally:
            self.assertFalse(sink.close())

    def test_stalled_event_writer_is_reported(self):
        started, release = threading.Event(), threading.Event()
        def blocked(value):
            started.set()
            release.wait(2)
        sink = EventWriter(blocked, capacity=2, stall_timeout=.02)
        try:
            self.assertTrue(sink.submit('event'))
            self.assertTrue(started.wait(1))
            deadline = time.monotonic() + 1
            while not sink.status()['error'] and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertEqual(sink.status()['error'], 'event writer stalled')
        finally:
            release.set()
            self.assertFalse(sink.close())


class ContinuousWindowTests(unittest.TestCase):
    def test_newest_sample_has_more_weight_and_stop_clears_window(self):
        node = SteeringCandidate(.05, .4, .5, enabled=True, max_frame_age=.4,
                                 window_seconds=.25, window_samples=3)
        def prediction(stamp, steering):
            return {'source': 'steering_cnn', 'valid': True, 'steering': steering,
                    'image_stamp_ns': stamp}
        node.update(prediction(10_000_000_000, .4), 1.0, 10_000_000_000)
        node.update(prediction(10_200_000_000, -.4), 1.2, 10_200_000_000)
        self.assertAlmostEqual(node.output(1.21, 10_210_000_000)[2], -.4 / 3)
        node.update({'source': 'steering_cnn', 'valid': False}, 1.22, 10_220_000_000)
        self.assertEqual(node.output(1.23, 10_230_000_000), (0.0, 0.0, 0.0))
        node.update(prediction(10_300_000_000, .3), 1.3, 10_300_000_000)
        self.assertAlmostEqual(node.output(1.31, 10_310_000_000)[2], .3)

    def test_replay_and_future_frames_cannot_refresh_or_poison_stream(self):
        node = SteeringCandidate(.05, .4, .5, enabled=True, max_frame_age=.4,
                                 window_seconds=.25, window_samples=3)
        def prediction(stamp):
            return {'source':'steering_cnn','valid':True,'steering':.2,'image_stamp_ns':stamp}
        node.update(prediction(10_000_000_000), 1, 10_000_000_000)
        node.update(prediction(10_000_000_000), 1.3, 10_300_000_000)
        self.assertEqual(node.output(1.41, 10_410_000_000), (0.0, 0.0, 0.0))
        node.update(prediction(30_000_000_000), 1.5, 10_500_000_000)
        self.assertEqual(node.output(1.5, 10_500_000_000), (0.0, 0.0, 0.0))
        node.update(prediction(10_600_000_000), 1.6, 10_600_000_000)
        self.assertEqual(node.output(1.61, 10_610_000_000), (.05, 0.0, .2))

    def test_gate_rechecks_image_age_and_priority_stop(self):
        gate = CommandGate(.05, .4, .5, enabled=True, max_frame_age=.4)
        gate.set_state({'calibrated':True, 'phase':'running', 'action':'follow_lane'}, 1.0)
        gate.set_safety(True, 1.0)
        gate.set_command(.05, 0, .2, 1.0, 10_000_000_000, 10_100_000_000)
        self.assertEqual(gate.output(1.1, 10_200_000_000), (.05, 0.0, .2))
        gate.set_safety(False, 1.11)
        self.assertEqual(gate.output(1.11, 10_210_000_000), (0.0, 0.0, 0.0))
        gate.set_safety(True, 1.12)
        self.assertEqual(gate.output(1.2, 10_500_000_000), (0.0, 0.0, 0.0))
        # Receiving the same old command again must not restart motion.
        gate.set_command(.05, 0, .2, 1.21, 10_000_000_000, 10_510_000_000)
        self.assertEqual(gate.output(1.22, 10_520_000_000), (0.0, 0.0, 0.0))

    def test_emergency_latches_until_fresh_clear_and_explicit_reset(self):
        latch = EmergencyLatch(.5)
        self.assertFalse(latch.allowed(1.0))
        latch.update(False, 1.0)
        self.assertFalse(latch.allowed(1.1))
        self.assertTrue(latch.reset(1.1))
        self.assertTrue(latch.allowed(1.2))
        latch.update(True, 1.21)
        self.assertFalse(latch.allowed(1.22))
        latch.update(False, 1.23)
        self.assertFalse(latch.allowed(1.24))
        self.assertTrue(latch.reset(1.25))
        self.assertFalse(latch.allowed(1.8))
        self.assertFalse(latch.reset(1.8))

    def test_odometry_stale_excess_speed_and_stop_failure(self):
        guard = FeedbackGuard(timeout=.5, max_source_age=.5, max_linear=.25,
                              max_angular=1.0, stop_grace=.5)
        self.assertEqual(guard.evaluate(1, 10_000_000_000, False), (False, 'odometry_stale'))
        guard.update(.1, 0, 0, 10_000_000_000, 1, 10_000_000_000)
        self.assertEqual(guard.evaluate(1.1, 10_100_000_000, False), (True, ''))
        guard.update(.1, 0, 0, 10_100_000_000, 1.1, 10_100_000_000)
        self.assertEqual(guard.evaluate(1.1, 10_100_000_000, True), (True, ''))
        guard.update(.1, 0, 0, 10_600_000_000, 1.6, 10_600_000_000)
        self.assertEqual(guard.evaluate(1.6, 10_600_000_000, True), (False, 'stop_not_confirmed'))
        self.assertFalse(guard.reset(1.6, 10_600_000_000))
        guard.update(0, 0, 0, 10_700_000_000, 1.7, 10_700_000_000)
        self.assertTrue(guard.reset(1.7, 10_700_000_000))
        guard.update(.3, 0, 0, 10_800_000_000, 1.8, 10_800_000_000)
        self.assertEqual(guard.evaluate(1.8, 10_800_000_000, False), (False, 'odometry_limit'))
        guard.update(0, 0, 0, 10_900_000_000, 1.9, 10_900_000_000)
        self.assertEqual(guard.evaluate(1.9, 10_900_000_000, False), (False, 'odometry_limit'))
        self.assertTrue(guard.reset(1.9, 10_900_000_000))

    def test_supervisor_requires_fresh_inputs_and_new_command_after_reset(self):
        gate = CommandGate(.05, .4, .5, enabled=True, max_frame_age=.4)
        feedback, estop = FeedbackGuard(), EmergencyLatch(.5)
        system = MotionSupervisor(gate, feedback, estop)
        gate.set_state({'calibrated':True, 'phase':'running', 'action':'follow_lane'}, 1)
        gate.set_safety(True, 1)
        estop.update(False, 1)
        feedback.update(0, 0, 0, 10_000_000_000, 1, 10_000_000_000)
        gate.set_command(.05, 0, .2, 1, 10_000_000_000, 10_000_000_000)
        self.assertEqual(system.output(1.01, 10_010_000_000), (0, 0, 0))
        self.assertTrue(system.reset(1.02, 10_020_000_000)[0])
        self.assertEqual(system.output(1.03, 10_030_000_000), (0, 0, 0))
        gate.set_command(.05, 0, .2, 1.04, 10_040_000_000, 10_040_000_000)
        self.assertEqual(system.output(1.05, 10_050_000_000), (.05, 0, .2))
        estop.update(True, 1.06)
        self.assertEqual(system.output(1.06, 10_060_000_000), (0, 0, 0))
        estop.update(False, 1.07)
        self.assertEqual(system.output(1.08, 10_080_000_000), (0, 0, 0))
        self.assertTrue(system.reset(1.09, 10_090_000_000)[0])
        self.assertEqual(system.output(1.1, 10_100_000_000), (0, 0, 0))
        gate.set_command(.05, 0, .2, 1.11, 10_110_000_000, 10_110_000_000)
        gate.set_safety(False, 1.12)
        self.assertEqual(system.output(1.12, 10_120_000_000), (0, 0, 0))

    def test_failed_reset_keeps_other_faults_latched(self):
        gate = CommandGate(.05, .4, .5, enabled=True, max_frame_age=.4)
        feedback, estop = FeedbackGuard(), EmergencyLatch(.5)
        system = MotionSupervisor(gate, feedback, estop)
        gate.set_safety(True, 1)
        feedback.update(.4, 0, 0, 10_000_000_000, 1, 10_000_000_000)
        feedback.update(0, 0, 0, 10_010_000_000, 1.01, 10_010_000_000)
        estop.update(True, 1.01)
        self.assertFalse(system.reset(1.02, 10_020_000_000)[0])
        self.assertEqual(feedback.latched_reason, 'odometry_limit')
        self.assertTrue(estop.latched)


if __name__ == '__main__':
    unittest.main()
