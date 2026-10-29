"""Exercise ROS adapters with in-memory messages and a fake serial port."""
import json
import struct
import sys
import types
import unittest
from unittest.mock import patch

from automation.base_protocol import velocity_frame
from ros2 import command_gate, deepbot_base


class Stamp:
    def __init__(self, ns):
        self.sec, self.nanosec = divmod(ns, 1_000_000_000)


class Vector:
    def __init__(self, x=0., y=0., z=0.):
        self.x, self.y, self.z = x, y, z


class Twist:
    def __init__(self):
        self.linear, self.angular = Vector(), Vector()


class TwistStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=Stamp(0), frame_id='')
        self.twist = Twist()


class Odometry:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=Stamp(0))
        self.twist = types.SimpleNamespace(twist=Twist())


class Message:
    def __init__(self, data=None):
        self.data = data


class Publisher:
    def __init__(self, kind, topic, depth):
        self.kind, self.topic, self.depth = kind, topic, depth
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    now_ns = 10_000_000_000

    def __init__(self, name):
        self.publishers, self.subscriptions, self.timers = [], [], []

    def create_publisher(self, kind, topic, depth):
        result = Publisher(kind, topic, depth)
        self.publishers.append(result)
        return result

    def create_subscription(self, kind, topic, callback, depth):
        self.subscriptions.append((kind, topic, callback, depth))

    def create_timer(self, interval, callback, clock=None):
        self.timers.append((interval, callback, clock))

    def create_service(self, kind, name, callback):
        self.service = callback

    def get_clock(self):
        value = FakeNode.now_ns
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=value, to_msg=lambda: Stamp(value)))

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *args, **kwargs: None,
                                     warning=lambda *args, **kwargs: None, error=lambda *args, **kwargs: None)

    def destroy_node(self):
        pass


class FakePort:
    instances = []

    def __init__(self, *args, **kwargs):
        self.writes, self.closed, self.incoming = [], False, bytearray()
        self.instances.append(self)

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, count):
        data=bytes(self.incoming[:count])
        del self.incoming[:count]
        return data

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True


def fake_modules(spin):
    modules = {}
    rclpy = types.ModuleType('rclpy')
    rclpy.init, rclpy.shutdown, rclpy.spin = lambda: None, lambda: None, spin
    modules['rclpy'] = rclpy
    node = types.ModuleType('rclpy.node')
    node.Node = FakeNode
    modules['rclpy.node'] = node
    clock = types.ModuleType('rclpy.clock')
    clock.ClockType = types.SimpleNamespace(STEADY_TIME=3)
    clock.Clock = lambda clock_type: types.SimpleNamespace(clock_type=clock_type)
    modules['rclpy.clock'] = clock
    qos = types.ModuleType('rclpy.qos')
    qos.qos_profile_sensor_data = object()
    qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    modules['rclpy.qos'] = qos
    for package, items in (
        ('geometry_msgs.msg', {'Twist': Twist, 'TwistStamped': TwistStamped}),
        ('nav_msgs.msg', {'Odometry': Odometry}),
        ('std_msgs.msg', {'String': Message, 'Bool': Message}),
        ('std_srvs.srv', {'Trigger': type('Trigger', (), {})}),
    ):
        name, child = package.split('.')
        modules.setdefault(name, types.ModuleType(name))
        sub = types.ModuleType(package)
        for key, value in items.items():
            setattr(sub, key, value)
        modules[package] = sub
    serial = types.ModuleType('serial')
    serial.Serial, serial.SerialException = FakePort, type('SerialException', (Exception,), {})
    modules['serial'] = serial
    return modules


def stamped(ns, x=0., yaw=0.):
    msg = TwistStamped()
    msg.header.stamp = Stamp(ns)
    msg.twist.linear.x, msg.twist.angular.z = x, yaw
    return msg


class AdapterTests(unittest.TestCase):
    def setUp(self):
        FakeNode.now_ns = 10_000_000_000
        FakePort.instances.clear()

    def test_gate_emits_stamped_stop_immediately_when_safety_fails(self):
        def spin(node):
            self.assertIs(node.publisher.kind, TwistStamped)
            self.assertEqual(node.publisher.depth, 1)
            self.assertTrue(all(timer[2].clock_type == 3 for timer in node.timers))
            node.state(Message('{"calibrated":true,"phase":"running","action":"follow_lane"}'))
            node.safety(Message(True))
            node.estop(Message(False))
            odom = Odometry()
            odom.header.stamp = Stamp(FakeNode.now_ns)
            node.odometry(odom)
            response = node.reset(None, types.SimpleNamespace(success=False, message=''))
            self.assertTrue(response.success)
            node.command(stamped(FakeNode.now_ns, .05, .2))
            node.update()
            movement = node.publisher.messages[-1]
            self.assertEqual((movement.twist.linear.x, movement.twist.angular.z), (.05, .2))
            self.assertEqual(movement.header.stamp.sec, 10)
            count = len(node.publisher.messages)
            node.safety(Message(False))
            self.assertEqual(len(node.publisher.messages), count + 1)
            self.assertEqual(node.publisher.messages[-1].twist.linear.x, 0)

        args = ['command_gate.py', '--activate', '--input-topic', '/trial/candidate',
                '--output-topic', '/trial/output', '--safety-topic', '/trial/safety',
                '--max-linear', '.05', '--max-angular', '.4']
        with patch.dict(sys.modules, fake_modules(spin)), patch.object(sys, 'argv', args), \
                patch.object(command_gate.time, 'monotonic', return_value=1.0):
            command_gate.main()

    def test_base_uses_latest_only_and_writes_stop_on_loss_or_explicit_stop(self):
        monotonic = [1.0]
        def spin(node):
            port = FakePort.instances[-1]
            self.assertEqual((node.subscriptions[0][0], node.subscriptions[0][3]), (TwistStamped, 1))
            self.assertTrue(all(timer[2].clock_type == 3 for timer in node.timers))
            node.command_cb(stamped(10_000_000_000, .05, .2))
            node.send_command()
            self.assertEqual(port.writes[-1], velocity_frame(.05, 0, .2))
            monotonic[0], FakeNode.now_ns = 1.2, 10_200_000_000
            node.command_cb(stamped(10_000_000_000, .05, .2))  # replay cannot extend the timeout
            monotonic[0], FakeNode.now_ns = 1.31, 10_310_000_000
            node.send_command()
            self.assertEqual(port.writes[-1], velocity_frame(0, 0, 0))
            monotonic[0], FakeNode.now_ns = 1.4, 10_400_000_000
            node.command_cb(stamped(FakeNode.now_ns, .05, .2))
            node.send_command()
            count = len(port.writes)
            monotonic[0], FakeNode.now_ns = 1.41, 10_410_000_000
            node.command_cb(stamped(FakeNode.now_ns))
            self.assertEqual(len(port.writes), count + 1)  # no timer needed for a stop
            self.assertEqual(port.writes[-1], velocity_frame(0, 0, 0))

        args = ['deepbot_base.py', '--device', '/fake-device', '--enable-drive']
        with patch.dict(sys.modules, fake_modules(spin)), patch.object(sys, 'argv', args), \
                patch.object(deepbot_base.time, 'monotonic', side_effect=lambda: monotonic[0]):
            deepbot_base.main()
        self.assertTrue(FakePort.instances[-1].closed)

    def test_readonly_base_observes_vendor_sensor_values_without_motor_writes(self):
        def spin(node):
            port = FakePort.instances[-1]
            frame=bytearray(32)
            frame[:3]=bytes([0xab,0xcd,32])
            frame[22:26]=struct.pack('>hh',1234,321)
            frame[26:30]=bytes([0,1,0,1])
            frame[-1]=sum(frame[:-1]) & 255
            port.incoming.extend(frame)
            node.last_publish=1.0  # Focus this test on the independent observation.
            node.receive()
            values=json.loads(node.sensor_pub.messages[-1].data)
            self.assertEqual(values['battery_vendor_value'],12.34)
            self.assertEqual(values['sonar_vendor_value'],32.1)
            self.assertEqual(values['infrared_vendor_values'],[0,1,0,1])
            self.assertEqual(values['stamp_ns'],FakeNode.now_ns)
            self.assertEqual(port.writes,[])
            self.assertFalse(node.subscriptions)

        args=['deepbot_base.py','--device','/fake-device',
              '--sensor-topic','/visual_race/base_sensor_diagnostics']
        with patch.dict(sys.modules, fake_modules(spin)), patch.object(sys, 'argv', args), \
                patch.object(deepbot_base.time, 'monotonic', return_value=1.0):
            deepbot_base.main()
        self.assertTrue(FakePort.instances[-1].closed)


if __name__ == '__main__':
    unittest.main()
