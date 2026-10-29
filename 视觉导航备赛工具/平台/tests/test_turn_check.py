import struct
import unittest
from automation.base_protocol import velocity_frame
from tests.test_bench_stop_check import Clock, Port
from tools.turn_check import run_check


class TurnCheckTests(unittest.TestCase):
    def test_both_yaw_signs_and_stop(self):
        for sign in (-1, 1):
            clock=Clock(); port=Port(clock)
            result=run_check(port,yaw_rate=.3*sign,clock=clock.time,pause=clock.sleep)
            self.assertNotIn('error',result)
            self.assertGreater(result['integrated_yaw_rad']*sign,.4)
            self.assertTrue(result['stop_feedback_confirmed'])
            self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_missing_feedback_cannot_start(self):
        clock=Clock(); port=Port(clock,respond=False)
        result=run_check(port,clock=clock.time,pause=clock.sleep)
        self.assertIn('error',result)
        self.assertEqual(result['nonzero_commands'],0)
    def test_opposite_yaw_stops(self):
        class Opposite(Port):
            def read(self,count):
                data=bytearray(super().read(count))
                if data:
                    data[20:22]=struct.pack('>h',-int(self.velocity[2]*1000));data[-1]=sum(data[:-1])&255
                return bytes(data)
        clock=Clock();port=Opposite(clock)
        result=run_check(port,clock=clock.time,pause=clock.sleep)
        self.assertIn('opposite sign',result['error'])
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))


if __name__=='__main__':unittest.main()
