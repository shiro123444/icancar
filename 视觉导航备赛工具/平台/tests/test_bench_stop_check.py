import struct
import unittest

from tools.bench_stop_check import run_check
from automation.base_protocol import velocity_frame


class Clock:
    def __init__(self):self.now=0.
    def time(self):return self.now
    def sleep(self,value):self.now+=value


class Port:
    def __init__(self,clock,respond=True,watchdog=None,reverse=False,sonar=57,unstable_reads=0,unstable_on_motion=False,moving_sonar=None):
        self.clock,self.respond,self.watchdog,self.reverse=clock,respond,watchdog,reverse
        self.writes=[];self.velocity=(0.,0.,0.);self.last_write=0.
        self.sonar=sonar
        self.moving_sonar=moving_sonar
        self.unstable_reads,self.unstable_on_motion,self.reads=unstable_reads,unstable_on_motion,0
    @property
    def in_waiting(self):return 32 if self.respond else 0
    def write(self,data):
        self.writes.append(data);self.velocity=tuple(x/1000 for x in struct.unpack('>hhh',data[4:10]))
        self.last_write=self.clock.time();return len(data)
    def read(self,count):
        if not count:return b''
        self.reads+=1
        values=self.velocity
        if self.watchdog is not None and self.clock.time()-self.last_write>self.watchdog:values=(0.,0.,0.)
        if self.reverse:values=(-values[0],values[1],values[2])
        frame=bytearray(32);frame[:3]=bytes([0xab,0xcd,32])
        frame[16:22]=struct.pack('>hhh',*(int(x*1000) for x in values));frame[-1]=sum(frame[:-1])&255
        sonar=30 if self.reads<=self.unstable_reads or (self.unstable_on_motion and self.velocity[0]>0) else self.sonar
        if self.moving_sonar is not None and self.velocity[0]>0:sonar=self.moving_sonar
        frame[22:26]=struct.pack('>hh',1400,int(sonar*10));frame[-1]=sum(frame[:-1])&255
        return bytes(frame)


class BenchStopTests(unittest.TestCase):
    def run_port(self,**kwargs):
        clock=Clock();port=Port(clock,**kwargs)
        result=run_check(port,clock=clock.time,pause=clock.sleep,watchdog=kwargs.get('watchdog') is not None)
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
        return result,port
    def test_zero_feedback_is_required_before_motion(self):
        result,port=self.run_port(respond=False)
        self.assertIn('error',result)
        self.assertTrue(all(x==velocity_frame(0,0,0) for x in port.writes))
    def test_forward_and_explicit_stop(self):
        result,_=self.run_port()
        self.assertTrue(result['stop_feedback_confirmed'])
        self.assertFalse(result['firmware_timeout_observed'])
        self.assertGreater(result['nonzero_commands'],0)
    def test_opposite_feedback_aborts(self):
        result,_=self.run_port(reverse=True)
        self.assertIn('negative forward feedback',result['error'])
    def test_firmware_timeout_is_separate_from_explicit_zero(self):
        result,_=self.run_port(watchdog=.35)
        self.assertTrue(result['firmware_timeout_observed'])
        self.assertGreaterEqual(result['heartbeat_silence_s'],.35)
        clock=Clock();port=Port(clock)
        result=run_check(port,watchdog=True,clock=clock.time,pause=clock.sleep)
        self.assertIn('error',result)
        self.assertFalse(result['firmware_timeout_observed'])
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_ground_test_needs_live_measured_clearance(self):
        clock=Clock();port=Port(clock,sonar=57)
        result=run_check(port,speed=.02,duration=.4,ground=True,clock=clock.time,pause=clock.sleep)
        self.assertTrue(result['stop_feedback_confirmed'])
        self.assertGreater(result['nonzero_commands'],0)
        for bad in (30,0,568):
            clock=Clock();port=Port(clock,sonar=bad)
            result=run_check(port,speed=.02,duration=.4,ground=True,clock=clock.time,pause=clock.sleep)
            self.assertIn('error',result)
            self.assertTrue(all(x==velocity_frame(0,0,0) for x in port.writes))
    def test_unstable_start_waits_but_motion_outlier_aborts(self):
        clock=Clock();port=Port(clock,unstable_reads=20)
        result=run_check(port,speed=.02,duration=.4,ground=True,clock=clock.time,pause=clock.sleep)
        self.assertTrue(result['stop_feedback_confirmed'])
        self.assertEqual(result['range_rejections'],20)
        clock=Clock();port=Port(clock,unstable_on_motion=True)
        result=run_check(port,speed=.02,duration=.4,ground=True,clock=clock.time,pause=clock.sleep)
        self.assertIn('observed 30.0 cm',result['error'])
        self.assertEqual(result['nonzero_commands'],1)
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_explicit_larger_step_keeps_a_small_distance_budget(self):
        clock=Clock();port=Port(clock)
        with self.assertRaises(ValueError):
            run_check(port,speed=.05,duration=.6,ground=True,clock=clock.time,pause=clock.sleep)
        result=run_check(port,speed=.05,duration=.6,ground=True,larger_step=True,clock=clock.time,pause=clock.sleep)
        self.assertTrue(result['stop_feedback_confirmed'])
        self.assertEqual(result['requested_speed_mps'],.05)
        self.assertLessEqual(result['requested_speed_mps']*result['requested_duration_s'],.03)
    def test_three_seconds_accepts_board_approach_and_then_sends_zero(self):
        clock=Clock();port=Port(clock,sonar=98,moving_sonar=83)
        result=run_check(port,speed=.05,duration=3,ground=True,three_second=True,clock=clock.time,pause=clock.sleep)
        self.assertNotIn('error',result)
        self.assertTrue(result['stop_feedback_confirmed'])
        self.assertGreaterEqual(result['motion_elapsed_s'],3)
        self.assertLess(result['motion_elapsed_s'],3.02)
        self.assertGreaterEqual(result['nonzero_commands'],28)
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_three_seconds_requires_one_metre_board_at_start(self):
        for sonar in (0,57,70,130,568):
            clock=Clock();port=Port(clock,sonar=sonar)
            result=run_check(port,speed=.05,duration=3,ground=True,three_second=True,clock=clock.time,pause=clock.sleep)
            self.assertIn('error',result)
            self.assertEqual(result['nonzero_commands'],0)
            self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_three_seconds_stops_early_for_close_or_missing_feedback(self):
        clock=Clock();port=Port(clock,sonar=98,moving_sonar=50)
        result=run_check(port,speed=.05,duration=3,ground=True,three_second=True,clock=clock.time,pause=clock.sleep)
        self.assertIn('observed 50.0 cm',result['error'])
        self.assertEqual(result['nonzero_commands'],1)
        self.assertLess(result['motion_elapsed_s'],.1)
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
        class DropsOnMotion(Port):
            @property
            def in_waiting(self):
                return 0 if any(x!=velocity_frame(0,0,0) for x in self.writes) else 32
        clock=Clock();port=DropsOnMotion(clock,sonar=98)
        result=run_check(port,speed=.05,duration=3,ground=True,three_second=True,clock=clock.time,pause=clock.sleep)
        self.assertIn('feedback lost',result['error'])
        self.assertLess(result['motion_elapsed_s'],.4)
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_three_seconds_preserves_speed_time_and_setup_limits(self):
        for options in ({'duration':3.1},{'speed':.06},{'watchdog':True},{'larger_step':True},{'ground':False}):
            clock=Clock();port=Port(clock,sonar=98)
            arguments=dict(speed=.05,duration=3,ground=True,three_second=True)
            arguments.update(options)
            with self.assertRaises(ValueError):run_check(port,clock=clock.time,pause=clock.sleep,**arguments)
            self.assertEqual(port.writes,[])
    def test_isolated_sonar_dips_recover_within_the_confirmation_window(self):
        class SingleFrameDips(Port):
            motion_reads=0
            def read(self,count):
                data=bytearray(super().read(count))
                if data and self.velocity[0]>0:
                    self.motion_reads+=1
                    sonar=34 if self.motion_reads%40==1 else 92
                    data[24:26]=struct.pack('>h',sonar*10);data[-1]=sum(data[:-1])&255
                return bytes(data)
        clock=Clock();port=SingleFrameDips(clock,sonar=92)
        result=run_check(port,speed=.05,duration=3,ground=True,three_second=True,clock=clock.time,pause=clock.sleep)
        self.assertNotIn('error',result)
        self.assertTrue(result['stop_feedback_confirmed'])
        self.assertGreater(result['transient_sonar_dips'],2)
        self.assertGreaterEqual(result['motion_elapsed_s'],3)
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_emergency_and_invalid_sonar_are_not_debounced(self):
        for sonar in (30,29,0,200):
            clock=Clock();port=Port(clock,sonar=92,moving_sonar=sonar)
            result=run_check(port,speed=.05,duration=3,ground=True,three_second=True,clock=clock.time,pause=clock.sleep)
            self.assertIn('observed %.1f cm'%sonar,result['error'])
            self.assertEqual(result['nonzero_commands'],1)
            self.assertLess(result['motion_elapsed_s'],.02)
            self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_pending_near_confirmation_cannot_wait_for_general_feedback_timeout(self):
        class NearThenSilence(Port):
            def read(self,count):
                data=super().read(count)
                if data and self.velocity[0]>0:self.respond=False
                return data
        clock=Clock();port=NearThenSilence(clock,sonar=92,moving_sonar=34)
        result=run_check(port,speed=.05,duration=3,ground=True,three_second=True,clock=clock.time,pause=clock.sleep)
        self.assertIn('confirmation deadline',result['error'])
        self.assertLess(result['motion_elapsed_s'],.08)
        self.assertEqual(result['nonzero_commands'],1)
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_next_speed_step_requires_more_space_and_stops_after_three_seconds(self):
        clock=Clock();port=Port(clock,sonar=148,moving_sonar=124)
        result=run_check(port,speed=.08,duration=3,ground=True,speed_step=True,clock=clock.time,pause=clock.sleep)
        self.assertNotIn('error',result)
        self.assertTrue(result['stop_feedback_confirmed'])
        self.assertGreaterEqual(result['motion_elapsed_s'],3)
        self.assertEqual(result['requested_speed_mps'],.08)
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
        clock=Clock();port=Port(clock,sonar=92)
        result=run_check(port,speed=.08,duration=3,ground=True,speed_step=True,clock=clock.time,pause=clock.sleep)
        self.assertIn('error',result)
        self.assertEqual(result['nonzero_commands'],0)
    def test_next_speed_step_cannot_raise_limits_or_share_other_motion_profiles(self):
        for options in ({'speed':.09},{'duration':3.1},{'watchdog':True},{'three_second':True},{'larger_step':True},{'ground':False}):
            clock=Clock();port=Port(clock,sonar=148)
            arguments=dict(speed=.08,duration=3,ground=True,speed_step=True)
            arguments.update(options)
            with self.assertRaises(ValueError):run_check(port,clock=clock.time,pause=clock.sleep,**arguments)
            self.assertEqual(port.writes,[])
    def test_speed_step_waits_longer_but_still_requires_stable_clearance(self):
        clock=Clock();port=Port(clock,sonar=148,unstable_reads=500)
        result=run_check(port,speed=.08,duration=3,ground=True,speed_step=True,clock=clock.time,pause=clock.sleep)
        self.assertNotIn('error',result)
        self.assertGreater(clock.time(),8)
        self.assertTrue(result['stop_feedback_confirmed'])
        clock=Clock();port=Port(clock,sonar=148,unstable_reads=900)
        result=run_check(port,speed=.08,duration=3,ground=True,speed_step=True,clock=clock.time,pause=clock.sleep)
        self.assertIn('stable clearance',result['error'])
        self.assertEqual(result['nonzero_commands'],0)
    def test_startup_filters_isolated_dips_but_repeated_near_readings_block_motion(self):
        class StartupDips(Port):
            consecutive=1
            def read(self,count):
                data=bytearray(super().read(count))
                if data and not self.velocity[0] and self.reads%20<self.consecutive:
                    data[24:26]=struct.pack('>h',340);data[-1]=sum(data[:-1])&255
                return bytes(data)
        clock=Clock();port=StartupDips(clock,sonar=148)
        result=run_check(port,speed=.08,duration=3,ground=True,speed_step=True,clock=clock.time,pause=clock.sleep)
        self.assertNotIn('error',result)
        self.assertGreater(result['transient_sonar_dips'],0)
        clock=Clock();port=StartupDips(clock,sonar=148);port.consecutive=2
        result=run_check(port,speed=.08,duration=3,ground=True,speed_step=True,clock=clock.time,pause=clock.sleep)
        self.assertIn('stable clearance',result['error'])
        self.assertEqual(result['nonzero_commands'],0)
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_supervised_straight_requires_two_metre_physical_clearance_and_bounded_speed(self):
        for options in ({'measured_clearance':None},{'measured_clearance':1.5},{'measured_clearance':float('nan')},{'speed':.21},{'speed':.8},{'duration':3.1},{'watchdog':True},{'speed_step':True},{'ground':False}):
            clock=Clock();port=Port(clock,sonar=195)
            arguments=dict(speed=.2,duration=3,ground=True,straight_test=True,measured_clearance=2.)
            arguments.update(options)
            with self.assertRaises(ValueError):run_check(port,clock=clock.time,pause=clock.sleep,**arguments)
            self.assertEqual(port.writes,[])
    def test_supervised_straight_ramps_and_then_stops(self):
        clock=Clock();port=Port(clock,sonar=195,moving_sonar=140)
        result=run_check(port,speed=.2,duration=3,ground=True,straight_test=True,measured_clearance=2.,clock=clock.time,pause=clock.sleep)
        self.assertNotIn('error',result)
        self.assertTrue(result['stop_feedback_confirmed'])
        speeds=[struct.unpack('>hhh',x[4:10])[0] for x in port.writes]
        positive=[v for v in speeds if v>0]
        self.assertEqual(positive[:5],[40,80,120,160,200])
        self.assertEqual(max(positive),200)
        self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_supervised_straight_records_sonar_without_using_it_as_permission(self):
        for sonar in (0,29,45,568):
            clock=Clock();port=Port(clock,sonar=195,moving_sonar=sonar)
            result=run_check(port,speed=.2,duration=3,ground=True,straight_test=True,measured_clearance=2.,clock=clock.time,pause=clock.sleep)
            self.assertNotIn('error',result)
            self.assertEqual(result['sonar_mode'],'diagnostic_only')
            self.assertFalse(result['range_interlock_enabled'])
            self.assertTrue(result['stop_feedback_confirmed'])
            self.assertEqual(port.writes[-1],velocity_frame(0,0,0))
    def test_supervised_straight_retains_feedback_loss_reverse_and_overspeed_stops(self):
        class Overspeed(Port):
            def read(self,count):
                data=bytearray(super().read(count))
                if data and self.velocity[0]>0:
                    data[16:22]=struct.pack('>hhh',400,0,0);data[-1]=sum(data[:-1])&255
                return bytes(data)
        class MissingOnMotion(Port):
            @property
            def in_waiting(self):return 0 if any(x!=velocity_frame(0,0,0) for x in self.writes) else 32
        for port_class,kwargs,error in ((Port,{'reverse':True},'negative forward'),(Overspeed,{},'feedback exceeded'),(MissingOnMotion,{},'feedback lost')):
            clock=Clock();port=port_class(clock,sonar=195,**kwargs)
            result=run_check(port,speed=.2,duration=3,ground=True,straight_test=True,measured_clearance=2.,clock=clock.time,pause=clock.sleep)
            self.assertIn(error,result['error'])
            self.assertLess(result['motion_elapsed_s'],.4)
            self.assertEqual(port.writes[-1],velocity_frame(0,0,0))


if __name__=='__main__':unittest.main()
