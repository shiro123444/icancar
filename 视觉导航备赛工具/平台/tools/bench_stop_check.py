#!/usr/bin/env python3
"""Short DEEPBOT stop checks with explicit stand or supervised-clearance setup."""
import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from automation.base_protocol import FrameDecoder, measured_sensors, measured_velocity, velocity_frame


def validate_limits(speed, duration, ground=False, larger_step=False, three_second=False, watchdog=False, speed_step=False, straight_test=False, measured_clearance=None):
    if (sum((larger_step,three_second,speed_step,straight_test))>1 or (larger_step or three_second or speed_step or straight_test) and not ground
            or (three_second or speed_step or straight_test) and watchdog):
        raise ValueError('incompatible bench test setup')
    if straight_test and (measured_clearance is None or not math.isfinite(measured_clearance) or measured_clearance<2):
        raise ValueError('supervised straight test requires a measured clear path of at least 2 m')
    speed_limit=(.2 if straight_test else .08 if speed_step else .05 if larger_step or three_second else .02) if ground else .05
    duration_limit=(3 if three_second or speed_step or straight_test else .6 if larger_step else .4) if ground else 1
    distance_limit=.6 if straight_test else .24 if speed_step else .15 if three_second else .03 if larger_step else .008
    if (not 0 < speed <= speed_limit or not 0 < duration <= duration_limit
            or (ground and speed*duration>distance_limit+1e-9)):
        raise ValueError('bench limits exceeded')


def run_check(port, speed=.03, duration=.8, watchdog=False, clock=time.monotonic, pause=time.sleep, ground=False, larger_step=False, three_second=False, speed_step=False, straight_test=False, measured_clearance=None):
    validate_limits(speed,duration,ground,larger_step,three_second,watchdog,speed_step,straight_test,measured_clearance)
    long_ground=three_second or speed_step
    # Only this supervised, measured-clearance actuator test treats raw sonar as
    # diagnostic data. It neither enables autonomous driving nor publishes safety
    # permission. All other ground profiles retain their range interlocks.
    startup_range=(135,175) if speed_step else (85,115) if three_second else (45,65)
    motion_range=(75,175) if speed_step else (60,115) if three_second else (45,65)
    decoder=FrameDecoder();latest=None;received=None;samples=0;peak=0.;nonzero=0
    range_ok=bool(straight_test);range_bad_at=None;range_rejections=0;last_sonar=None
    motion_started=None;motion_ended=None;motion_active=False
    near_pending=None;transient_dips=0
    startup_deadline=8 if speed_step or straight_test else 4
    result={'physical_motor_test':True,'setup':'ground_clearance_operator_at_power' if ground else 'wheel_stand',
            'test':'firmware-watchdog' if watchdog else 'forward-stop',
            'requested_speed_mps':speed,'requested_duration_s':duration,
            'startup_deadline_s':startup_deadline,
            'stop_feedback_confirmed':False,'firmware_timeout_observed':False}
    if ground and not straight_test:
        result.update(startup_sonar_cm=list(startup_range),motion_sonar_cm=list(motion_range))
    if long_ground:
        result.update(emergency_sonar_cm=30,near_confirmation_frames=2,near_confirmation_deadline_s=.05)
    if straight_test:
        result.update(setup='supervised_measured_straight',operator_measured_clearance_m=measured_clearance,
                      clearance_source='operator measurement and camera',sonar_mode='diagnostic_only',
                      range_interlock_enabled=False,speed_step_per_command_mps=.04)

    def send(values):
        nonlocal nonzero
        data=velocity_frame(*values)
        if port.write(data)!=len(data):raise RuntimeError('short serial write')
        if values!=(0.,0.,0.):nonzero+=1

    def receive(allow_unstable=False):
        nonlocal latest,received,samples,peak,range_ok,range_bad_at,range_rejections,last_sonar,near_pending,transient_dips
        if near_pending is not None and clock()-near_pending>=.05:
            near_pending=None;range_ok=False;range_bad_at=clock()
            if not allow_unstable:raise RuntimeError('near-distance confirmation deadline expired')
        changed=False
        for frame in decoder.feed(port.read(min(port.in_waiting,8192))):
            try:values=measured_velocity(frame)
            except ValueError:continue
            latest,received=values,clock();samples+=1;changed=True
            if ground:
                last_sonar=measured_sensors(frame)['sonar_vendor_value']
            if ground and not straight_test:
                low,high=motion_range if motion_active else startup_range
                if low<=last_sonar<=high:
                    range_ok=True
                    if near_pending is not None:transient_dips+=1;near_pending=None
                else:
                    range_rejections+=1
                    # Keep prior verified clearance through at most one short dip.
                    # Startup cannot release while confirmation is pending. Two
                    # near frames, <=30 cm, invalid/far values or a 50 ms timeout
                    # block startup or stop motion, without automatic motion retry.
                    if long_ground and 30<last_sonar<low and near_pending is None:
                        near_pending=clock()
                    else:
                        range_ok=False;range_bad_at=clock();near_pending=None
                        if not allow_unstable:
                            raise RuntimeError('ground test requires the verified upright board at %g..%g cm sonar distance; observed %.1f cm' % (low,high,last_sonar))
            peak=max(peak,math.hypot(values[0],values[1]))
            feedback_limit=(.3 if straight_test else .14 if speed_step else .1 if larger_step or three_second else .05) if ground else .12
            if math.hypot(values[0],values[1])>feedback_limit or abs(values[2])>(.1 if ground else .35):
                raise RuntimeError('feedback exceeded bench limit')
        return changed

    def stopped():
        return latest is not None and math.hypot(latest[0],latest[1])<=(.004 if ground else .01) and abs(latest[2])<=.03

    def fresh():
        return received is not None and 0<=clock()-received<=.3

    try:
        start=clock();sent=-float('inf');clear_since=None
        while clock()-start<startup_deadline:
            if clock()-sent>=.1:send((0.,0.,0.));sent=clock()
            receive(allow_unstable=True)
            if samples>=5 and fresh() and stopped() and (not ground or range_ok):
                if clear_since is None or (range_bad_at is not None and range_bad_at>=clear_since):clear_since=clock()
                if clock()-clear_since>=.8 and near_pending is None:break
            else:clear_since=None
            pause(.01)
        if (clear_since is None or clock()-clear_since<.8 or near_pending is not None
                or not fresh() or not stopped() or ground and not range_ok):
            raise RuntimeError('fresh stationary feedback for 0.8 s required' if straight_test else 'fresh stationary feedback and 0.8 s stable clearance required')
        peak=0.
        motion_active=True;motion_started=clock()
        start=clock();sent=-float('inf')
        while clock()-start<duration:
            receive()
            if not fresh():raise RuntimeError('feedback lost during bench movement')
            if latest[0]<-.01:raise RuntimeError('forward command produced negative forward feedback')
            if near_pending is None and clock()-sent>=.1:
                target=min(speed,.04*(nonzero+1)) if straight_test else speed
                send((target,0.,0.));sent=clock()
            pause(.01)
        motion_ended=clock()
        if peak<.01:
            result['inconclusive']='wheel movement was not observed in encoder feedback'
        last_motion_sent=sent
        start=clock();sent=-float('inf');clear=0
        silence=watchdog and peak>=.01 and not stopped()
        if watchdog and not silence:result['inconclusive']='moving feedback was not present when heartbeat withholding began'
        stop_limit=.8 if ground else 1.2
        while clock()-start<stop_limit:
            if not silence and clock()-sent>=.1:send((0.,0.,0.));sent=clock()
            changed=receive()
            if not fresh():raise RuntimeError('feedback lost while checking stop')
            if changed:clear=clear+1 if stopped() else 0
            if clear>=3:
                result['stop_feedback_confirmed']=True
                result['stop_feedback_delay_s']=round(clock()-start,3)
                result['firmware_timeout_observed']=bool(silence)
                if silence:result['heartbeat_silence_s']=round(clock()-last_motion_sent,3)
                break
            pause(.01)
        if not result['stop_feedback_confirmed']:result['error']='stationary feedback not confirmed within stop deadline'
    except Exception as error:
        result['error']=str(error)
    finally:
        if motion_started is not None:
            result['motion_elapsed_s']=round((motion_ended if motion_ended is not None else clock())-motion_started,3)
        # Explicit zero remains the fallback even when the firmware test fails.
        for _ in range(3):
            try:send((0.,0.,0.))
            except Exception:pass
            pause(.03)
        result.update(samples=samples,peak_linear_feedback=round(peak,4),nonzero_commands=nonzero,
                      range_rejections=range_rejections,last_sonar_cm=last_sonar,
                      transient_sonar_dips=transient_dips)
    return result


def main():
    parser=argparse.ArgumentParser(description='在架空或有实测距离保护及现场断电人员的条件下检查低速停止反馈')
    parser.add_argument('--device',required=True)
    parser.add_argument('--wheels-lifted',action='store_true')
    parser.add_argument('--ground-clearance',action='store_true')
    parser.add_argument('--larger-ground-step',action='store_true',help='显式选择已说明的最多约 3 cm 地面短步')
    parser.add_argument('--three-second-ground-test',action='store_true',help='1 m 实物挡板及现场看护：最多 3 秒、0.05 m/s、理论 15 cm，发零速停车')
    parser.add_argument('--speed-step-ground-test',action='store_true',help='1.5 m 实物挡板及现场看护：最多 3 秒、0.08 m/s、理论 24 cm，发零速停车')
    parser.add_argument('--supervised-straight-test',action='store_true',help='实测前方至少 2 m 空地及现场看护：最多 3 秒、0.2 m/s，逐级加速并发零速停车')
    parser.add_argument('--measured-clearance-m',type=float,help='现场实际确认的前方空地长度；不由超声原始值替代')
    parser.add_argument('--operator-at-power-switch',action='store_true')
    parser.add_argument('--run-motion-test',action='store_true')
    parser.add_argument('--speed',type=float,default=.03)
    parser.add_argument('--duration',type=float,default=.8)
    parser.add_argument('--test',choices=('forward-stop','firmware-watchdog'),default='forward-stop')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.wheels_lifted==args.ground_clearance or not args.run_motion_test:
        parser.error('需明确一种现场准备方式，并显式启用测试')
    if args.ground_clearance and not args.operator_at_power_switch:parser.error('地面测试需现场人员守在电源开关旁')
    try:validate_limits(args.speed,args.duration,args.ground_clearance,args.larger_ground_step,args.three_second_ground_test,args.test=='firmware-watchdog',args.speed_step_ground_test,args.supervised_straight_test,args.measured_clearance_m)
    except ValueError as error:parser.error(str(error))
    if args.output and args.output.exists():parser.error('输出文件已存在')
    import serial
    with serial.Serial(args.device,115200,timeout=0,write_timeout=.2,exclusive=True) as port:
        result=run_check(port,args.speed,args.duration,args.test=='firmware-watchdog',ground=args.ground_clearance,larger_step=args.larger_ground_step,three_second=args.three_second_ground_test,speed_step=args.speed_step_ground_test,straight_test=args.supervised_straight_test,measured_clearance=args.measured_clearance_m)
    text=json.dumps(result,ensure_ascii=False,allow_nan=False,indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x',encoding='utf-8') as out:out.write(text+'\n')
    print(text,flush=True)
    if 'error' in result:raise SystemExit(1)


if __name__=='__main__':main()
