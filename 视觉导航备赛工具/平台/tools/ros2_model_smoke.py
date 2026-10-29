#!/usr/bin/env python3
"""Isolated ROS 2 replay of two laps using synthetic crosswalk-model messages."""
import io
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '78' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('仅允许在本机隔离 ROS 2 域 78 中运行')
    from PIL import Image, ImageDraw
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    root = Path(__file__).resolve().parents[1]
    def jpeg(line):
        picture=Image.new('RGB',(640,480),'black')
        if line: ImageDraw.Draw(picture).rectangle((70,395,570,420),fill='white')
        out=io.BytesIO();picture.save(out,format='JPEG');return out.getvalue()
    scenes={'clear':jpeg(False),'line':jpeg(True)}
    with tempfile.TemporaryDirectory(prefix='visual-race-model-smoke-') as tmp:
        folder=Path(tmp)
        stripe=json.loads((root/'data/perception.example.json').read_text(encoding='utf-8'))
        model=json.loads((root/'data/crosswalk-model.example.json').read_text(encoding='utf-8'))
        stripe['calibrated']=model['calibrated']=True  # Only in this disposable synthetic test.
        stripe_path,model_path=folder/'stripe.json',folder/'model.json'
        stripe_path.write_text(json.dumps(stripe),encoding='utf-8')
        model_path.write_text(json.dumps(model),encoding='utf-8')
        cmd=[sys.executable,str(root/'ros2/automatic_race.py'), '--camera-topic','/trial/model_camera',
             '--camera-type','compressed','--odom-topic','/trial/model_odom','--config',str(stripe_path),
             '--crosswalk-source','model','--model-config',str(model_path), '--model-topic','/trial/model_results',
             '--status-topic','/trial/model_status','--speech-topic','/trial/model_speech',
             '--speech-ack-topic','/trial/model_speech_ack']
        with tempfile.TemporaryFile(mode='w+t') as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
            rclpy.init()
            class Trial(Node):
                def __init__(self):
                    super().__init__('visual_race_model_replay')
                    self.camera=self.create_publisher(CompressedImage,'/trial/model_camera',qos_profile_sensor_data)
                    self.odom=self.create_publisher(Odometry,'/trial/model_odom',qos_profile_sensor_data)
                    self.model=self.create_publisher(String,'/trial/model_results',1)
                    self.acks=self.create_publisher(String,'/trial/model_speech_ack',1)
                    self.create_subscription(String,'/trial/model_status',self.status,1)
                    self.create_subscription(String,'/trial/model_speech',self.speech,1)
                    self.arm=self.create_client(Trigger,'/visual_race/arm')
                    self.reset=self.create_client(Trigger,'/visual_race/reset')
                    self.create_timer(.1,self.tick)
                    self.stage,self.scene,self.crosswalk='warmup','clear',False
                    self.state,self.future,self.finish={},None,None
                    self.stop_count=0
                    self.model_enabled=True
                def status(self,msg):
                    try:self.state=json.loads(msg.data)
                    except (TypeError,ValueError):pass
                def speech(self,msg):
                    try:
                        value=json.loads(msg.data)
                        if value.get('kind')=='crosswalk' and value.get('id'):
                            self.acks.publish(String(data=json.dumps({'id':value['id'],'success':True})))
                    except (TypeError,ValueError):pass
                def change(self,stage,scene='clear',crosswalk=False):
                    self.stage,self.scene,self.crosswalk=stage,scene,crosswalk
                def tick(self):
                    stamp=self.get_clock().now().to_msg()
                    stamp_ns=stamp.sec*1_000_000_000+stamp.nanosec
                    image=CompressedImage();image.header.stamp=stamp;image.format='jpeg';image.data=scenes[self.scene]
                    odom=Odometry();odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.0
                    self.camera.publish(image);self.odom.publish(odom)
                    if self.model_enabled:
                        boxes=([{'label':'crossing','score':.9,'box':[200,120,420,360]}] if self.crosswalk else [])
                        value={'source':'openvino_race','valid':True,'stamp_ns':stamp_ns,'width':640,'height':480,
                               'inference_ms':1,'detections':boxes}
                        self.model.publish(String(data=json.dumps(value)))
                    s=self.state
                    if s.get('phase') in ('yield','finished','fault') and s.get('action')!='stop':
                        self.finish={'error':'controller did not request stop'};return
                    clear=s.get('line') is False and s.get('crosswalk') is False
                    if self.stage in ('warmup','warmup_loss') and clear and self.arm.service_is_ready():
                        self.future=self.arm.call_async(Trigger.Request())
                        self.change('arming' if self.stage=='warmup' else 'arming_loss')
                    elif self.stage in ('arming','arming_loss') and self.future.done():
                        response=self.future.result()
                        if not response.success:self.finish={'error':'arm rejected: '+response.message}
                        else:self.change('start' if self.stage=='arming' else 'start_loss','line')
                    elif self.stage=='start' and s.get('phase')=='running':self.change('before_crosswalk_1')
                    elif self.stage=='before_crosswalk_1' and clear:self.change('crosswalk_1',crosswalk=True)
                    elif self.stage=='crosswalk_1' and s.get('phase')=='yield':
                        self.stop_count+=1;self.change('waiting_1',crosswalk=True)
                    elif self.stage=='waiting_1' and s.get('phase')=='running' and s.get('crosswalk_stops',0)>=1:self.change('before_lap_1')
                    elif self.stage=='before_lap_1' and clear:self.change('lap_1','line')
                    elif self.stage=='lap_1' and s.get('laps')==1:self.change('before_crosswalk_2')
                    elif self.stage=='before_crosswalk_2' and clear:self.change('crosswalk_2',crosswalk=True)
                    elif self.stage=='crosswalk_2' and s.get('phase')=='yield':
                        self.stop_count+=1;self.change('waiting_2',crosswalk=True)
                    elif self.stage=='waiting_2' and s.get('phase')=='running' and s.get('crosswalk_stops',0)>=2:self.change('before_lap_2')
                    elif self.stage=='before_lap_2' and clear:self.change('lap_2','line')
                    elif self.stage=='lap_2' and s.get('phase')=='finished':
                        if s.get('laps')!=2 or s.get('crosswalk_stops')!=2 or self.stop_count!=2:
                            self.finish={'error':'two-lap checks failed'}
                        elif self.reset.service_is_ready():
                            self.future=self.reset.call_async(Trigger.Request());self.change('resetting')
                    elif self.stage=='resetting' and self.future.done():
                        response=self.future.result()
                        if not response.success:self.finish={'error':'reset rejected: '+response.message}
                        else:self.change('warmup_loss')
                    elif self.stage=='start_loss' and s.get('phase')=='running':
                        self.model_enabled=False;self.change('model_loss')
                    elif self.stage=='model_loss' and s.get('phase')=='fault':
                        self.finish=({'two_laps':True,'crosswalk_stops':2,'model_loss_stop':True,
                                      'simulated_model_and_feedback':True,'physical_commands':False}
                                     if s.get('reason')=='crosswalk_model_unavailable'
                                     else {'error':'unexpected fault: '+str(s.get('reason'))})
                    elif s.get('phase')=='fault' and self.stage!='model_loss':
                        self.finish={'error':'controller fault: '+str(s.get('reason'))}
            node=None
            try:
                node=Trial()
                deadline=time.monotonic()+40
                while time.monotonic()<deadline and node.finish is None and child.poll() is None:
                    rclpy.spin_once(node,timeout_sec=.2)
                result=node.finish or {'error':'timeout or controller exited at '+node.stage}
                if 'error' in result:
                    log.seek(0);result['controller_log']=log.read()[-2000:]
                print(json.dumps(result,ensure_ascii=False),flush=True)
                if 'error' in result:raise SystemExit(1)
            finally:
                if node is not None:node.destroy_node()
                rclpy.shutdown();child.terminate()
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:child.kill();child.wait()


if __name__=='__main__':main()
