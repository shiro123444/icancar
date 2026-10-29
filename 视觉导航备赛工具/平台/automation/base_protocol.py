"""DEEPBOT x4 serial framing based on the vehicle's installed driver source."""
import math
import struct

HEADER = b'\xab\xcd'


class FrameDecoder:
    def __init__(self):
        self.buffer=bytearray()
    def feed(self,data):
        self.buffer.extend(data)
        frames=[]
        while len(self.buffer)>=3:
            pos=self.buffer.find(HEADER)
            if pos<0:
                self.buffer[:]=self.buffer[-1:] if self.buffer[-1:]==HEADER[:1] else b''
                break
            if pos: del self.buffer[:pos]
            if len(self.buffer)<3: break
            length=self.buffer[2]
            if length<4:
                del self.buffer[0]
                continue
            if len(self.buffer)<length: break
            frame=bytes(self.buffer[:length])
            if sum(frame[:-1]) & 255 != frame[-1]:
                del self.buffer[0]
                continue
            del self.buffer[:length]
            frames.append(frame)
        if len(self.buffer)>4096: self.buffer.clear()
        return frames


def measured_velocity(frame):
    if len(frame)<31 or frame[:2]!=HEADER or frame[2]!=len(frame) or sum(frame[:-1]) & 255 != frame[-1]:
        raise ValueError('invalid base sensor frame')
    return tuple(value/1000.0 for value in struct.unpack_from('>hhh',frame,16))


def velocity_frame(vx,vy,yaw):
    if any(not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) or abs(v)>32 for v in (vx,vy,yaw)):
        raise ValueError('invalid velocity')
    data=HEADER+bytes((11,0x11))+struct.pack('>hhh',*(int(v*1000) for v in (vx,vy,yaw)))
    return data+bytes((sum(data)&255,))
