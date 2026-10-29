"""Read-only lane geometry and route-constrained gap proposals.

Pixel clearances are provisional. This module does not certify physical clearance
and does not publish commands. A second barrier cannot silently switch to right.
"""
import math
import numpy as np


def observe_lane(image):
    import cv2
    a=np.asarray(image)
    if a.dtype!=np.uint8 or a.ndim!=3 or a.shape[2]!=3 or min(a.shape[:2])<32:raise ValueError('RGB image required')
    h,w=a.shape[:2];hsv=cv2.cvtColor(a,cv2.COLOR_RGB2HSV)
    yellow=cv2.inRange(hsv,np.array([18,70,70],np.uint8),np.array([42,255,255],np.uint8))>0
    rows=[]
    for f in (.50,.56,.62,.68,.74,.80,.86):
        y=int(h*f);row=yellow[max(0,y-2):min(h,y+3)].mean(axis=0)>.4
        d=np.diff(np.r_[False,row,False].astype(np.int8));runs=list(zip(np.flatnonzero(d==1),np.flatnonzero(d==-1)))
        runs=[(x/w,z/w) for x,z in runs if z-x>=max(2,w*.004)]
        left=[z for x,z in runs if z<.5];right=[x for x,z in runs if x>.5]
        if left and right and min(right)-max(left)>.15:rows.append([f,max(left),min(right)])
    if len(rows)<3:return dict(valid=False,confidence=len(rows)/7,rows=rows,blue_boxes=[])
    r=np.array(rows);lf=np.polyfit(r[:,0],r[:,1],1);rf=np.polyfit(r[:,0],r[:,2],1)
    lo,hi=float(np.polyval(lf,.74)),float(np.polyval(rf,.74));near=(lo+hi)/2
    far=float((np.polyval(lf,.54)+np.polyval(rf,.54))/2)
    lateral=(.5-near)/max(.15,hi-lo);heading=math.atan2(near-far,.20)
    hint=max(-.65,min(.65,1.3*lateral+.8*heading))
    blue=cv2.inRange(hsv,np.array([90,60,35],np.uint8),np.array([135,255,255],np.uint8))
    corridor=np.zeros((h,w),np.uint8)
    # Clip actual blue pixels to the observed road. A bounding box around the
    # connected blue background otherwise spans both rails and looks like a wall.
    for y in range(int(h*r[:,0].min()),int(h*.90)):
        low=max(0,int((np.polyval(lf,y/h)+.025)*w));high=min(w,int((np.polyval(rf,y/h)-.025)*w))
        if high>low:corridor[y,low:high]=blue[y,low:high]
    count,_,stats,_=cv2.connectedComponentsWithStats(corridor,connectivity=8);boxes=[]
    for x,y,bw,bh,area in stats[1:]:
        bottom=(y+bh)/h
        if area<h*w*.003 or bh<h*.04 or not .4<=bottom<=.98:continue
        l=max(0.,float(np.polyval(lf,bottom)));rr=min(1.,float(np.polyval(rf,bottom)))
        overlap=max(0.,min((x+bw)/w,rr-.01)-max(x/w,l+.01))
        if overlap>.06:boxes.append(dict(box=[float(x/w),float(y/h),float((x+bw)/w),float(bottom)],lane_overlap=float(overlap)))
    return dict(valid=True,confidence=len(rows)/7,rows=rows,lane=[max(0.,lo),min(1.,hi)],
                near_center=near,far_center=far,lateral_error=lateral,heading_error=heading,
                left_margin=(.5-lo)/max(.15,hi-lo),right_margin=(hi-.5)/max(.15,hi-lo),
                steering_hint=hint,blue_boxes=boxes)


class RouteGapPlanner:
    def __init__(self,sequence=('right','left'),min_gap_ratio=.45,padding=.025):
        if not sequence or any(s not in ('left','right') for s in sequence):raise ValueError('invalid barrier route')
        if not .2<=min_gap_ratio<=.8 or not 0<=padding<=.1:raise ValueError('invalid image gap settings')
        self.sequence=tuple(sequence);self.min_gap_ratio=min_gap_ratio;self.padding=padding
        self.encounters=0;self.active=False;self.side=None;self.last_seen=None;self.enter_distance=0.
    def propose(self,observation,distance,model_steering):
        if not math.isfinite(distance) or not math.isfinite(model_steering):raise ValueError('finite inputs required')
        if not observation.get('valid'):return dict(mode='model_only',steering=model_steering,speed_cap=.12,review_required=True)
        lane=observation['lane'];boxes=[x['box'] for x in observation.get('blue_boxes',[]) if x['box'][3]>=.52]
        if boxes:
            if not self.active:
                self.active=True;self.encounters+=1;self.enter_distance=distance
                self.side=self.sequence[(self.encounters-1)%len(self.sequence)]
            self.last_seen=distance
            intervals=[tuple(lane)]
            for box in boxes:
                a,b=box[0]-self.padding,box[2]+self.padding;remaining=[]
                for lo,hi in intervals:
                    if b<=lo or a>=hi:remaining.append((lo,hi))
                    else:
                        if a>lo:remaining.append((lo,min(a,hi)))
                        if b<hi:remaining.append((max(b,lo),hi))
                intervals=remaining
            width=lane[1]-lane[0];targets=[]
            for lo,hi in intervals:
                if hi-lo<width*self.min_gap_ratio:continue
                target=(lo+hi)/2;side='left' if target<.5 else 'right'
                if side==self.side:targets.append((abs(target-.5),target,[lo,hi]))
            if not targets:return dict(mode='route_gap_unresolved',barrier_index=self.encounters,
                required_side=self.side,steering=None,speed_cap=0.,review_required=True)
            _,target,gap=min(targets)
            return dict(mode='barrier_route',barrier_index=self.encounters,required_side=self.side,
                target_x=target,gap=gap,steering=max(-.65,min(.65,2.0*(.5-target))),speed_cap=.12,review_required=True)
        if self.active and self.last_seen is not None and distance-self.last_seen>.45:self.active=False
        confidence=observation['confidence'];weight=.5*confidence
        steering=(1-weight)*model_steering+weight*observation['steering_hint']
        margin=min(observation['left_margin'],observation['right_margin'])
        return dict(mode='lane_centering',steering=max(-.65,min(.65,steering)),
                    speed_cap=.12 if margin<.24 or abs(observation['heading_error'])>.35 else .2,
                    barrier_index=self.encounters,review_required=True)
