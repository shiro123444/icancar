#!/usr/bin/env python3
"""Extract vector yellow track contours from a reference photo; no registration guess."""
import argparse
import hashlib
import json
from pathlib import Path


def scan(path):
    import cv2
    import numpy as np
    raw=Path(path).read_bytes();a=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    if a is None or a.shape[0]*a.shape[1]>20000000:raise ValueError('invalid photo')
    hsv=cv2.cvtColor(a,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([18,70,70],np.uint8),np.array([42,255,255],np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    lines=[]
    for c in contours:
        if cv2.arcLength(c,True)<100 or cv2.contourArea(c)<80:continue
        points=cv2.approxPolyDP(c,2.0,True).reshape(-1,2).tolist()
        if len(points)>=4:lines.append(points)
    return dict(version=1,image_sha256=hashlib.sha256(raw).hexdigest(),image_size=[a.shape[1],a.shape[0]],
                coordinate_space='photo_pixels',yellow_contours=lines,map_registration=None,
                registration_state='awaiting_correspondences',metric_calibrated=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--image',required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();value=scan(args.image);args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as out:json.dump(value,out,ensure_ascii=False,indent=2);out.write('\n')
    print(json.dumps({'contours':len(value['yellow_contours']),'points':sum(map(len,value['yellow_contours'])),'coordinate_space':value['coordinate_space']}))
