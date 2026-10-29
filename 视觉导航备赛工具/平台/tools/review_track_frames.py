#!/usr/bin/env python3
"""Replay saved camera images into numeric observations; no ROS or hardware I/O."""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from automation.detector import detect,validate_config
from automation.supervised_follow import view_quality
from automation.visual_route import observe_lane


def review(folder,config):
    paths=sorted(p for p in folder.rglob('*') if p.suffix.lower() in ('.jpg','.jpeg','.png') and p.is_file() and not p.is_symlink())
    if len(paths)>2000:raise ValueError('too many review images')
    cv2.setNumThreads(1);records=[];timings=[]
    for path in paths:
        if path.stat().st_size>10_000_000:raise ValueError('image exceeds size limit: '+path.name)
        content=path.read_bytes();bgr=cv2.imdecode(np.frombuffer(content,np.uint8),cv2.IMREAD_COLOR)
        if bgr is None or bgr.size>48_000_000:raise ValueError('invalid image: '+path.name)
        rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        started=time.monotonic();lane=observe_lane(rgb);quality=view_quality(rgb);stripes=detect(rgb,config)
        elapsed=(time.monotonic()-started)*1000;timings.append(elapsed)
        records.append(dict(image_path=str(path.relative_to(folder)),sha256=hashlib.sha256(content).hexdigest(),
            image_size=[rgb.shape[1],rgb.shape[0]],lane=lane,quality=quality,stripes=stripes,
            processing_ms=round(elapsed,3),training_label_valid=False))
    return dict(source='saved camera images',read_only=True,physical_motor_commands=False,
        frames=len(records),valid_paired_lane_frames=sum(v['lane']['valid'] for v in records),
        blue_candidate_frames=sum(bool(v['lane']['blue_boxes']) for v in records),
        obscured_frames=sum(not v['quality']['visible'] for v in records),
        processing_ms_p50=round(float(np.percentile(timings,50)),3) if timings else None,
        processing_ms_p95=round(float(np.percentile(timings,95)),3) if timings else None,
        benchmark_scope='local image processing only; excludes inference, network and vehicle response',
        review_labels_verified=False,records=records)


def main():
    parser=argparse.ArgumentParser(description='现场历史图片的车道、白线与遮挡回放')
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'data/perception.site-review.json')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not args.input.is_dir() or args.output.exists():parser.error('采集目录无效或输出文件已经存在')
    config=validate_config(json.loads(args.config.read_text()))
    result=review(args.input.resolve(),config)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as out:json.dump(result,out,ensure_ascii=False,allow_nan=False,indent=2);out.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='records'},ensure_ascii=False))


if __name__=='__main__':main()
