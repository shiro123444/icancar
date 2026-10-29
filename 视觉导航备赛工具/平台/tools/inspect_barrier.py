#!/usr/bin/env python3
"""Draw candidate barrier boxes on a saved frame for offline calibration."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from automation.barrier_detection import detect_barriers, validate_config


def main():
    p=argparse.ArgumentParser(description='在已保存的画面上查看挡板候选框')
    p.add_argument('--image',required=True,type=Path)
    p.add_argument('--config',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path,help='新建 PNG，不覆盖已有文件')
    args=p.parse_args()
    if args.output.exists() or args.output.suffix.lower()!='.png':
        p.error('输出必须是不存在的 PNG 文件')
    try:
        import cv2
    except ImportError as error:
        raise SystemExit('需要 OpenCV：'+str(error))
    with args.config.open(encoding='utf-8') as handle:config=validate_config(json.load(handle))
    image=cv2.imread(str(args.image),cv2.IMREAD_COLOR)
    if image is None:raise SystemExit('无法读取图像')
    result=detect_barriers(cv2.cvtColor(image,cv2.COLOR_BGR2RGB),config)
    height,width=image.shape[:2]
    for item in result['boxes']:
        x1,y1,x2,y2=item['box']
        cv2.rectangle(image,(int(x1*width),int(y1*height)),(int(x2*width),int(y2*height)),(0,0,255),2)
    labels=[('CAL: YES' if result['calibrated'] else 'CAL: NO')+'   BOX: '+str(len(result['boxes'])),
            'LANE ROWS: '+str(round(result['valid_row_fraction']*100))+'%']
    for index,label in enumerate(labels):
        position=(6,18+index*18)
        cv2.putText(image,label,position,cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(image,label,position,cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),1,cv2.LINE_AA)
    ok,encoded=cv2.imencode('.png',image)
    if not ok:raise SystemExit('无法生成 PNG')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('xb') as out:out.write(encoded.tobytes())
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    print('调试图片：'+str(args.output))


if __name__=='__main__':main()
