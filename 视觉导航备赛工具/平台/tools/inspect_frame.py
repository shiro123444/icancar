#!/usr/bin/env python3
"""Inspect detection regions on a still image without connecting a vehicle."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PIL import Image, ImageDraw
from automation.detector import detect


def main():
    parser = argparse.ArgumentParser(description='检查起点线与人行道的图像检测区域')
    parser.add_argument('image', help='摄像头图片路径')
    parser.add_argument('--config', required=True, help='检测配置 JSON')
    parser.add_argument('--output', help='保存标记检测区域的图片')
    args = parser.parse_args()
    with open(args.config, encoding='utf-8') as f:
        config = json.load(f)
    with Image.open(args.image) as source:
        image = source.convert('RGB')
    result = detect(image, config)
    print(json.dumps({'calibrated': config['calibrated'], 'detections': result}, ensure_ascii=False, indent=2))
    if args.output:
        draw = ImageDraw.Draw(image)
        for label, color in (('start_line', 'red'), ('crosswalk', 'cyan')):
            item = result[label]
            draw.rectangle(item['rect'], outline=color, width=3)
            draw.text((item['rect'][0] + 4, item['rect'][1] + 4),
                      '%s: %s bands=%s' % (label, item['detected'], item['bands']), fill=color)
        image.save(args.output)


if __name__ == '__main__':
    main()
