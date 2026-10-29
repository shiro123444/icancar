#!/usr/bin/env python3
"""Inspect a CSV driving dataset without modifying it."""
import argparse
import csv
import json
from pathlib import Path

def audit(path, limit=10000):
    path=path.expanduser().resolve()
    if not path.is_file(): raise ValueError('CSV 不存在')
    if limit<1: raise ValueError('样本检查数必须大于 0')
    result={'csv':str(path),'columns':[],'rows_checked':0,'existing_image_paths':0,'missing_image_paths':0,'blank_image_paths':0,'truncated':False,'notes':[]}
    with path.open(encoding='utf-8-sig',newline='') as f:
        reader=csv.reader(f)
        header=next(reader,None)
        if header is None: raise ValueError('CSV 为空')
        result['columns']=header
        # The video shows image path as column 0, but actual schema needs confirmation.
        result['notes'].append('依据视频示例按第一列检查图片路径；请核对真实 CSV 列含义。')
        for row in reader:
            if result['rows_checked'] >= limit:
                result['truncated']=True;break
            result['rows_checked']+=1
            value=row[0].strip() if row else ''
            if not value:
                result['blank_image_paths']+=1;continue
            image=Path(value).expanduser()
            if not image.is_absolute(): image=path.parent/image
            result['existing_image_paths' if image.is_file() else 'missing_image_paths']+=1
    return result

def main():
    p=argparse.ArgumentParser(description='只读检查采集 CSV 和图像路径（不训练、不删除文件）')
    p.add_argument('csv',type=Path);p.add_argument('--max-rows',type=int,default=10000)
    a=p.parse_args()
    try: print(json.dumps(audit(a.csv,a.max_rows),ensure_ascii=False,indent=2))
    except (OSError,ValueError,csv.Error) as e:p.exit(2,f'检查失败：{e}\n')
if __name__=='__main__':main()
