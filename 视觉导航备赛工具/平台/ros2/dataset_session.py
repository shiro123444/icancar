"""Filesystem-only recording primitives for synchronized driving samples."""
import csv
import json
import math
from pathlib import Path

class DatasetSession:
    def __init__(self, output, metadata, *, min_interval_s=0.2, max_command_age_s=0.25):
        self.path=Path(output).expanduser()
        if not 0 < min_interval_s <= 10 or not 0 < max_command_age_s <= 2:
            raise ValueError('采样间隔或指令有效期不在允许范围')
        self.interval=min_interval_s;self.max_age=max_command_age_s
        self.last_capture=None;self.command=None;self.count=0;self.skipped_stale=0
        # New output directory only. Never append to an existing training dataset.
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path/'images').mkdir()
        (self.path/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
        self.file=(self.path/'data.csv').open('x',encoding='utf-8',newline='')
        self.csv=csv.DictWriter(self.file,fieldnames=['image','linear_x','linear_y','angular_z','command_age_ms','image_stamp_ns'])
        self.csv.writeheader();self.file.flush()
    def set_command(self, linear_x, linear_y, angular_z, now):
        values=(linear_x,linear_y,angular_z,now)
        if not all(math.isfinite(float(x)) for x in values): raise ValueError('控制量不是有限数字')
        self.command=(float(linear_x),float(linear_y),float(angular_z),float(now))
    def add_frame(self,jpeg,image_stamp_ns,now):
        if not math.isfinite(float(now)): raise ValueError('时间无效')
        if self.last_capture is not None and now-self.last_capture < self.interval: return False
        if self.command is None or not 0 <= now-self.command[3] <= self.max_age:
            self.skipped_stale+=1;return False
        if not isinstance(jpeg,(bytes,bytearray)) or not bytes(jpeg).startswith(b'\xff\xd8') or len(jpeg)>10_000_000:
            raise ValueError('需要有效大小的 JPEG 数据')
        name=f'images/frame_{self.count+1:06d}.jpg'
        with (self.path/name).open('xb') as image:image.write(jpeg)
        row={'image':name,'linear_x':self.command[0],'linear_y':self.command[1],
             'angular_z':self.command[2], 'command_age_ms':round((now-self.command[3])*1000,2),
             'image_stamp_ns':int(image_stamp_ns)}
        self.csv.writerow(row);self.file.flush();self.count+=1;self.last_capture=float(now)
        return True
    def close(self):self.file.close()
