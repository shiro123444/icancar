#!/usr/bin/env python3
"""Replay local task-2 events; does not command or access any robot."""
import argparse
import json
import sys
from pathlib import Path
from task2 import Mission

def main():
 p=argparse.ArgumentParser(description='离线重放赛题事件，不连接小车')
 p.add_argument('events',type=Path,help='JSONL，每行包含 t、event；motion 另含 value')
 args=p.parse_args();mission=Mission()
 try:
  with args.events.open(encoding='utf-8') as f:
   for number,line in enumerate(f,1):
    if not line.strip():continue
    try:
     event=json.loads(line);state=mission.process(event['event'],event['t'],event.get('value'))
     print(json.dumps({'line':number,'event':event['event'],**state},ensure_ascii=False))
    except (ValueError,TypeError,KeyError,json.JSONDecodeError) as exc:raise ValueError(f'第 {number} 行：{exc}') from exc
 except (OSError,ValueError) as exc:p.exit(2,f'重放失败：{exc}\n')
if __name__=='__main__':main()
