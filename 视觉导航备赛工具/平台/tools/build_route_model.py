#!/usr/bin/env python3
"""Derive a sampled, directed centreline from the existing vector track layout."""
import json
import math
from pathlib import Path


def build():
    # Control points lie between the existing inner and outer vector boundaries.
    sections=[('start_straight','起点至人行道',[[458,420],[458,148]]),
      ('upper_right','右上弯道',[[458,148],[458,100],[424,75],[378,75]]),
      ('barrier_bay','上方挡板区',[[378,75],[153,75]]),
      ('upper_left','左上弯道',[[153,75],[78,75],[43,105],[43,152]]),
      ('s_link','S 弯连接段',[[43,152],[43,202],[78,228],[128,228]]),
      ('junction_approach','路口上行段',[[128,228],[176,228],[198,267],[198,307]]),
      ('junction_down','路口向下直行',[[198,307],[198,462]]),
      ('lower_left_a','左下环岛',[[198,462],[198,508],[164,540],[122,540]]),
      ('lower_left_b','左下环岛',[[122,540],[76,540],[43,508],[43,462]]),
      ('lower_left_c','左下环岛',[[43,462],[43,416],[76,385],[121,385]]),
      ('junction_east','路口向右直行',[[121,385],[235,385]]),
      ('lower_right_a','右下连接弯',[[235,385],[281,385],[306,415],[306,457]]),
      ('lower_right_b','右下弯道',[[306,457],[306,506],[337,540],[381,540]]),
      ('lower_right_c','右下弯道',[[381,540],[426,540],[458,506],[458,462]]),
      ('return_straight','返回起点',[[458,462],[458,420]])]
    points=[];labels=[]
    for sid,label,p in sections:
        estimate=sum(math.dist(a,b) for a,b in zip(p,p[1:]));count=max(2,math.ceil(estimate/4))
        for i in range(count):
            t=i/count
            if len(p)==2:v=[(1-t)*p[0][a]+t*p[1][a] for a in (0,1)]
            else:v=[(1-t)**3*p[0][a]+3*(1-t)**2*t*p[1][a]+3*(1-t)*t*t*p[2][a]+t**3*p[3][a] for a in (0,1)]
            points.append([round(x,3) for x in v]);labels.append(sid)
    return dict(version=1,closed=True,points=points,point_sections=labels,
                sections=[dict(id=s,label=l) for s,l,_ in sections],
                barrier_sequence=['right','left'],scale_estimate=dict(width_m=4.,height_m=4.5,calibrated=False),
                reference='existing_vector_boundaries',route=[1,3,5,6,7,8])


if __name__=='__main__':
    p=Path(__file__).resolve().parents[1]/'data/track.json'
    value=json.loads(p.read_text());value['navigation']=build();value['version']=2
    p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    print('route samples:',len(value['navigation']['points']))
