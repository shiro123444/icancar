"""Postprocessing for the vehicle's two-class YOLO-style race model."""
import numpy as np


def predictions(output,width,height,threshold=.30,iou_limit=.40):
    if width<=0 or height<=0 or not 0<threshold<1 or not 0<iou_limit<1:
        raise ValueError('invalid inference parameters')
    values=np.asarray(output)
    if values.ndim==3 and values.shape[0]==1:values=values[0]
    if values.ndim!=2 or values.shape[1]!=7:raise ValueError('expected [N,7] race model output')
    scores=values[:,4:5]*values[:,5:7]
    candidates=[]
    for row,prob in zip(values,scores):
        if not np.isfinite(row).all():continue
        cls=int(np.argmax(prob))
        confidence=float(prob[cls])
        if confidence<threshold:continue
        cx,cy,bw,bh=(float(v) for v in row[:4])
        if bw<=0 or bh<=0:continue
        x1=max(0,min(width,(cx-bw/2)*width/640))
        x2=max(0,min(width,(cx+bw/2)*width/640))
        y1=max(0,min(height,(cy-bh/2)*height/640))
        y2=max(0,min(height,(cy+bh/2)*height/640))
        if x2<=x1 or y2<=y1:continue
        candidates.append({'label':('crossing','start')[cls],'score':confidence,'box':[x1,y1,x2,y2]})
    candidates.sort(key=lambda item:item['score'],reverse=True)
    selected=[]
    for item in candidates:
        x1,y1,x2,y2=item['box']
        area=(x2-x1)*(y2-y1)
        suppressed=False
        for other in selected:
            if other['label']!=item['label']:continue
            a,b,c,d=other['box']
            intersection=max(0,min(x2,c)-max(x1,a))*max(0,min(y2,d)-max(y1,b))
            union=area+(c-a)*(d-b)-intersection
            if union>0 and intersection/union>iou_limit:
                suppressed=True
                break
        if not suppressed:
            selected.append(item)
            if len(selected)>=100:break
    return [{'label':x['label'],'score':round(x['score'],3),
             'box':[round(v,1) for v in x['box']]} for x in selected]
