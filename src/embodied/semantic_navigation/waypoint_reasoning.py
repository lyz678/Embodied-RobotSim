"""VLM guesses search locations by selecting measured free-space waypoints."""
import base64
import io
import json
import math
from collections import deque
import numpy as np
from PIL import Image,ImageDraw


def search_waypoints(grid,objects,observations,limit=20):
    stride=max(1,round(.8/grid.res));pool=[]
    for row in range(stride,grid.a.shape[0]-stride,stride):
        for col in range(stride,grid.a.shape[1]-stride,stride):
            point=grid.point(row,col)
            if grid.clear(*point,radius=.48):pool.append(point)
    if not pool:return []
    chosen=[]
    def add(point):
        if all(math.dist(point,p)>=.7 for p in chosen):chosen.append(point)
    anchors=[o['position'][:2] for o in observations if o.get('confidence',0)>=.65]
    anchors += [o['position'][:2] for o in sorted(objects,key=lambda o:(o['class_id'] not in (9,3),-o.get('observations',0)))]
    for anchor in anchors:
        point=min(pool,key=lambda p:math.dist(p,anchor))
        if math.dist(point,anchor)<=2.:add(point)
        if len(chosen)>=limit//2:break
    if not chosen:add(pool[0])
    while len(chosen)<min(limit,len(pool)):
        point=max(pool,key=lambda p:min(math.dist(p,c) for c in chosen))
        if min(math.dist(point,c) for c in chosen)<.7:break
        add(point)
    rows=[]
    for index,point in enumerate(chosen):
        nearby=sorted(objects,key=lambda o:math.dist(point,o['position'][:2]))
        features=[]
        for obj in nearby:
            name=obj.get('class_name',str(obj['class_id']));distance=math.dist(point,obj['position'][:2])
            if distance>3. or name in [v[0] for v in features]:continue
            features.append([name,round(distance,1)])
            if len(features)>=3:break
        rows.append(dict(id=f'P{index}',xy=[round(v,3) for v in point],near=features))
    return rows


def infer_navigation_hints(client,cfg,grid,objects,observations,names=('厨房','书房')):
    waypoints=search_waypoints(grid,objects,observations)
    if not waypoints:raise ValueError('No collision-free search waypoint')
    pixels=np.where(grid.a<0,170,np.where(grid.a>20,35,240)).astype(np.uint8)
    image=Image.fromarray(pixels[::-1]).convert('RGB');draw=ImageDraw.Draw(image)
    for point in waypoints:
        row,col=grid.cell(*point['xy']);y=image.height-1-row
        draw.ellipse((col-2,y-2,col+2,y+2),fill='red');draw.text((col+2,y),point['id'],fill='blue')
    image.thumbnail((640,640));buffer=io.BytesIO();image.save(buffer,format='PNG')
    views=[dict(room=o['room'],confidence=o['confidence'],xy=[round(v,2) for v in o['position'][:2]],evidence=o.get('evidence','')[:60]) for o in observations if o.get('confidence',0)>=.65][-8:]
    prompt=('请根据全局地图、附近检测物体和相机观察，为'+ '、'.join(names)+
        '各猜测一个最可能的位置，作为机器人到达后搜寻物体的起点。允许不确定，confidence反映猜测把握。'
        '只从候选P编号中选择，不要输出房间边界或编造坐标。候选均已检查机器人净空。'
        'near是候选附近的检测类别与距离(米)，检测类别可能有误；相机明确看到水槽/橱柜优先支持厨房，书桌/书籍支持书房。'
        '相机xy是观察者位置，可作为回访观察点，不等于房间中心。厨房和书房尽量选择不同位置。'
        '返回JSON {"hints":[{"name":"房间名","waypoint_id":"P编号","confidence":0到1,"reason":"猜测依据"}]}。'
        '候选：'+json.dumps(waypoints,ensure_ascii=False)+'。相机观察：'+json.dumps(views,ensure_ascii=False))
    fields=dict(name={'type':'string','enum':list(names)},waypoint_id={'type':'string','enum':[p['id'] for p in waypoints]},confidence={'type':'number','minimum':0,'maximum':1},reason={'type':'string'})
    schema={'type':'object','properties':{'hints':{'type':'array','items':{'type':'object','properties':fields,'required':list(fields),'additionalProperties':False},'maxItems':len(names)}},'required':['hints'],'additionalProperties':False}
    options=cfg.completion_options();options['max_tokens']=600
    options['response_format']={'type':'json_schema','json_schema':{'name':'room_search_hints','strict':True,'schema':schema}}
    response=client.chat.completions.create(model=cfg.vision_model,**options,messages=[dict(role='user',content=[dict(type='image_url',image_url=dict(url='data:image/png;base64,'+base64.b64encode(buffer.getvalue()).decode())),dict(type='text',text=prompt)])])
    raw=response.choices[0].message.content or '';data=json.loads(raw[raw.index('{'):raw.rindex('}')+1]);known={p['id']:p for p in waypoints};hints=[]
    for row in data.get('hints',[]):
        if row.get('name') not in names or row.get('waypoint_id') not in known:continue
        confidence=float(row['confidence']);point=known[row['waypoint_id']]
        if not math.isfinite(confidence) or not 0<confidence<=1 or not grid.clear(*point['xy'],radius=.48):continue
        if any(h['name']==row['name'] for h in hints):continue
        hints.append(dict(name=row['name'],position=point['xy'],confidence=confidence,reason=row['reason'],waypoint_id=point['id'],source='vlm_search_guess',search_radius=4.))
    if not hints:raise ValueError('VLM did not choose a valid search waypoint')
    return hints,buffer.getvalue(),raw


def search_region(grid,hint):
    """A bounded reachable search area, without claiming a room boundary."""
    center=np.asarray(hint['position'],float);radius=float(hint.get('search_radius',3.))
    if center.shape!=(2,) or not np.isfinite(center).all() or not grid.clear(*center,radius=.48):return None
    radius=min(4.,max(1.,radius));stride=max(1,int(math.ceil(.15/grid.res)))
    row,col=grid.cell(*center);seed=(row//stride,col//stride);pending=deque([seed]);seen={seed};selected=[]
    while pending:
        row,col=pending.popleft();y=row*stride;x=col*stride
        if y<0 or x<0 or y+stride>grid.a.shape[0] or x+stride>grid.a.shape[1]:continue
        block=grid.a[y:y+stride,x:x+stride]
        if not np.all((block>=0)&(block<=20)) or math.dist(grid.point(y,x),center)>radius:continue
        selected.append((row,col))
        for p in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
            if p not in seen:seen.add(p);pending.append(p)
    if not selected:return None
    runs=[]
    for row in sorted({p[0] for p in selected}):
        xs=sorted(x for y,x in selected if y==row);start=last=xs[0]
        for x in xs[1:]:
            if x!=last+1:runs.append([row,start,last]);start=x
            last=x
        runs.append([row,start,last])
    return dict(id='hint_'+hint['name'],name=hint['name'],center=center.tolist(),bounds_xy=[(center-radius).tolist(),(center+radius).tolist()],cell_runs=runs,cell_resolution=grid.res*stride,confidence=hint['confidence'],reason=hint.get('reason',''),boundary_source='vlm_navigation_hint')
