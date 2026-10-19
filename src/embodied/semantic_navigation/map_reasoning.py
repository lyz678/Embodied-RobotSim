"""VLM proposes semantic regions from the global occupancy/object projection."""
import base64,io,json
import numpy as np
from PIL import Image,ImageDraw
from world import ROOMS
NAMES={0:'person',1:'sofa',2:'cup',3:'book',4:'table',5:'chair',6:'desk',7:'bed',8:'TV',9:'coke'}
def infer_regions(client,cfg,grid,objects,observations=()):
 ranked=sorted(objects,key=lambda o:(o.get('source')!='yoloe_tracking',-o['observations']))
 objects=[];counts={}
 for obj in ranked:
  cid=obj['class_id']
  if counts.get(cid,0)>=3:continue
  objects.append(obj);counts[cid]=counts.get(cid,0)+1
  if len(objects)>=25:break
 pixels=np.where(grid.a<0,170,np.where(grid.a>20,35,240)).astype(np.uint8);image=Image.fromarray(pixels[::-1]).convert('RGB');draw=ImageDraw.Draw(image)
 rows=[]
 for obj in objects:
  row,col=grid.cell(*obj['position'][:2]);draw.ellipse((col-3,image.height-row-4,col+3,image.height-row+2),fill='red');draw.text((col+3,image.height-row),obj['id']+' '+NAMES.get(obj['class_id'],'unknown'),fill='blue')
  rows.append(dict(id=obj['id'],class_name=NAMES.get(obj['class_id'],'unknown'),xyz=[round(v,2) for v in obj['position']]))
 image.thumbnail((640,640));buffer=io.BytesIO();image.save(buffer,format='PNG');b64=base64.b64encode(buffer.getvalue()).decode()
 prompt='根据全局地图上已观测到的物体推断区域用途。白色可通行，黑色障碍，灰色未知。坐标来自测量，不可编造物体。返回JSON {"regions":[{"name":"厨房/客厅/书房/卧室/走廊/卫生间","bounds_xy":[[xmin,ymin],[xmax,ymax]],"object_ids":["证据物体id"],"confidence":0到1,"reason":"依据"}]}。bounds_xy必须包围每一个object_ids证据物体的XY坐标，不可遗漏远处证据；不能把所有物体强行归为同一房间。边界应包围支持物体及附近至少0.8m宽的可通行区域；证据不足的区域不要命名。房间可能开放连通，不能仅根据单个可移动杯子或可乐就断言厨房。物体：'+json.dumps(rows,ensure_ascii=False)
 # Observer positions are supplementary evidence, never assumed room coordinates.
 nearby=[{k:o[k] for k in ('room','confidence','position','evidence') if k in o} for o in observations if o.get('confidence',0)>=.65 and any(np.linalg.norm(np.asarray(o['position'][:2])-np.asarray(obj['position'][:2]))<4 for obj in objects)][-12:]
 prompt+='。辅助相机观测（position仅为观察者位置，不能直接当作房间或物体位置）：'+json.dumps(nearby,ensure_ascii=False)
 schema=region_schema([o['id'] for o in objects])
 options=cfg.completion_options();options['max_tokens']=1200
 options['response_format']={'type':'json_schema','json_schema':{'name':'semantic_regions','strict':True,'schema':schema}}
 response=client.chat.completions.create(model=cfg.vision_model,**options,messages=[dict(role='user',content=[dict(type='image_url',image_url=dict(url='data:image/png;base64,'+b64)),dict(type='text',text=prompt)])])
 text=response.choices[0].message.content or '';result=json.loads(text[text.index('{'):text.rindex('}')+1]);regions=result.get('regions',[result] if 'name' in result and 'bounds_xy' in result else [])
 if not isinstance(regions,list):raise ValueError('Invalid VLM region list')
 return regions,buffer.getvalue(),text

def region_schema(object_ids):
 """Constrain names and evidence IDs; free-space validation remains independent."""
 pair={'type':'array','items':{'type':'number'},'minItems':2,'maxItems':2}
 fields={'name':{'type':'string','enum':list(ROOMS)},
         'bounds_xy':{'type':'array','items':pair,'minItems':2,'maxItems':2},
         'object_ids':{'type':'array','items':{'type':'string','enum':object_ids},'minItems':1},
         'confidence':{'type':'number','minimum':0,'maximum':1},'reason':{'type':'string'}}
 return {'type':'object','properties':{'regions':{'type':'array','items':{'type':'object','properties':fields,'required':list(fields),'additionalProperties':False}}},'required':['regions'],'additionalProperties':False}
