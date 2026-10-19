"""Persist structured global semantics from occupancy, YOLOE and VLM observations."""
import sys,os,json,time,math,threading,base64,io
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np,yaml,rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image,PointCloud2
from vision_msgs.msg import Detection3DArray
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'llm_agent'))
from model_config import ModelConfig,create_client
from world import Grid,ROOMS,build_rooms,region_at,regions_from_proposals
from map_reasoning import infer_regions

def classify(client,cfg,b64):
 response=client.chat.completions.create(model=cfg.vision_model,**cfg.completion_options(),messages=[dict(role='user',content=[dict(type='image_url',image_url=dict(url='data:image/jpeg;base64,'+b64)),dict(type='text',text='只根据图中可见证据判断当前房间。返回JSON：{"room":"厨房/客厅/书房/卧室/走廊/卫生间/未知","confidence":0到1,"evidence":"可见家具依据"}。没有足够证据就返回未知；不要根据指令猜测。')])])
 text=response.choices[0].message.content or '';data=json.loads(text[text.index('{'):text.rindex('}')+1]);confidence=float(data['confidence'])
 if data['room'] not in (*ROOMS,'未知') or not math.isfinite(confidence) or not 0<=confidence<=1:raise ValueError('Invalid VLM room response')
 return dict(room=data['room'],confidence=confidence,evidence=str(data.get('evidence','')))

class SemanticWorld(Node):
 def __init__(self):
  super().__init__('semantic_world',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.path=Path(os.environ['EMBODIED_SEMANTIC_WORLD']);self.grid=None;self.image=None;self.observations=[];self.objects=[];self.status='complete' if os.environ.get('EMBODIED_GEOMETRY_READY')=='1' else 'not_started';self.error=None;self.lock=threading.RLock();self.pool=ThreadPoolExecutor(max_workers=1);self.pending=None;self.last_vision=-100.;self.last_map_reasoning=-100.;self.proposals=[];self.cfg=ModelConfig.from_env();self.client=create_client(self.cfg)
  if self.path.exists():
   data=json.loads(self.path.read_text());self.observations=data.get('room_observations',[]);self.objects=data.get('objects',[]);self.status='complete' if os.environ.get('EMBODIED_GEOMETRY_READY')=='1' else data.get('exploration_status','not_started');self.proposals=data.get('region_proposals',[])
   for obj in self.objects:obj['last_time']=-1.
   if self.status!='complete':self.observations=[];self.objects=[];self.status='not_started'
  qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.pub=self.create_publisher(String,'/semantic_world/state',qos)
  self.subs=[self.create_subscription(OccupancyGrid,'/map',self.map,qos),self.create_subscription(Image,'/x_bot/camera_left/image_raw',self.camera,qos_profile_sensor_data),self.create_subscription(Detection3DArray,'/yoloe_multi_text_prompt/detections_3d',self.detect,10),self.create_subscription(PointCloud2,'/local_map/voxels',self.cloud,qos),self.create_subscription(String,'/explore/status',self.explore,qos)]
  self.tf=Buffer();self.listener=TransformListener(self.tf,self);self.timer=self.create_timer(5.,self.tick)
  palette=yaml.safe_load((Path(__file__).resolve().parents[2]/'perception/yoloe_infer/configs/manipulation.yaml').read_text());self.palette={(v['color'][0]<<16)|(v['color'][1]<<8)|v['color'][2]:v['id'] for v in palette['classes']}
 def explore(self,msg):self.status=msg.data
 def map(self,msg):
  q=msg.info.origin.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  if msg.header.frame_id!='map':self.error='Expected map-frame occupancy grid';return
  self.grid=Grid(msg.data,msg.info.width,msg.info.height,msg.info.resolution,[msg.info.origin.position.x,msg.info.origin.position.y],yaw)
 def camera(self,msg):self.image=msg
 def point(self,xyz,frame,stamp):
  tf=self.tf.lookup_transform('map',frame,rclpy.time.Time.from_msg(stamp),timeout=rclpy.duration.Duration(seconds=.1));q=tf.transform.rotation;p=tf.transform.translation
  v=np.array([q.x,q.y,q.z]);a=np.asarray(xyz);a=a+2*np.cross(v,np.cross(v,a)+q.w*a)
  return (a+np.array([p.x,p.y,p.z])).tolist()
 def observe_object(self,cid,position,stamp,confidence,source):
  if not np.isfinite(position).all() or not .2<=position[2]<=2.5:return
  match=next((v for v in self.objects if v['class_id']==cid and np.linalg.norm(np.array(v['position'])-position)<.30),None)
  if match is None:
   match=dict(id=f'{cid}_{len(self.objects)}',class_id=cid,position=position,observations=0,last_time=-1.);self.objects.append(match)
  if stamp<=match['last_time']:return
  match.update(position=(np.array(match['position'])*.6+np.array(position)*.4).tolist(),observations=match['observations']+1,last_time=stamp,confidence=confidence,source=source)
 def detect(self,msg):
  try:
   for d in msg.detections:
    if not d.results:continue
    h=d.results[0];header=d.header if d.header.frame_id else msg.header;age=self.get_clock().now().nanoseconds/1e9-(header.stamp.sec+header.stamp.nanosec/1e9)
    if not -.1<=age<=.75 or h.hypothesis.score<.25:continue
    p=d.bbox.center.position;position=self.point([p.x,p.y,p.z],header.frame_id,header.stamp)
    self.observe_object(int(h.hypothesis.class_id),position,header.stamp.sec+header.stamp.nanosec/1e9,h.hypothesis.score,'yoloe_tracking')
  except Exception as e:self.error='Object detection: '+str(e)
 def cloud(self,msg):
  try:
   fields={f.name:f for f in msg.fields};required=('x','y','z','rgb','semantic_confidence')
   if not all(k in fields for k in required):return
   endian='>' if msg.is_bigendian else '<';dtype=np.dtype(dict(names=required,formats=[endian+'f4']*3+[endian+'u4',endian+'f4'],offsets=[fields[k].offset for k in required],itemsize=msg.point_step));a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
   for color,cid in self.palette.items():
    selected=a[(a['rgb']==color)&(a['semantic_confidence']>=.6)]
    if len(selected)<3:continue
    xyz=np.stack([selected[k] for k in ('x','y','z')],1);buckets=np.floor(xyz[:,:2]/.35).astype(int)
    for key in set(map(tuple,buckets)):
     points=xyz[np.all(buckets==key,axis=1)]
     if len(points)<3:continue
     position=self.point(np.median(points,axis=0),msg.header.frame_id,msg.header.stamp)
     self.observe_object(cid,position,msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,.8,'semantic_voxels')
  except Exception as e:self.error='Semantic cloud: '+str(e)
 def vision(self,msg,position):
  try:
   channels=4 if msg.encoding in ('rgba8','bgra8') else 3;a=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*channels].reshape(msg.height,msg.width,channels)[:,:,:3]
   if msg.encoding.startswith('bgr'):a=a[:,:,::-1]
   image=PILImage.fromarray(a);image.thumbnail((640,640));b=io.BytesIO();image.save(b,format='JPEG');result=classify(self.client,self.cfg,base64.b64encode(b.getvalue()).decode())
   result.update(position=position,time=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
   with self.lock:
    if not any(v['room']==result['room'] and np.linalg.norm(np.array(v['position'])-position)<.2 for v in self.observations):self.observations.append(result)
   self.get_logger().info('VLM room observation '+json.dumps(result,ensure_ascii=False))
  except Exception as e:self.error='VLM: '+str(e)
 def reason_map(self,grid,objects):
  try:
   proposals,png,raw=infer_regions(self.client,self.cfg,grid,objects,list(self.observations))
   self.path.with_suffix('.vlm.json').write_text(json.dumps(dict(time=time.time(),raw=raw,objects=objects),ensure_ascii=False,indent=2))
   with self.lock:self.proposals=proposals;self.error=''
   self.path.with_suffix('.png').write_bytes(png)
   self.get_logger().info('VLM map regions '+json.dumps(proposals,ensure_ascii=False))
  except Exception as e:self.error='Map VLM: '+str(e)
 def tick(self):
  if self.grid is None:return
  eligible=[o.copy() for o in self.objects if o['observations']>=3]
  if len(eligible)>=3 and time.monotonic()-self.last_map_reasoning>=30 and (self.pending is None or self.pending.done()):
   self.last_map_reasoning=time.monotonic();self.pending=self.pool.submit(self.reason_map,self.grid,eligible)
  if self.image and time.monotonic()-self.last_vision>=12 and (self.pending is None or self.pending.done()):
   msg=self.image;age=self.get_clock().now().nanoseconds/1e9-(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
   if 0<=age<=.5:
    try:
     position=self.point([0,0,0],'base_footprint',msg.header.stamp);self.last_vision=time.monotonic();self.pending=self.pool.submit(self.vision,msg,position)
    except Exception as e:self.error='Image pose: '+str(e)
  with self.lock:rooms,labels,stride=regions_from_proposals(self.grid,self.proposals,eligible)
  for obj in self.objects:
   region=region_at(self.grid,labels,stride,obj['position'][:2],near=True);obj['room_id']=region;obj['room']=next((r['name'] for r in rooms if r['id']==region),'未知')
  data=dict(frame='map',schema=2,region_proposals=self.proposals,exploration_status=self.status,rooms=rooms,objects=[v for v in self.objects if v['observations']>=3],room_observations=self.observations,grid=dict(resolution=self.grid.res,origin=self.grid.origin,yaw=self.grid.yaw,width=self.grid.a.shape[1],height=self.grid.a.shape[0]),last_error=self.error)
  self.path.parent.mkdir(parents=True,exist_ok=True);temp=self.path.with_suffix('.tmp');temp.write_text(json.dumps(data,ensure_ascii=False,indent=2));temp.replace(self.path);self.pub.publish(String(data=json.dumps(data,ensure_ascii=False)))
def main():
 rclpy.init();node=SemanticWorld()
 executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
 try:executor.spin()
 finally:executor.shutdown();node.pool.shutdown(wait=False,cancel_futures=True);node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
