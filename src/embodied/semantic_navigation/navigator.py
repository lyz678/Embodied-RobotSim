"""Semantic room lookup, VLM arrival confirmation and tracked target approach."""
import asyncio,json,math,time
from collections import deque
import numpy as np,rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String,Bool
from vision_msgs.msg import Detection3DArray
from world import Grid,candidates
from semantic_world import classify

def install(node):
 node.semantic_world={};node.semantic_grid=None;node.target_tracks=deque(maxlen=200);node.semantic_received=0.
 qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 def world(msg):node.semantic_world=json.loads(msg.data);node.semantic_received=time.monotonic()
 def grid(msg):
  if msg.header.frame_id!='map':return
  q=msg.info.origin.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  node.semantic_grid=Grid(msg.data,msg.info.width,msg.info.height,msg.info.resolution,[msg.info.origin.position.x,msg.info.origin.position.y],yaw)
 def detections(msg):
  for d in msg.detections:
   if not d.results:continue
   result=d.results[0];h=d.header if d.header.frame_id else msg.header;t=h.stamp.sec+h.stamp.nanosec/1e9;now=node.get_clock().now().nanoseconds/1e9
   if not -.1<=now-t<=.75 or result.hypothesis.score<.25:continue
   try:
    p=d.bbox.center.position;tf=node.tf_buffer.lookup_transform('map',h.frame_id,rclpy.time.Time.from_msg(h.stamp));q=tf.transform.rotation;v=np.array([q.x,q.y,q.z]);a=np.array([p.x,p.y,p.z]);a=a+2*np.cross(v,np.cross(v,a)+q.w*a);tr=tf.transform.translation;a+=np.array([tr.x,tr.y,tr.z]);node.target_tracks.append((int(result.hypothesis.class_id),time.monotonic(),t,a))
   except Exception:continue
 node.pub_explore_control=node.create_publisher(Bool,'/explore/resume',10)
 node._semantic_subs=[node.create_subscription(String,'/semantic_world/state',world,qos),node.create_subscription(OccupancyGrid,'/map',grid,qos),node.create_subscription(Detection3DArray,'/yoloe_multi_text_prompt/detections_3d',detections,10)]

def base(node):
 t=node.tf_buffer.lookup_transform('map','base_footprint',rclpy.time.Time());age=node.get_clock().now().nanoseconds/1e9-(t.header.stamp.sec+t.header.stamp.nanosec/1e9)
 if not -.1<=age<=.5:raise ValueError('Base map transform is stale')
 p=t.transform.translation;q=t.transform.rotation
 return p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def room_contains(grid,room,x,y):
 stride=max(1,round(room['cell_resolution']/grid.res));y,x=grid.cell(x,y);y//=stride;x//=stride
 return any(row==y and start<=x<=end for row,start,end in room['cell_runs'])

class SemanticNavigator:
 def __init__(self,robot):self.robot=robot;self.room=None
 async def navigate(self,name):
  n=self.robot._node;n._nav_cancel_requested=False
  if n.semantic_world.get('exploration_status')=='paused':n.pub_explore_control.publish(Bool(data=True))
  deadline=time.monotonic()+1800
  while n.semantic_world.get('exploration_status')!='complete':
   if n._nav_cancel_requested:return False,'任务已取消'
   if n.semantic_world.get('exploration_status') in ('failed','save_failed'):return False,'探索或地图保存失败，需要检查地图链路'
   if time.monotonic()>deadline:return False,'自动探索尚未完成，不能使用未核验的地图导航'
   await asyncio.sleep(2)
  try:
   if n.semantic_grid is None or time.monotonic()-n.semantic_received>15:return False,'结构化语义地图缺失或更新中断'
   rooms=[r for r in n.semantic_world['rooms'] if r['name']==name]
   if not rooms:
    await self.survey(name)
    rooms=[r for r in n.semantic_world['rooms'] if r['name']==name]
   if not rooms:return False,f'地图视觉巡查后仍未确认{name}范围，不使用固定坐标兜底'
   self.room=min(rooms,key=lambda r:math.dist(base(n)[:2],r['center']));target=getattr(self.robot,'target_class',None)
   objects=[o for o in n.semantic_world['objects'] if o['class_id']==target and o['room_id']==self.room['id']]
   goals=[]
   if objects:
    goals=candidates(n.semantic_grid,objects[0]['position'][:2],base(n)[:2],lambda x,y:room_contains(n.semantic_grid,self.room,x,y))
   if not goals:
    stride=max(1,round(self.room['cell_resolution']/n.semantic_grid.res))
    for row,start,end in self.room['cell_runs']:
     for col in range(start,end+1,3):
      x,y=n.semantic_grid.point(row*stride,col*stride)
      if n.semantic_grid.clear(x,y):goals.append(dict(x=x,y=y,yaw_deg=math.degrees(math.atan2(self.room['center'][1]-y,self.room['center'][0]-x)),distance=math.dist([x,y],self.room['center'])))
    goals.sort(key=lambda p:p['distance'])
   if not goals:return False,'房间内没有满足机器人净空要求的导航候选'
   # Failed plans may try another free candidate; no direct velocity override.
   for goal in goals[:3]:
    if n._nav_cancel_requested:return False,'任务已取消'
    ok,msg=await self.robot._go_pose(goal)
    if ok:break
   if not ok:return False,msg
   client=getattr(self.robot,'vision_client',None)
   if client is None:return False,'VLM未配置，不能确认到达房间'
   b64=await self.robot._loop.run_in_executor(self.robot._exec,n.get_camera_base64)
   if b64 is None:return False,'没有新鲜图像，不能确认房间'
   from model_config import ModelConfig
   result=await self.robot._loop.run_in_executor(self.robot._exec,lambda:classify(client,ModelConfig.from_env(),b64))
   n.get_logger().info('ARRIVAL_VLM '+json.dumps(result,ensure_ascii=False))
   if result['room']!=name or result['confidence']<.65:return False,'VLM未确认到达'+name+'：'+json.dumps(result,ensure_ascii=False)
   n.current_scene=name;return True,'地图导航及VLM房间确认通过：'+json.dumps(result,ensure_ascii=False)
  except Exception as e:return False,'语义导航失败：'+str(e)
 async def survey(self,requested):
  n=self.robot._node;grid=n.semantic_grid;n.semantic_phase='semantic_survey';attempted=[]
  for step in range(8):
   if n._nav_cancel_requested:return
   if any(r['name']==requested for r in n.semantic_world.get('rooms',[])):return
   observed=[v['position'][:2] for v in n.semantic_world.get('room_observations',[])];x,y,_=base(n);views=[];stride=max(1,round(1./grid.res))
   for row in range(stride,grid.a.shape[0]-stride,stride):
    for col in range(stride,grid.a.shape[1]-stride,stride):
     px,py=grid.point(row,col)
     if not grid.clear(px,py) or any(math.dist([px,py],p)<.6 for p in attempted):continue
     gain=min((math.dist([px,py],p) for p in observed),default=5.)
     if gain<1.:continue
     views.append((gain/(1+.25*math.dist([px,py],[x,y])),px,py))
   if not views:return
   _,px,py=max(views);attempted.append([px,py]);n.get_logger().info(f'SEMANTIC_SURVEY step={step} map_candidate=({px:.3f},{py:.3f})')
   ok,_=await self.robot._go_pose(dict(x=px,y=py,yaw_deg=math.degrees(math.atan2(py-y,px-x))))
   if not ok:continue
   for angle in (0.,90.,180.,270.):
    if n._nav_cancel_requested:return
    ok,_=await self.robot._go_pose(dict(x=px,y=py,yaw_deg=angle))
    if not ok:break
    await asyncio.sleep(3)
    if any(r['name']==requested for r in n.semantic_world.get('rooms',[])):return
 async def approach(self,class_id,progress=None):
  n=self.robot._node
  if self.room is None:return False,'先按语义地图导航并确认目标房间'
  def notify(msg):
   n.get_logger().info(msg)
   if progress:progress(msg)
  for correction in range(8):
   if n._nav_cancel_requested:return False,'任务已取消'
   end=time.monotonic()+8;target=None
   while time.monotonic()<end:
    rows=[r for r in n.target_tracks if r[0]==class_id and time.monotonic()-r[1]<.75]
    unique={r[2]:r[3] for r in rows}
    if len(unique)>=3:
     xyz=np.stack(list(unique.values()))
     if np.max(np.linalg.norm(xyz-np.median(xyz,axis=0),axis=1))<.05:target=np.median(xyz,axis=0);break
    await asyncio.sleep(.1)
   if target is None:
    memory=[o for o in n.semantic_world['objects'] if o['class_id']==class_id and o['room_id']==self.room['id']]
    if not memory:
     if correction>=4:return False,'当前房间中未找到新鲜目标，观察搜索结束'
     x,y,yaw=base(n);notify('地图目标未知，在已到达区域用Nav2转向观察')
     ok,msg=await self.robot._go_pose(dict(x=x,y=y,yaw_deg=math.degrees(yaw+math.pi/2)))
     if not ok:return False,msg
     continue
    target=np.asarray(memory[0]['position']);fresh=False
   else:fresh=True
   x,y,yaw=base(n);distance=math.dist([x,y],target[:2]);angle=math.atan2(target[1]-y,target[0]-x);error=abs(math.atan2(math.sin(angle-yaw),math.cos(angle-yaw)))
   notify(f'目标跟踪 class={class_id} fresh={fresh} distance={distance:.3f}m heading_error={math.degrees(error):.1f}deg correction={correction}')
   if fresh and .62<=distance<=.76 and error<=.10:
    stopped=await self.robot._loop.run_in_executor(self.robot._exec,n.base_stopped)
    if stopped:return True,'稳定实时目标已进入候选抓取范围，最终可达性由GraspNet／MoveIt核验'
   goals=candidates(n.semantic_grid,target[:2],[x,y],lambda x,y:room_contains(n.semantic_grid,self.room,x,y))
   if not goals:return False,'目标附近没有无障碍的可抓取距离候选位姿'
   ok,msg=await self.robot._go_pose(goals[min(correction,len(goals)-1)])
   if not ok:return False,'目标接近失败：'+msg
  return False,'目标跟踪接近未收敛；禁止仅凭地图旧位置抓取'
