"""Semantic room lookup, VLM arrival confirmation and tracked target approach."""
import asyncio,json,math,time
from collections import deque
import numpy as np,rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String,Bool
from vision_msgs.msg import Detection3DArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray
from visual_servo import Observation
from world import Grid,candidates
from semantic_world import classify
from waypoint_reasoning import search_region

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
    p=d.bbox.center.position;a=np.array([p.x,p.y,p.z])
    if h.frame_id!='odom':
     tf=node.tf_buffer.lookup_transform('odom',h.frame_id,rclpy.time.Time.from_msg(h.stamp));q=tf.transform.rotation;v=np.array([q.x,q.y,q.z]);a=a+2*np.cross(v,np.cross(v,a)+q.w*a);tr=tf.transform.translation;a+=np.array([tr.x,tr.y,tr.z])
    node.target_tracks.append(Observation(int(result.hypothesis.class_id),time.monotonic(),t,a,d.id))
   except Exception:continue
 node.pub_explore_control=node.create_publisher(Bool,'/explore/resume',10)
 node.pub_servo_vel=node.create_publisher(Twist,'/cmd_vel_nav',10)
 node.servo_camera_info=None
 node.servo_nav_active=False
 def camera_info(msg):node.servo_camera_info=msg
 def nav_status(msg):node.servo_nav_active=any(s.status in (1,2,3) for s in msg.status_list)
 node._semantic_subs=[node.create_subscription(String,'/semantic_world/state',world,qos),node.create_subscription(OccupancyGrid,'/map',grid,qos),node.create_subscription(Detection3DArray,'/yoloe_multi_text_prompt/detections_3d',detections,10),node.create_subscription(Detection3DArray,'/yoloe_multi_text_prompt/tracks_3d',detections,10),node.create_subscription(CameraInfo,'/x_bot/camera_left/camera_info',camera_info,qos_profile_sensor_data),node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',nav_status,qos)]

def base(node,frame='map'):
 t=node.tf_buffer.lookup_transform('odom','base_footprint',rclpy.time.Time());age=node.get_clock().now().nanoseconds/1e9-(t.header.stamp.sec+t.header.stamp.nanosec/1e9)
 if not -.1<=age<=.35:raise ValueError(f'Base odom transform is stale (age={age:.3f}s)')
 p=t.transform.translation;q=t.transform.rotation
 yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 if frame=='odom':return p.x,p.y,yaw
 correction=node.tf_buffer.lookup_transform('map','odom',rclpy.time.Time());age=node.get_clock().now().nanoseconds/1e9-(correction.header.stamp.sec+correction.header.stamp.nanosec/1e9)
 if not -.1<=age<=1.:raise ValueError(f'Map correction is stale (age={age:.3f}s)')
 tr=correction.transform.translation;q=correction.transform.rotation;v=np.array([q.x,q.y,q.z]);a=np.array([p.x,p.y,p.z]);a=a+2*np.cross(v,np.cross(v,a)+q.w*a)
 return a[0]+tr.x,a[1]+tr.y,yaw+math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def room_contains(grid,room,x,y):
 stride=max(1,round(room['cell_resolution']/grid.res));y,x=grid.cell(x,y);y//=stride;x//=stride
 return any(row==y and start<=x<=end for row,start,end in room['cell_runs'])

def object_in_room(grid,room,xy):
 # Objects can occupy obstacle cells, but must be near this room's free space.
 lo,hi=np.asarray(room['bounds_xy']);p=np.asarray(xy)
 if not np.all((p>=lo)&(p<=hi)):return False
 stride=max(1,round(room['cell_resolution']/grid.res));row,col=grid.cell(*p);row/=stride;col/=stride
 return any((y-row)**2+(min(max(col,start),end)-col)**2<=(.75/room['cell_resolution'])**2 for y,start,end in room['cell_runs'])

class SemanticNavigator:
 def __init__(self,robot):self.robot=robot;self.room=None
 def notify(self,phase,message):
  n=self.robot._node;n.semantic_phase=phase;n.get_logger().info(message)
  callback=getattr(n,'navigation_progress',None)
  if callback:callback(message)
 async def wait_map(self):
  n=self.robot._node;started=time.monotonic();last_notice=-100.;resumed=False
  while True:
   now=time.monotonic();elapsed=now-started
   if n._nav_cancel_requested:return False,'任务已取消'
   fresh=bool(n.semantic_world) and now-n.semantic_received<=15
   if not fresh:
    phase='waiting_semantic_world';message='等待结构化语义地图节点发布状态'
    if elapsed>=10:return False,'10秒内未收到有效语义地图状态，未发送导航目标；请检查 Semantic World 节点和 /map'
   elif n.semantic_grid is None:
    phase='waiting_map';message='等待二维 /map，地图服务可能尚未激活'
    if elapsed>=10:return False,'未收到二维 /map，未发送导航目标；请检查 map_server 是否 active'
   else:
    status=n.semantic_world.get('exploration_status')
    if status=='complete':return True,''
    if status in ('failed','save_failed'):return False,'探索或地图保存失败，需要检查地图链路'
    if status not in ('running','saving','paused'):
     return False,'几何地图未就绪且没有正在执行的探索任务，未发送导航目标'
    if status=='paused' and not resumed:n.pub_explore_control.publish(Bool(data=True));resumed=True
    phase='waiting_exploration';message=f'等待自动探索及地图保存完成，当前状态：{status}'
    if elapsed>=1800:return False,'自动探索尚未完成，不能使用未核验的地图导航'
   if now-last_notice>=5:self.notify(phase,message);last_notice=now
   await asyncio.sleep(.2)
 async def navigate(self,name):
  n=self.robot._node;n._nav_cancel_requested=False;self.room=None;n.current_scene='未知';n.semantic_phase='room_lookup'
  try:
   ready,message=await self.wait_map()
   if not ready:return False,message
   if n.semantic_grid is None or time.monotonic()-n.semantic_received>15:return False,'结构化语义地图缺失或更新中断'
   rooms=[r for r in n.semantic_world['rooms'] if r['name']==name]
   guessed=False
   if not rooms:
    if n.semantic_world.get('navigation_hints_status')=='pending':
     self.notify('waiting_room_hint',f'正在让VLM估计{name}的搜寻位置')
     deadline=time.monotonic()+45
     while time.monotonic()<deadline and n.semantic_world.get('navigation_hints_status')=='pending':
      if n._nav_cancel_requested:return False,'任务已取消'
      await asyncio.sleep(.5)
    hints=[h for h in n.semantic_world.get('navigation_hints',[]) if h['name']==name]
    if not hints:return False,f'语义地图尚未确认{name}范围，VLM也未提供可用搜寻点；未发送导航目标'
    hint=max(hints,key=lambda h:h['confidence']);room=search_region(n.semantic_grid,hint)
    if room is None:return False,f'VLM估计的{name}位置没有足够净空，未发送导航目标'
    rooms=[room];guessed=True
    self.notify('room_hint_navigation',f"VLM估计{name}搜寻点 ({room['center'][0]:.2f}, {room['center'][1]:.2f})，到达后搜索目标；依据：{room['reason']}")
   room=min(rooms,key=lambda r:math.dist(base(n)[:2],r['center']));target=getattr(self.robot,'target_class',None)
   objects=[o for o in n.semantic_world['objects'] if o['class_id']==target and (o.get('room_id')==room['id'] or (guessed and object_in_room(n.semantic_grid,room,o['position'][:2])))]
   goals=[]
   if guessed:
    x,y=room['center'];aim=min(objects,key=lambda o:math.dist(o['position'][:2],[x,y]))['position'][:2] if objects else None
    yaw=math.atan2(aim[1]-y,aim[0]-x) if aim else base(n)[2]
    goals.append(dict(x=x,y=y,yaw_deg=math.degrees(yaw),distance=0.))
   elif objects:
    goals=candidates(n.semantic_grid,objects[0]['position'][:2],base(n)[:2],lambda x,y:room_contains(n.semantic_grid,room,x,y))
   if not goals or guessed:
    stride=max(1,round(room['cell_resolution']/n.semantic_grid.res))
    for row,start,end in room['cell_runs']:
     for col in range(start,end+1,3):
      x,y=n.semantic_grid.point(row*stride,col*stride)
      if n.semantic_grid.clear(x,y,radius=.48 if guessed else .30):goals.append(dict(x=x,y=y,yaw_deg=math.degrees(math.atan2(room['center'][1]-y,room['center'][0]-x)),distance=math.dist([x,y],room['center'])))
    goals.sort(key=lambda p:p['distance'])
   if not goals:return False,'房间内没有满足机器人净空要求的导航候选'
   n.semantic_phase='room_navigation'
   # Failed plans may try another free candidate; no direct velocity override.
   for goal in goals[:3]:
    if n._nav_cancel_requested:return False,'任务已取消'
    ok,msg=await self.robot._go_pose(goal,accept_radius=.30) if guessed else await self.robot._go_pose(goal)
    if ok:break
    if n._nav_cancel_requested:return False,msg
   if not ok:return False,msg
   if not room_contains(n.semantic_grid,room,*base(n)[:2]):return False,'实际停车位置不在目标房间区域内'
   if guessed:
    self.room=room;n.semantic_phase='room_search_ready'
    return True,f'已到达VLM估计的{name}搜寻点，接下来开启局部相机建图并搜索目标；{msg}'
   n.semantic_phase='room_confirmation'
   client=getattr(self.robot,'vision_client',None)
   if client is None:return False,'VLM未配置，不能确认到达房间'
   b64=await self.robot._loop.run_in_executor(self.robot._exec,n.get_camera_base64)
   if b64 is None:return False,'没有新鲜图像，不能确认房间'
   from model_config import ModelConfig
   result=await self.robot._loop.run_in_executor(self.robot._exec,lambda:classify(client,ModelConfig.from_env(),b64))
   n.get_logger().info('ARRIVAL_VLM '+json.dumps(result,ensure_ascii=False))
   if result['room']!=name or result['confidence']<.65:return False,'VLM未确认到达'+name+'：'+json.dumps(result,ensure_ascii=False)
   if n._nav_cancel_requested:return False,'任务已取消'
   self.room=room;n.current_scene=name;n.semantic_phase='room_confirmed';return True,'地图导航及VLM房间确认通过：'+json.dumps(result,ensure_ascii=False)
  except Exception as e:return False,'语义导航失败：'+str(e)
  finally:
   if self.room is None:n.semantic_phase='room_navigation_failed'
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
  if self.room is None:return False,'先按语义地图导航并确认目标房间'
  from servo_navigation import run
  return await run(self.robot,self.room,class_id,progress)
