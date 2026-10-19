"""Map-derived room regions and object goals. No room coordinates or simulator truth."""
import math
from collections import Counter,deque
import numpy as np
ROOMS=('厨房','客厅','书房','卧室','走廊','卫生间')
class Grid:
 def __init__(self,data,width,height,resolution,origin,yaw=0.):
  self.a=np.asarray(data,dtype=np.int16).reshape(height,width);self.res=float(resolution);self.origin=origin;self.yaw=yaw
 def cell(self,x,y):
  x-=self.origin[0];y-=self.origin[1];c=math.cos(self.yaw);s=math.sin(self.yaw)
  return int(math.floor((-s*x+c*y)/self.res)),int(math.floor((c*x+s*y)/self.res))
 def point(self,row,col):
  x=(col+.5)*self.res;y=(row+.5)*self.res;c=math.cos(self.yaw);s=math.sin(self.yaw)
  return self.origin[0]+c*x-s*y,self.origin[1]+s*x+c*y
 def clear(self,x,y,radius=.30):
  row,col=self.cell(x,y);n=int(math.ceil(radius/self.res));h,w=self.a.shape
  if row<n or col<n or row+n>=h or col+n>=w:return False
  yy,xx=np.ogrid[-n:n+1,-n:n+1];mask=xx*xx+yy*yy<=(radius/self.res)**2
  return bool(np.all((self.a[row-n:row+n+1,col-n:col+n+1][mask]>=0)&(self.a[row-n:row+n+1,col-n:col+n+1][mask]<=20)))
 def regions(self,clearance=.45):
  stride=max(1,int(math.ceil(.15/self.res)));h,w=self.a.shape;h=h//stride*stride;w=w//stride*stride
  free=((self.a[:h,:w]>=0)&(self.a[:h,:w]<=20)).reshape(h//stride,stride,w//stride,stride).all(axis=(1,3));eroded=free.copy();n=int(math.ceil(clearance/(self.res*stride)))
  for dy in range(-n,n+1):
   for dx in range(-n,n+1):
    if dx*dx+dy*dy>n*n:continue
    shifted=np.zeros_like(free);y0=max(0,dy);y1=min(len(free),len(free)+dy);x0=max(0,dx);x1=min(free.shape[1],free.shape[1]+dx)
    shifted[y0:y1,x0:x1]=free[y0-dy:y1-dy,x0-dx:x1-dx];eroded&=shifted
  labels=np.zeros(free.shape,dtype=np.int32);visited=set();label=0;queue=deque()
  for y,x in zip(*np.nonzero(eroded)):
   if (y,x) in visited:continue
   component=[];pending=deque([(y,x)]);visited.add((y,x))
   while pending:
    a,b=pending.popleft();component.append((a,b))
    for c,d in ((a-1,b),(a+1,b),(a,b-1),(a,b+1)):
     if 0<=c<len(free) and 0<=d<free.shape[1] and eroded[c,d] and (c,d) not in visited:visited.add((c,d));pending.append((c,d))
   if len(component)*(self.res*stride)**2<.75:continue
   label+=1
   for y,x in component:labels[y,x]=label;queue.append((y,x))
  while queue:
   y,x=queue.popleft()
   for a,b in ((y-1,x),(y+1,x),(y,x-1),(y,x+1)):
    if 0<=a<len(free) and 0<=b<free.shape[1] and free[a,b] and not labels[a,b]:labels[a,b]=labels[y,x];queue.append((a,b))
  return labels,stride

def region_at(grid,labels,stride,point,near=False):
 y,x=grid.cell(*point);y//=stride;x//=stride
 if 0<=y<len(labels) and 0<=x<labels.shape[1] and labels[y,x]:return int(labels[y,x])
 if near:
  yy,xx=np.nonzero(labels);distance=(yy-y)**2+(xx-x)**2
  if len(distance) and distance.min()*(grid.res*stride)**2<=1.:return int(labels[yy[distance.argmin()],xx[distance.argmin()]])
 return None

def build_rooms(grid,observations):
 # Select a geometric scale by agreement with independent visual observations.
 # This avoids requiring manually chosen doorway widths for every house.
 def consistency(result):
  labels,stride=result;counts={}
  for row in observations:
   region=region_at(grid,labels,stride,row['position'][:2])
   if region and row['room'] in ROOMS and row['confidence']>=.65:counts.setdefault(region,Counter())[row['room']]+=1
  return sum((max(v.values()) if max(v.values())>=2 else 0)-(sum(v.values())-max(v.values())) for v in counts.values())
 partitions=[grid.regions(radius) for radius in (.45,.65,.85,1.05)]
 labels,stride=max(partitions,key=consistency);votes={}
 for row in observations:
  region=region_at(grid,labels,stride,row['position'][:2])
  if region and row['room'] in ROOMS and row['confidence']>=.65:votes.setdefault(region,Counter())[row['room']]+=1
 rooms=[]
 for label in sorted(set(labels.flat)-{0}):
  yy,xx=np.where(labels==label);points=np.array([grid.point(int(y)*stride,int(x)*stride) for y,x in zip(yy,xx)]);count=votes.get(label,Counter());name,n=count.most_common(1)[0] if count else ('未知',0)
  if n<2 or n<sum(count.values())*.6:name='未知'
  runs=[]
  for y in sorted(set(yy)):
   xs=sorted(xx[yy==y]);start=last=int(xs[0])
   for x in xs[1:]:
    if x!=last+1:runs.append([int(y),start,last]);start=int(x)
    last=int(x)
   runs.append([int(y),start,last])
  rooms.append(dict(id=int(label),name=name,bounds_xy=[points.min(0).tolist(),points.max(0).tolist()],center=points.mean(0).tolist(),cell_runs=runs,cell_resolution=grid.res*stride,votes=dict(count),area_m2=len(points)*(grid.res*stride)**2))
 return rooms,labels,stride

def candidates(grid,object_xy,base_xy,room_filter=None):
 """Collision-free camera-facing base poses on a grasp-distance annulus."""
 result=[];tx,ty=object_xy
 for radius in (.68,.73,.78):
  for angle in np.linspace(-math.pi,math.pi,48,endpoint=False):
   x=tx+radius*math.cos(angle);y=ty+radius*math.sin(angle)
   if grid.clear(x,y) and (room_filter is None or room_filter(x,y)):
    yaw=math.atan2(ty-y,tx-x);result.append(dict(x=x,y=y,yaw_deg=math.degrees(yaw),distance=math.hypot(x-base_xy[0],y-base_xy[1])))
 return sorted(result,key=lambda p:p['distance'])

def regions_from_proposals(grid,proposals,objects):
 """Validate VLM region proposals against observed object IDs and free grid cells."""
 stride=max(1,int(math.ceil(.15/grid.res)));h,w=grid.a.shape;h=h//stride*stride;w=w//stride*stride
 free=((grid.a[:h,:w]>=0)&(grid.a[:h,:w]<=20)).reshape(h//stride,stride,w//stride,stride).all(axis=(1,3));labels=np.zeros(free.shape,np.int32);rooms=[];known={o['id']:o for o in objects}
 yy,xx=np.nonzero(free);points=np.array([grid.point(int(y)*stride,int(x)*stride) for y,x in zip(yy,xx)])
 if not len(points):return rooms,labels,stride
 for proposal in proposals:
  try:
   name=proposal['name'];confidence=float(proposal['confidence']);ids=proposal['object_ids'];lo,hi=np.asarray(proposal['bounds_xy'],float)
   if name not in ROOMS or not .65<=confidence<=1 or not ids or any(i not in known for i in ids):continue
   if not np.isfinite([lo,hi]).all() or np.any(hi<=lo):continue
   evidence=np.array([known[i]['position'][:2] for i in ids])
   if not np.all((evidence>=lo)&(evidence<=hi)):continue
   mask=np.all((points>=lo)&(points<=hi),axis=1);selected=np.flatnonzero(mask)
   if len(selected)*(grid.res*stride)**2<.75:continue
   label=len(rooms)+1;labels[yy[selected],xx[selected]]=label;runs=[]
   for row in sorted(set(yy[selected])):
    xs=sorted(xx[selected][yy[selected]==row]);start=last=int(xs[0])
    for x in xs[1:]:
     if x!=last+1:runs.append([int(row),start,last]);start=int(x)
     last=int(x)
    runs.append([int(row),start,last])
   rooms.append(dict(id=label,name=name,bounds_xy=[lo.tolist(),hi.tolist()],center=points[selected].mean(0).tolist(),cell_runs=runs,cell_resolution=grid.res*stride,area_m2=len(selected)*(grid.res*stride)**2,confidence=confidence,object_ids=ids,reason=proposal.get('reason',''),boundary_source='vlm_objects_and_occupancy'))
  except (ValueError,KeyError,TypeError):continue
 return rooms,labels,stride
