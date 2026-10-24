"""Visual feedback to Nav2's velocity smoother, collision monitor and safety gate."""
import asyncio
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from visual_servo import TargetLock,ServoConfig,command


def point_in_frame(node,point,frame,stamp=None):
    tf=node.tf_buffer.lookup_transform(frame,'odom',stamp or rclpy.time.Time())
    if stamp is None:
        age=node.get_clock().now().nanoseconds/1e9-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)
        if not -.1<=age<=(1. if frame=='map' else .35):raise ValueError('Visual servo transform is stale')
    t=tf.transform.translation;q=tf.transform.rotation;v=np.array([q.x,q.y,q.z]);p=np.asarray(point)
    return p+2*np.cross(v,np.cross(v,p)+q.w*p)+np.array([t.x,t.y,t.z])


def image_projection(node,point):
    info=getattr(node,'servo_camera_info',None);image=getattr(node,'_camera_image',None)
    if info is None or image is None or info.k[0]<=0 or info.k[4]<=0 or not info.width or not info.height:return None
    now=node.get_clock().now().nanoseconds/1e9;stamp=image.header.stamp
    if not 0<=now-(stamp.sec+stamp.nanosec/1e9)<=.5:return None
    if info.header.frame_id!=image.header.frame_id:return None
    # This is a prediction of the target in the current camera, not the
    # conversion of an old image measurement. Keep measurement TF exact in
    # perception; use only a fresh latest camera pose for servo projection.
    p=point_in_frame(node,point,image.header.frame_id)
    if p[2]<=.05:return None
    return ((info.k[0]*p[0]/p[2]+info.k[2])/info.width,
            (info.k[4]*p[1]/p[2]+info.k[5])/info.height)


async def run(robot,room,class_id,progress=None,config=None,clock=None,sleep=None):
    from navigator import base,room_contains,object_in_room
    clock=clock or time.monotonic;sleep=sleep or asyncio.sleep
    n=robot._node;c=config or ServoConfig();lock=TargetLock(c)
    started=clock();last_notice=-100.;ready_since=None;search_start=None;last_pose=None;last_motion=clock()
    def send(linear=0.,angular=0.):
        msg=Twist();msg.linear.x=float(linear);msg.angular.z=float(angular)
        # /cmd_vel_nav is the RAW Nav2 input to velocity_smoother. The separate
        # /x_bot/cmd_vel_nav is collision-monitor output and must never be used here.
        n.pub_servo_vel.publish(msg)
    def notify(text):
        n.get_logger().info(text)
        if progress:progress(text)
    if getattr(n,'_nav_handle',None) is not None or getattr(n,'servo_nav_active',False):return False,'Nav2仍有活动目标，不能同时启动视觉伺服'
    n.semantic_phase='visual_search'
    try:
        while clock()-started<c.timeout:
            wall=clock()
            if n._nav_cancel_requested:return False,'任务已取消'
            if getattr(n,'servo_nav_active',False):return False,'有新的Nav2目标接管底盘，视觉伺服已停止'
            if not n.localization_ready():return False,'视觉伺服停止：定位状态未就绪或已过期'
            mx,my,myaw=base(n);x,y,yaw=base(n,'odom')
            if not room_contains(n.semantic_grid,room,mx,my):return False,'视觉伺服停止：已离开本次搜寻区域'
            pose=np.array([x,y,yaw])
            if last_pose is None or math.dist(pose[:2],last_pose[:2])>.015 or abs(math.atan2(math.sin(yaw-last_pose[2]),math.cos(yaw-last_pose[2])))>.03:
                last_pose=pose;last_motion=wall
            sim=n.get_clock().now().nanoseconds/1e9
            allowed=lambda p:object_in_room(n.semantic_grid,room,point_in_frame(n,p,'map')[:2])
            target=lock.update(list(n.target_tracks),class_id,wall,sim,[x,y],allowed)
            if target is None:
                send();ready_since=None;n.semantic_phase='visual_reacquire' if lock.last else 'visual_search'
                lost=wall-lock.last.wall if lock.last else wall-started
                if lost>(c.lost_timeout if lock.last else c.search_timeout):
                    return False,'视觉伺服未重新观测到锁定目标' if lock.last else '转向搜寻后未发现稳定的指定目标'
                if search_start is None:search_start=(yaw,wall)
                # Initial search covers all headings. During loss, only turn
                # towards this same instance's last bearing; never translate.
                if lock.last:
                    bearing=math.atan2(lock.last.position[1]-y,lock.last.position[0]-x)
                    error=math.atan2(math.sin(bearing-yaw),math.cos(bearing-yaw))
                    angular=float(np.clip(c.heading_gain*error,-c.search_speed,c.search_speed))
                    if abs(error)<.06:angular=c.search_speed*(1 if int(lost/2)%2==0 else -1)
                else:angular=c.search_speed
                if abs(angular)>.04 and wall-last_motion>c.blocked_timeout:return False,'视觉搜寻转向被局部障碍阻挡，已停止底盘'
                send(0.,angular)
                if wall-last_notice>=2:
                    notify('视觉伺服：目标暂不可见，停止平移并转向重新观测' if lock.last else '视觉搜寻：低速转向寻找指定目标')
                    last_notice=wall
                await sleep(.1);continue
            search_start=None;n.semantic_phase='visual_servo'
            try:
                target_base=point_in_frame(n,target.position,'base_footprint')
                target_arm=point_in_frame(n,target.position,'fr3_link1')
                uv=image_projection(n,target.position)
                output=command(target_base,target_arm,uv,c)
            except Exception as exc:
                send();ready_since=None
                if wall-last_notice>=2:notify('视觉伺服：等待相机/底盘坐标变换：'+str(exc));last_notice=wall
                await sleep(.1);continue
            if wall-last_notice>=1:
                notify(f'视觉伺服 class={class_id} distance={output.distance:.3f}m lateral={target_base[1]:.3f}m heading={math.degrees(output.heading):.1f}deg arm_distance={output.arm_distance:.3f}m visible={output.visible}')
                last_notice=wall
            if output.ready:
                send();n.semantic_phase='visual_servo_settling';ready_since=ready_since or wall
                if wall-ready_since>=c.settle_seconds:
                    stopped=await robot._loop.run_in_executor(robot._exec,n.base_stopped)
                    latest=lock.update(list(n.target_tracks),class_id,clock(),n.get_clock().now().nanoseconds/1e9,[x,y],allowed)
                    if stopped and latest is not None:
                        verified=command(point_in_frame(n,latest.position,'base_footprint'),point_in_frame(n,latest.position,'fr3_link1'),image_projection(n,latest.position),c)
                        if verified.ready:
                            n.semantic_phase='visual_servo_ready'
                            return True,'视觉伺服已收敛：目标持续可见、底盘停稳，已进入候选抓取工作区；最终位姿由GraspNet／MoveIt核验'
                    ready_since=None
                await sleep(.1);continue
            ready_since=None
            if target_arm[2]**2+c.distance_min**2>c.arm_radius**2:
                return False,'目标高度超出当前候选抓取工作区，底盘视觉伺服无法修正高度'
            linear,angular=output.linear,output.angular
            # Local sensors still enforce footprint collision checking. Also
            # forbid the next commanded center from leaving mapped search space.
            if abs(linear)>.001:
                px=mx+linear*.6*math.cos(myaw);py=my+linear*.6*math.sin(myaw)
                row,col=n.semantic_grid.cell(px,py)
                if not room_contains(n.semantic_grid,room,px,py) or not (0<=row<n.semantic_grid.a.shape[0] and 0<=col<n.semantic_grid.a.shape[1] and 0<=n.semantic_grid.a[row,col]<=20):
                    linear=0.
            if abs(linear)>.02 or abs(angular)>.04:
                if wall-last_motion>c.blocked_timeout:return False,'视觉伺服被局部障碍阻挡，停止接近'
            else:last_motion=wall
            send(linear,angular)
            await sleep(.1)
        return False,'视觉伺服未在限定时间内收敛，已停止底盘'
    finally:
        send()
        if n.semantic_phase!='visual_servo_ready':n.semantic_phase='visual_servo_stopped'
