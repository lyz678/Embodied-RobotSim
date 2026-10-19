"""Truth-free category-conditioned ACT control in the embodied scene."""
import argparse
import json
import os
import pickle
import select
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image,JointState,CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger,SetBool
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from vision_msgs.msg import Detection3DArray
from common.fr3_ros import Observations,stamp
from common.fr3_geometry import interpolate
from common.fr3_common import SAM3_PYTHON
from common.grasp_return_task import HOME,smooth_command
from common.fr3_gripper import apply_grip_hold

HERE=Path(__file__).resolve().parent
SETTINGS=json.loads((HERE/'config.json').read_text())
# Shared gripper action status validation.
sys.path.insert(0,str(HERE.parents[1]/'manipulation/x_bot_manipulation/scripts'))
from gripper_action import control_gripper


class PolicyObservations(Observations):
    """Reuse sensor decoding only. Never subscribe to simulation truth or reset topics."""
    def __init__(self,node,cfg):
        self.node=node;self.cfg=cfg;self.lock=threading.RLock()
        self.image=self.joints=None;self.images=deque(maxlen=120);self.joint_history=deque(maxlen=1200)
        self.depths=deque(maxlen=120);self.calibration=None;self.sensor_error=None
        self.rgb_spec=None;self.base=None;self.detected={}
        self.subs=[node.create_subscription(Image,cfg['image_topic'],self.on_image,qos_profile_sensor_data),
                   node.create_subscription(JointState,'/joint_states',self.on_joints,qos_profile_sensor_data),
                   node.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data),
                   node.create_subscription(Detection3DArray,'/yoloe_multi_text_prompt/detections_3d',self.on_detections,10)]
        if cfg.get('visual_mode','rgb')!='rgb':
            self.subs.extend((node.create_subscription(Image,cfg['depth_topic'],self.on_depth,qos_profile_sensor_data),
                              node.create_subscription(CameraInfo,cfg['camera_info_topic'],self.on_info,qos_profile_sensor_data)))
    def on_odom(self,msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        yaw=np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        with self.lock:self.base=(stamp(msg),np.array([p.x,p.y,yaw]),time.monotonic())
    def on_detections(self,msg):
        now=self.node.get_clock().now().nanoseconds/1e9
        if not -.1<=now-stamp(msg)<=SETTINGS['detector_max_age']:return
        with self.lock:
            for d in msg.detections:
                for r in d.results:
                    try:cid=int(r.hypothesis.class_id)
                    except ValueError:continue
                    if r.hypothesis.score>=SETTINGS['detector_confidence']:
                        self.detected[cid]=(time.monotonic(),stamp(msg),float(r.hypothesis.score))
    def snapshot(self,**unused):
        with self.lock:
            if self.sensor_error:raise ValueError(self.sensor_error)
            now=self.node.get_clock().now().nanoseconds/1e9
            for it,image in reversed(self.images):
                if not 0<=now-it<=.3:continue
                joint=interpolate(self.joint_history,it,self.cfg.get('joint_sync_tolerance',.01))
                if joint is None:continue
                (q,v),bracket=joint
                row=dict(time=it,image_time=it,image=image.copy(),qpos=q.copy(),qvel=v.copy(),now=now,joint_bracket=bracket)
                if self.cfg.get('visual_mode','rgb')!='rgb':
                    if not self.depths or self.calibration is None:continue
                    d=min(self.depths,key=lambda d:abs(d[0]-it))
                    if self.rgb_spec!=self.calibration[:3] or d[3]!=self.rgb_spec:raise ValueError('RGB-D alignment mismatch')
                    if abs(d[0]-it)>self.cfg.get('depth_sync_tolerance',.02):continue
                    row.update(depth=d[1].copy(),depth_valid=d[2].copy(),intrinsics=self.calibration[3].copy())
                return row
        return None


def contact_and_home(q,departed,latched):
    return bool(departed and latched and .008<float(q[7])<.055 and np.max(abs(q[:7]-HOME))<.08)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--class-id',type=int,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--model',choices=['rgb','rgbd','rgbxyz'],default='rgb')
    parser.add_argument('--checkpoint',type=Path)
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    checkpoint=str(args.checkpoint or SETTINGS['checkpoints'][args.model])
    trace=[];frames=[];worker=None;node=None;executor=None;spin=None;last_goal=None;obs=None;cfg=None;stop=[False]
    result=dict(success=False,failure_reason='initialization',backend='act',target_class=args.class_id,checkpoint=checkpoint,model=args.model)
    signal.signal(signal.SIGTERM,lambda *a:stop.__setitem__(0,True))
    signal.signal(signal.SIGINT,lambda *a:stop.__setitem__(0,True))
    def read_worker(timeout):
        if not select.select([worker.stdout],[],[],timeout)[0]:raise TimeoutError('ACT worker timeout')
        return pickle.load(worker.stdout)
    try:
        worker=subprocess.Popen([str(SAM3_PYTHON),'-m','evaluation.fr3_inference_worker',checkpoint],
                                cwd=SETTINGS['act_root'],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        ready=read_worker(60);cfg=ready['config'];result.update(training_step=ready['step'],visual_mode=cfg['visual_mode'])
        if cfg['visual_mode']!=args.model:raise ValueError('Model visual mode mismatch')
        if args.class_id not in cfg.get('target_classes',[]):raise ValueError('Checkpoint does not support requested category')
        if cfg.get('visual_encoder','resnet18')!='resnet18' or cfg.get('target_conditioning','category')!='category':raise ValueError('Deployment requires RGB ResNet18 + category policy')
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('embodied_act_policy',parameter_overrides=[Parameter('use_sim_time',value=True)])
        obs=PolicyObservations(node,cfg);arm=node.create_publisher(JointTrajectory,'/fr3_arm_controller/joint_trajectory',10)
        base=node.create_publisher(Twist,'/cmd_vel',10);gripper=ActionClient(node,GripperCommand,'/fr3_gripper_controller/gripper_cmd')
        executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node);spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
        def future(f,timeout):
            deadline=time.monotonic()+timeout
            while not f.done():
                if stop[0] or time.monotonic()>deadline:raise TimeoutError('ROS operation interrupted or timed out')
                time.sleep(.01)
            return f.result()
        def toggle(name,enabled):
            client=node.create_client(SetBool,name)
            try:
                if client.wait_for_service(timeout_sec=2):
                    request=SetBool.Request();request.data=enabled;future(client.call_async(request),5)
            finally:node.destroy_client(client)
        def command(q,duration=.1):
            msg=JointTrajectory();msg.joint_names=cfg['joint_names'][:7]
            point=JointTrajectoryPoint();point.positions=np.asarray(q[:7],float).tolist();point.time_from_start=Duration(sec=0,nanosec=int(duration*1e9));msg.points=[point];arm.publish(msg)
        def grip(position):
            nonlocal last_goal
            goal=GripperCommand.Goal();goal.command.position=float(position);goal.command.max_effort=10.
            last_goal=future(gripper.send_goal_async(goal),3)
            if not last_goal.accepted:raise RuntimeError('Gripper action rejected')
        toggle('/graspnet_node/enable_inference',False)
        toggle('/yoloe_multi_text_prompt/enable_inference',True)
        # Initial Home/open are preparation, all subsequent arm targets come from ACT.
        base.publish(Twist());home=node.create_client(Trigger,'/robot_actions/go_home')
        try:
            if not home.wait_for_service(timeout_sec=15):raise TimeoutError('Home service missing')
            response=future(home.call_async(Trigger.Request()),45)
            if not response.success:raise RuntimeError('Initial Home failed: '+response.message)
        finally:node.destroy_client(home)
        if not control_gripper(node,gripper,.06,20):raise RuntimeError('Initial gripper open failed')
        start_wait=time.monotonic();initial=None
        while time.monotonic()-start_wait<12 and not stop[0]:
            row=obs.snapshot();det=obs.detected.get(args.class_id)
            if row is not None and obs.base and time.monotonic()-obs.base[2]<.5 and det and time.monotonic()-det[0]<=SETTINGS['detector_max_age']:
                initial=row;break
            time.sleep(.03)
        if initial is None:raise RuntimeError('Fresh RGB/joints/odom or requested YOLOE category missing')
        result['yoloe_target']=dict(class_id=args.class_id,confidence=det[2],source_time=det[1])
        toggle('/yoloe_multi_text_prompt/enable_inference',False)
        started=initial['time'];previous=started-1;previous_command=initial['qpos'].copy();velocity=np.zeros(7);bank=[]
        base_initial=obs.base[1].copy();latched=False;departed=False;stable_since=None;last_grip=None;last_fresh=time.monotonic();latencies=[]
        result['start_time']=started;wall_started=time.monotonic()
        print('ACT START',json.dumps(result),flush=True)
        while time.monotonic()-wall_started<SETTINGS['max_seconds']*6 and not stop[0]:
            row=obs.snapshot()
            if row is None or row['time']-previous<.095:
                if time.monotonic()-last_fresh>5:raise TimeoutError('Fresh policy observations lost; actions paused')
                time.sleep(.003);continue
            last_fresh=time.monotonic()
            if row['time']-started>=SETTINGS['max_seconds']:break
            if not obs.base or time.monotonic()-obs.base[2]>.5:raise RuntimeError('Odometry stale; stopping policy')
            delta=obs.base[1]-base_initial;delta[2]=np.arctan2(np.sin(delta[2]),np.cos(delta[2]))
            if np.linalg.norm(delta[:2])>.05 or abs(delta[2])>.08:raise RuntimeError('Base drift exceeded 5cm / 0.08rad')
            previous=row['time'];base.publish(Twist());q=row['qpos'];departed|=bool(np.max(abs(q[:7]-HOME))>.15)
            request=dict(qpos=q,image=row['image'],target_class=args.class_id)
            if cfg['visual_mode']!='rgb':request.update({k:row[k] for k in ('depth','depth_valid','intrinsics')})
            tick=time.perf_counter();pickle.dump(request,worker.stdin);worker.stdin.flush();chunk=read_worker(5)
            elapsed=(time.perf_counter()-tick)*1000;latencies.append(elapsed)
            if not np.isfinite(chunk).all():raise RuntimeError('Nonfinite ACT actions')
            bank.append((row['image_time'],chunk));index=lambda t:max(0,int(round((row['now']-t)*cfg['control_hz'])))
            bank=[(t,c) for t,c in bank if index(t)<len(c)]
            weights=np.exp(-SETTINGS['temporal_decay']*np.arange(len(bank)))
            target=np.average(np.stack([c[index(t)] for t,c in bank]),axis=0,weights=weights)
            limits=np.r_[np.repeat(cfg['max_joint_step'],7),cfg['max_gripper_step']]
            bounded=q+np.clip(np.clip(target,cfg['joint_lower'],cfg['joint_upper'])-q,-limits,limits)
            bounded,velocity=smooth_command(bounded,previous_command,velocity,.1,cfg);previous_command=bounded.copy()
            bounded,latched=apply_grip_hold(target,bounded,latched,SETTINGS['gripper_mode']);command(bounded)
            if last_grip is None or abs(bounded[7]-last_grip)>.002:
                grip(bounded[7]);last_grip=float(bounded[7])
            stable=contact_and_home(q,departed,latched)
            stable_since=(row['time'] if stable_since is None else stable_since) if stable else None
            trace.append(dict(time=row['time'],qpos=q.tolist(),qvel=row['qvel'].tolist(),raw_action=target.tolist(),action=bounded.tolist(),
                              gripper_latched=bool(latched),inference_ms=elapsed,image_age_ms=(row['now']-row['image_time'])*1000,base_odom=obs.base[1].tolist()))
            frames.append(row['image'])
            if len(trace)%50==0:print('ACT PROGRESS',len(trace),'elapsed',round(row['time']-started,2),'latched',latched,'home_error',float(np.max(abs(q[:7]-HOME))),flush=True)
            if stable_since is not None and row['time']-stable_since>=2.:
                result.update(success=True,failure_reason=None);break
        if not result['success']:result['failure_reason']='interrupted' if stop[0] else 'ACT did not complete contact and Home within deadline'
        result.update(steps=len(trace),departed_home=bool(departed),gripper_latched=bool(latched),simulation_seconds=previous-started,
                      inference_latency_ms=dict(median=float(np.median(latencies)),p95=float(np.percentile(latencies,95))) if latencies else {})
    except Exception as exc:
        import traceback
        traceback.print_exc();result.update(success=False,failure_reason=str(exc))
    finally:
        if node and obs:
            try:
                row=obs.snapshot()
                if row is not None:command(row['qpos'],.2)
                base.publish(Twist())
                toggle('/yoloe_multi_text_prompt/enable_inference',True)
            except Exception:pass
        if worker:
            if worker.poll() is None:
                try:pickle.dump(None,worker.stdin);worker.stdin.flush();worker.wait(timeout=5)
                except Exception:worker.kill();worker.wait()
        (out/'result.json').write_text(json.dumps(result,indent=2))
        (out/'trajectory.json').write_text(json.dumps(dict(result=result,trace=trace),indent=2))
        if frames:
            writer=cv2.VideoWriter(str(out/'act_rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10,(320,320))
            for image in frames:writer.write(cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
            writer.release()
        if executor:executor.shutdown()
        if spin:spin.join(timeout=3)
        if node:node.destroy_node();rclpy.shutdown()
        print('ACT RESULT',json.dumps(result),flush=True)
    return 0 if result['success'] else 1

if __name__=='__main__':sys.exit(main())
