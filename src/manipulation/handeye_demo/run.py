#!/usr/bin/env python3
"""Hand-eye experiment CLI. Offline commands do not require ROS or Isaac."""
import argparse
from collections import deque
import json
from pathlib import Path
import shutil
import threading
import time
import uuid
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from core import board_pose, detect, errors, fit_depth_grasp, make_assets, transform
from offline import calibrate, evaluate, report, write_json, grasp_history


def pose_matrix(message):
    t = message.transform
    return transform(Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix(),
                     [t.translation.x,t.translation.y,t.translation.z])


def camera_view(position, target, roll=0.):
    z = np.asarray(target)-position; z /= np.linalg.norm(z)
    x = np.array([1.,0,0]); x -= (x@z)*z; x /= np.linalg.norm(x)
    y = np.cross(z,x)
    R = np.column_stack([x,y,z]) @ Rotation.from_rotvec([0,0,roll]).as_matrix()
    return transform(R,position)


def run_live(args, cfg):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import Buffer, TransformListener
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, CameraInfo, JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import PoseStamped
    from control_msgs.action import GripperCommand
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'x_bot_manipulation/scripts'))
    from gripper_action import control_gripper

    class Experiment(Node):
        def __init__(self):
            super().__init__('handeye_experiment', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
            self.bridge = CvBridge(); self.buffer = Buffer(cache_time=Duration(seconds=30))
            self.listener = TransformListener(self.buffer,self)
            self.frames = cfg['frames']; self.images = {k:deque(maxlen=8) for k in ('rgb','depth','ideal','info')}
            self.truths = deque(maxlen=2000); self.acks = {}; self.status = None; self.joints = None
            self.subscriptions_owned = []
            for key,topic in {'rgb':'/handeye/rgb/image_raw','depth':'/handeye/depth/image_raw',
                              'ideal':'/handeye/debug/ideal_depth/image_raw','info':'/handeye/camera_info'}.items():
                self.subscriptions_owned.append(self.create_subscription(CameraInfo if key=='info' else Image,
                    topic,lambda msg,k=key:self.images[k].append(msg),qos_profile_sensor_data))
            self.subscriptions_owned.append(self.create_subscription(String,'/handeye/debug/truth',
                lambda msg:self.truths.append(json.loads(msg.data)),10))
            self.subscriptions_owned.append(self.create_subscription(String,'/handeye/scene_ack',
                lambda msg:self.acks.update({json.loads(msg.data)['id']:json.loads(msg.data)}),10))
            self.subscriptions_owned.append(self.create_subscription(String,'/arm_command/status', self.motion_status,10))
            self.subscriptions_owned.append(self.create_subscription(JointState, '/joint_states', lambda msg:setattr(self,'joints',msg),10))
            self.ik_client = self.create_client(GetPositionIK,'/compute_ik')
            self.command_pub = self.create_publisher(String,'/handeye/scene_command',10)
            self.motion_pub = self.create_publisher(PoseStamped,'/arm_command/pose',10)
            self.cart_pub = self.create_publisher(PoseStamped,'/arm_command/cartesian_pose',10)
            self.gripper = ActionClient(self,GripperCommand,'/fr3_gripper_controller/gripper_cmd')
            self.home_client = self.create_client(Trigger,'/robot_actions/go_home')
            self.scene_client = self.create_client(ApplyPlanningScene,'/apply_planning_scene')
            self.last_stamp = -1

        def motion_status(self, msg):
            self.status = (time.monotonic(),msg.data)

        @staticmethod
        def stamp(msg):
            return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

        def wait(self, predicate, timeout):
            end = time.monotonic()+timeout
            while time.monotonic()<end:
                result = predicate()
                if result is not None and result is not False:
                    return result
                time.sleep(.02)
            raise TimeoutError('timed out waiting for experiment data/control')

        def tf(self, parent, child, stamp=None):
            return pose_matrix(self.buffer.lookup_transform(parent,child,Time() if stamp is None else Time.from_msg(stamp),
                                                          timeout=Duration(seconds=2)))

        def bundle(self, after=None):
            threshold = max(self.last_stamp, self.get_clock().now().nanoseconds*1e-9 if after is None else after)
            def available():
                if any(not self.images[k] for k in self.images):
                    return None
                for rgb in reversed(list(self.images['rgb'])):
                    stamp = self.stamp(rgb)
                    if stamp <= threshold:
                        continue
                    matches = {k:next((m for m in reversed(list(v)) if abs(self.stamp(m)-stamp)<1e-6),None)
                               for k,v in self.images.items() if k!='rgb'}
                    if all(m is not None for m in matches.values()):
                        return rgb,matches
                return None
            rgb,matches = self.wait(available,cfg['sampling']['data_timeout'])
            for key,msg in matches.items():
                if msg.header.frame_id != rgb.header.frame_id:
                    raise ValueError('unaligned_image_frames')
            info = matches['info']; K = np.asarray(info.k).reshape(3,3)
            if info.width!=rgb.width or info.height!=rgb.height or any(abs(d)>1e-10 for d in info.d):
                raise ValueError('unsupported_image_geometry_requires_registration')
            if matches['depth'].encoding!='32FC1' or matches['ideal'].encoding!='32FC1':
                raise ValueError('depth_must_be_32FC1_meters')
            depth = self.bridge.imgmsg_to_cv2(matches['depth'],desired_encoding='32FC1').copy()
            ideal = self.bridge.imgmsg_to_cv2(matches['ideal'],desired_encoding='32FC1').copy()
            if depth.shape!=(rgb.height,rgb.width) or ideal.shape!=depth.shape:
                raise ValueError('unaligned_image_dimensions')
            self.last_stamp = self.stamp(rgb)
            return {'stamp':self.last_stamp,'stamp_msg':rgb.header.stamp,
                    'rgb':self.bridge.imgmsg_to_cv2(rgb,desired_encoding='bgr8').copy(),
                    'depth':depth,'ideal':ideal,'K':K,'D':np.asarray(info.d)}

        def truth_at(self, stamp):
            tol = cfg['sampling']['sync_tolerance_seconds']
            return self.wait(lambda: next((s for s in reversed(list(self.truths)) if abs(s['simulation_time']-stamp)<=tol),None),10)

        def scene(self, mode, **kw):
            request = {'id':uuid.uuid4().hex,'mode':mode,**kw}
            self.wait(lambda:self.command_pub.get_subscription_count()>0,20)
            if self.command_pub.get_subscription_count()!=1:
                raise RuntimeError('multiple scene bridges found; use a different ROS domain')
            end=time.monotonic()+20
            while request['id'] not in self.acks and time.monotonic()<end:
                self.command_pub.publish(String(data=json.dumps(request)))
                time.sleep(.25)
            ack=self.acks.get(request['id'])
            if ack is None:
                raise TimeoutError('scene command acknowledgement missing')
            if not ack['ok']:
                raise RuntimeError(ack.get('error','scene command failed'))
            time.sleep(cfg['sampling']['settle_seconds'])

        def move(self, T, cartesian=False):
            msg = PoseStamped(); msg.header.frame_id = self.frames['base']; msg.header.stamp=self.get_clock().now().to_msg()
            p = msg.pose.position; p.x,p.y,p.z = map(float,T[:3,3])
            q = msg.pose.orientation; q.x,q.y,q.z,q.w = map(float,Rotation.from_matrix(T[:3,:3]).as_quat())
            if not cartesian and not self.reachable_msg(msg):
                return False
            self.status = None; started=time.monotonic()
            (self.cart_pub if cartesian else self.motion_pub).publish(msg)
            status = self.wait(lambda:self.status if self.status and self.status[0]>=started else None,cfg['sampling']['motion_timeout'])
            if status[1]!='SUCCESS':
                return False
            time.sleep(cfg['sampling']['settle_seconds'])
            measured = self.tf(self.frames['base'],self.frames['tcp'])
            e = errors(measured,T)
            return e['translation_mm']<10 and e['rotation_deg']<3

        def reachable_msg(self, msg):
            if not self.ik_client.wait_for_service(timeout_sec=10):
                raise RuntimeError('MoveIt IK service unavailable')
            self.wait(lambda:self.joints,10)
            request=GetPositionIK.Request()
            request.ik_request.group_name='fr3_arm'; request.ik_request.ik_link_name=self.frames['tcp']
            request.ik_request.pose_stamped=msg
            request.ik_request.robot_state.joint_state=self.joints
            request.ik_request.avoid_collisions=True
            request.ik_request.timeout.nanosec=200000000
            future=self.ik_client.call_async(request)
            self.wait(lambda:future.done(),10)
            return future.result().error_code.val==1

        def reachable(self, T):
            msg=PoseStamped();msg.header.frame_id=self.frames['base']
            msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=map(float,T[:3,3])
            q=Rotation.from_matrix(T[:3,:3]).as_quat()
            msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=map(float,q)
            return self.reachable_msg(msg)

        def observe(self, camera_pose):
            # Nominal URDF transforms are only used to choose safe observation motions.
            hand_camera = self.tf(self.frames['hand'],self.frames['camera'])
            hand_tcp = self.tf(self.frames['hand'],self.frames['tcp'])
            return self.move(camera_pose @ np.linalg.inv(hand_camera) @ hand_tcp)

        def home(self):
            if not self.home_client.wait_for_service(timeout_sec=10):
                raise RuntimeError('go_home service unavailable')
            future=self.home_client.call_async(Trigger.Request())
            self.wait(lambda:future.done(),cfg['sampling']['motion_timeout'])
            if not future.result().success:
                raise RuntimeError('home_reset_failed: refusing next trial')
            time.sleep(cfg['sampling']['settle_seconds'])

        def collision_table(self):
            if not self.scene_client.wait_for_service(timeout_sec=30):
                raise RuntimeError('MoveIt planning scene unavailable')
            scene=PlanningScene(); scene.is_diff=True
            obj=CollisionObject(); obj.id='handeye_table'; obj.header.frame_id=self.frames['base']; obj.operation=CollisionObject.ADD
            primitive=SolidPrimitive(); primitive.type=SolidPrimitive.BOX; primitive.dimensions=[.65,.65,.04]
            pose=Pose(); pose.orientation.w=1.; pose.position.x=.65; pose.position.z=cfg['scene']['table_top']-.02
            obj.primitives=[primitive]; obj.primitive_poses=[pose]; scene.world.collision_objects=[obj]
            future=self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
            self.wait(lambda:future.done(),30)
            if not future.result().success:
                raise RuntimeError('failed to insert table collision geometry')

        def save_frame(self, directory, b):
            directory.mkdir(parents=True,exist_ok=True)
            cv2.imwrite(str(directory/'rgb.png'),b['rgb'])
            np.savez_compressed(directory/'depth.npz',sensor=b['depth'],ideal=b['ideal'])
            write_json(directory/'camera.json',{'K':b['K'].tolist(),'D':b['D'].tolist(),
                'stamp':b['stamp'],'frame':self.frames['camera'],'depth_units':'m','depth_convention':'optical_z','depth_registered_to_rgb':True})

        def collect(self):
            rng = np.random.default_rng(42)
            # Fixed candidate seed keeps independent training/test poses reproducible.
            candidates=[]
            for split, fraction in [('train',.75),('test',.25)]:
                for _ in range(round(cfg['sampling']['candidates']*fraction)):
                    center=np.asarray(cfg['scene']['board_center'])
                    position=center+np.array([rng.uniform(-.22,-.08),rng.uniform(-.12,.12),rng.uniform(.22,.34)])
                    candidates.append((split,camera_view(position,center,rng.uniform(-.55,.55))))
            write_json(args.output/'candidate_poses.json',[{'split':s,'T_base_camera_nominal':T.tolist()} for s,T in candidates])
            directory=args.output/'grid'; directory.mkdir(parents=True,exist_ok=True)
            if (directory/'samples.json').exists():
                raise RuntimeError(f'{directory}: samples already exist; choose a fresh output directory')
            self.scene('calibration')
            samples=[]; truths=[]; skipped=[]
            write_json(directory/'samples.json',samples); write_json(directory/'truth.json',truths)
            for index,(split,T) in enumerate(candidates):
                if sum(s['split']==split for s in samples)>=cfg['sampling'][split]:
                    continue
                label=f'{index:03d}'
                try:
                    if not self.observe(T):
                        skipped.append({'candidate':index,'split':split,'reason':'unreachable'})
                        write_json(directory/'skipped.json',skipped)
                        continue
                    b=self.bundle(); self.save_frame(directory/label,b)
                    detected=detect(b['rgb'])
                    target,reprojection=board_pose(detected,b['K'],b['D'],cfg['sampling']['max_reprojection_px'])
                    hand=self.tf(self.frames['base'],self.frames['hand'],b['stamp_msg'])
                    truth=self.truth_at(b['stamp'])
                    samples.append({'sample_id':label,'candidate':index,'split':split,'stamp':b['stamp'],
                        'T_base_hand':hand.tolist(),'T_camera_target':target.tolist(),'reprojection_px':reprojection,
                        'corners':{str(k):v.tolist() for k,v in detected.items()},'K':b['K'].tolist(),'D':b['D'].tolist()})
                    truths.append({'sample_id':label,**truth})
                    self.get_logger().info(f'AprilGrid {split}: accepted {label}, reprojection {reprojection:.3f} px')
                except TimeoutError:
                    # A timed-out motion may still be executing: do not issue another command.
                    write_json(directory/'samples.json',samples); write_json(directory/'truth.json',truths)
                    write_json(directory/'skipped.json',skipped)
                    raise
                except Exception as error:
                    # TF and detection failures reject only this stationary sample.
                    skipped.append({'candidate':index,'split':split,'reason':str(error)})
                write_json(directory/'samples.json',samples); write_json(directory/'truth.json',truths)
                write_json(directory/'skipped.json',skipped)

        def grasp(self):
            result=json.loads((args.output/'calibration.json').read_text())
            if not result['complete']:
                raise RuntimeError('incomplete AprilGrid calibration')
            path=args.output/'grasps.json'
            trials=grasp_history(args.output,cfg,result,args.resume)
            completed={t['trial'] for t in trials}
            extrinsic=np.asarray(result['T_hand_camera'])
            for index, trial in enumerate(cfg['grasp']['trials']):
                if index in completed:
                    continue
                frame_dir=args.output/'grasp_frames'/str(index)
                if frame_dir.exists():
                    archived=args.output/'diagnostics'/('aborted_grasp_'+uuid.uuid4().hex)
                    archived.mkdir(parents=True,exist_ok=True)
                    shutil.move(str(frame_dir),str(archived/str(index)))
                entry={'trial':index,'placement':trial,'outcome':'perception_failure'}
                self.scene('grasp',trial=trial)
                try:
                    if not control_gripper(self,self.gripper,.04):
                        raise ValueError('gripper_open_failed')
                    view=camera_view(np.asarray(cfg['grasp']['observation']),[trial[0],trial[1],cfg['scene']['table_top']+.04])
                    if not self.observe(view):
                        entry['outcome']='planning_failure'; raise ValueError('observation_unreachable')
                    perceived=None
                    for attempt in range(cfg['grasp']['attempts']):
                        try:
                            b=self.bundle(); self.save_frame(args.output/'grasp_frames'/str(index)/str(attempt),b)
                            detections=detect(b['rgb'])
                            if 100 not in detections:
                                raise ValueError('cube_tag_not_detected')
                            corners=detections[100]
                            camera_grasp,depth_metrics=fit_depth_grasp(b['depth'],corners,b['K'],
                                cfg['grasp']['max_plane_residual_m'],cfg['grasp']['min_depth_points'],
                                cfg['grasp']['tcp_center_backoff_m'])
                            hand=self.tf(self.frames['base'],self.frames['hand'],b['stamp_msg'])
                            perceived=(b,camera_grasp,hand,depth_metrics,corners)
                            break
                        except (ValueError,cv2.error) as error:
                            entry.setdefault('perception_attempts',[]).append(str(error))
                    if perceived is None:
                        raise ValueError('depth_perception_failed_after_three_attempts')
                    b,camera_grasp,hand,depth_metrics,corners=perceived
                    truth=self.truth_at(b['stamp'])  # scoring only
                    goal=hand @ extrinsic @ camera_grasp
                    entry['depth']=depth_metrics
                    # Scoring mask is independent of perception output; never replaces depth.
                    mask=np.zeros(b['depth'].shape,np.uint8)
                    cv2.fillConvexPoly(mask,np.round(corners.mean(axis=0)+.65*(corners-corners.mean(axis=0))).astype(np.int32),1)
                    valid=(mask>0)&np.isfinite(b['depth'])&np.isfinite(b['ideal'])&(b['depth']>0)&(b['ideal']>0)
                    if valid.any():
                        delta=(b['depth'][valid]-b['ideal'][valid])*1000
                        entry['depth'].update(depth_bias_mm=float(delta.mean()),depth_rmse_mm=float(np.sqrt(np.mean(delta**2))))
                    entry['initial_truth']=truth
                    before=np.asarray(truth['T_base_cube'])
                    expected=before.copy()
                    # top-down grasp: TCP x = approach, TCP y = cube/tag x.
                    expected[:3,:3]=np.column_stack([-before[:3,2], before[:3,0], -before[:3,1]])
                    expected[:3,3]+=cfg['grasp']['tcp_center_backoff_m']*before[:3,2]
                    entry['outcome']='planning_failure'
                    feasible=None
                    angles=[0.,np.pi,np.pi/2,-np.pi/2] if cfg['grasp'].get('square_grasp_symmetry',True) else [0.]
                    for angle in angles:
                        symmetry=transform(Rotation.from_rotvec([angle,0,0]).as_matrix())
                        candidate=goal@symmetry
                        pre=candidate.copy(); pre[:3,3]-=cfg['grasp'].get('pregrasp_distance_m',.05)*candidate[:3,0]
                        lifted=candidate.copy(); lifted[2,3]+=cfg['grasp']['lift_m']
                        if all(self.reachable(T) for T in (pre,candidate,lifted)):
                            feasible=(candidate,pre,symmetry);break
                    if feasible is None:
                        raise ValueError('no_collision_free_approach_grasp_lift_ik')
                    goal,pre,symmetry=feasible
                    expected=expected@symmetry
                    entry['T_base_grasp']=goal.tolist(); entry['grasp_symmetry_deg']=float(np.degrees(angle))
                    if not self.move(pre) or not self.move(goal,cartesian=True):
                        raise ValueError('pregrasp_or_approach_failed')
                    latest=self.truth_at(self.get_clock().now().nanoseconds*1e-9)
                    entry['preclose_truth']=latest
                    entry['tcp_error']=errors(np.asarray(latest['T_base_tcp']),expected)
                    entry['outcome']='not_grasped'
                    if not control_gripper(self,self.gripper,0.):
                        raise ValueError('gripper_close_failed')
                    lift=goal.copy(); lift[2,3]+=cfg['grasp']['lift_m']
                    if not self.move(lift,cartesian=True):
                        entry['outcome']='planning_failure'; raise ValueError('lift_motion_failed')
                    raised=self.truth_at(self.get_clock().now().nanoseconds*1e-9)
                    entry['raised_truth']=raised
                    raised_cube=np.asarray(raised['T_base_cube']); raised_tcp=np.asarray(raised['T_base_tcp'])
                    entry['object_lift_mm']=float((raised_cube[2,3]-before[2,3])*1000)
                    if raised_cube[2,3]-before[2,3] < cfg['grasp']['minimum_object_lift_m']:
                        raise ValueError('object_not_lifted')
                    offset=np.linalg.inv(raised_tcp)@raised_cube
                    entry['outcome']='dropped'
                    start=self.get_clock().now().nanoseconds*1e-9
                    hold_end=start+cfg['grasp']['hold_seconds']
                    self.wait(lambda:self.get_clock().now().nanoseconds*1e-9>=hold_end,30)
                    hold=[s for s in list(self.truths) if start<=s['simulation_time']<=hold_end]
                    entry['hold_truth']=hold
                    if (not hold or hold[0]['simulation_time']>start+.1 or hold[-1]['simulation_time']<hold_end-.1
                        or any(b['simulation_time']-a['simulation_time']>.1 for a,b in zip(hold,hold[1:]))):
                        raise ValueError('missing_hold_telemetry')
                    for s in hold:
                        cube=np.asarray(s['T_base_cube']); tcp=np.asarray(s['T_base_tcp'])
                        if cube[2,3]-before[2,3]<cfg['grasp']['minimum_object_lift_m'] or errors(np.linalg.inv(tcp)@cube,offset)['translation_mm']>10:
                            raise ValueError('object_slipped_or_dropped')
                    entry['outcome']='success'
                except TimeoutError as error:
                    entry['reason']=str(error); trials.append(entry); write_json(path,trials); report(args.output)
                    raise
                except Exception as error:
                    entry['reason']=str(error)
                trials.append(entry); write_json(path,trials); report(args.output)
                self.get_logger().info(f'AprilGrid trial {index}: {entry["outcome"]}')
                control_gripper(self,self.gripper,.04)
                # A common joint home pose standardizes the next trial's IK seed and clears the object.
                self.home()

    rclpy.init()
    node=Experiment(); executor=MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    try:
        node.bundle(); node.collision_table()
        if args.command in ('collect','all'):
            node.collect()
            if not calibrate(args.output,cfg):
                report(args.output)
                raise RuntimeError('AprilGrid 采样不足或求解失败，详见 report.md / skipped.json')
            evaluate(args.output)
        if args.command in ('grasp','all'):
            node.grasp()
    finally:
        executor.shutdown(); node.destroy_node(); rclpy.try_shutdown(); thread.join(timeout=5)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['assets','collect','calibrate','evaluate','grasp','report','all'])
    parser.add_argument('--resume',action='store_true',help='grasp only: continue missing trials with unchanged frozen inputs')
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('config.yaml'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); args.output=args.output.resolve(); args.output.mkdir(parents=True,exist_ok=True)
    if args.resume and args.command!='grasp':
        parser.error('--resume is only supported by grasp')
    cfg=yaml.safe_load(args.config.read_text())
    if cfg['scene']['cube_size'] != .04:
        raise ValueError('this standard fixture requires scene.cube_size=0.04 m')
    if args.command=='assets':
        make_assets(args.output/'assets'); return
    if args.command in ('collect','all'):
        if (args.output/'grid/samples.json').exists():
            raise RuntimeError('output contains samples; use a fresh directory')
        if args.config.resolve()!=(args.output/'config.yaml').resolve():
            shutil.copyfile(args.config,args.output/'config.yaml')
        write_json(args.output/'metadata.json',{'opencv':cv2.__version__,'numpy':np.__version__,
            'detector':'AprilTag36h11 / contour refinement','matrix_convention':'T_a_b maps b into a',
            'sensor':'SingleViewDepthCameraSensor','isaac_required':'6.1','candidate_seed':42,
            'depth_plane_ransac_seed':0,'sensor_noise_seed':'SDK stochastic, not controlled',
            'calibration_depth_registration':'native_left_XYZ_rays_with_noisy_distance_zbuffer'})
    elif (args.output/'config.yaml').exists():
        cfg=yaml.safe_load((args.output/'config.yaml').read_text())
    if args.command=='calibrate':
        complete=calibrate(args.output,cfg); report(args.output)
        if not complete:
            raise SystemExit('incomplete calibration; see report.md')
    elif args.command=='evaluate':
        evaluate(args.output)
    elif args.command=='report':
        report(args.output)
    else:
        run_live(args,cfg)


if __name__=='__main__':
    main()
