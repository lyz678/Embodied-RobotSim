#!/usr/bin/env python3
"""
Pick and Place Demo - Using Real-time YOLOE Detection
使用实时 YOLOE 检测进行抓取和放置
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Quaternion
from vision_msgs.msg import Detection3DArray, Detection3D
from visualization_msgs.msg import Marker
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger, SetBool
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math
import json
import copy
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data
from physical_pick_verification import CLASS_OBJECTS, lifted, retained, in_bin
import time
import threading
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from collections import deque
from gripper_action import control_gripper as execute_gripper
from robot_action_services import RobotActionServices

class PickAndPlaceDemo(RobotActionServices, Node):
    """使用实时 YOLOE 检测的抓取放置节点"""
    
    def __init__(self):
        super().__init__('pick_and_place_demo')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅实时检测结果
        self.sub_detections = self.create_subscription(
            Detection3DArray, 
            '/graspnet/detections_3d', 
            self.detections_callback, 
            10
        )
        
        # 机械臂控制
        self.pub_arm = self.create_publisher(PoseStamped, '/arm_command/pose', 10)
        self.pub_cartesian = self.create_publisher(PoseStamped, '/arm_command/cartesian_pose', 10)
        self.pub_vis = self.create_publisher(Marker, '/target_pose_marker', 10)
        self.marker_id = 0
        self.gripper_client = ActionClient(self, GripperCommand, '/fr3_gripper_controller/gripper_cmd')
        
        # 点云过滤控制
        self.pub_cloud_filter = self.create_publisher(Int32, 'set_cloud_filter', 10)
        
        # CollisionObject发布器用于清除octomap
        self.pub_collision_object = self.create_publisher(CollisionObject, 'collision_object', 10)
   
        # 机器人动作服务
        self.go_home_client = self.create_client(Trigger, '/robot_actions/go_home')
        self.scan_client = self.create_client(Trigger, '/robot_actions/scan')
        
        # 推理控制服务客户端
        self.yoloe_cloud_client = self.create_client(SetBool, '/yoloe_multi_text_prompt/enable_pointcloud')
        self.graspnet_inference_client = self.create_client(SetBool, '/graspnet_node/enable_inference')
        self.depth_inference_client = self.create_client(SetBool, '/stereo_matching_node/enable_inference')
        
        # 状态订阅
        self.create_subscription(String, '/arm_command/status', self.status_callback, 10)
        
        # 配置参数
        self.declare_parameter('table_classes', [3, 2, 9])  # book, cup, coke
        self.declare_parameter('floor_classes', [4, 6])     # bottle, shoe
        self.declare_parameter('min_confidence', 0.25)
        self.declare_parameter('detection_timeout', 5.0)
        self.declare_parameter('floor_tcp_backoff', 0.035)
        self.declare_parameter('release_step', 0.003)
        self.declare_parameter('release_pause', 0.35)
        self.declare_parameter('elongated_place_classes', [3,6])
        self.declare_parameter('elongated_place_yaw', 0.7853981633974483)
        self.declare_parameter('thin_object_tcp_backoff', 0.02)
        self.declare_parameter('thin_object_classes', [9])
        self.declare_parameter('thin_object_approach_pitch', 0.52)
        self.declare_parameter('top_grasp_classes', [6])
        self.declare_parameter('fixture_config', os.path.join(get_package_share_directory('x_bot_manipulation'), 'config', 'manipulation_fixture.yaml'))
        
        self.table_classes = self.get_parameter('table_classes').value
        self.floor_classes = self.get_parameter('floor_classes').value
        self.target_classes = self.table_classes + self.floor_classes
        
        self.min_confidence = self.get_parameter('min_confidence').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        
        # 初始位置配置 (Cartesian Poses from User Verification)
        self.POSES = {
            'PickFloor': {
                'pos': [-0.01, 0.48, 0.51],
                'rot': [-0.50, 0.48, 0.52, 0.50]
            },
            'Place': {
                'pos': [0.0, -0.80, 0.58],
                'rot': [0.29, 0.28, -0.66, 0.64]
            },
            # 'palce':{
            #     'pos': [0.66, -0.03, -0.02],
            #     'rot': [-0.01, 0.31, 0.02, 0.95]
            # },
        }
        
        # 从YOLOE配置文件读取类别ID到名称的映射
        self.declare_parameter('class_config', '')
        self.class_rgb = {}
        self.class_id_to_name = self.load_class_mapping_from_config()
        self.floor_axes = {}
        self.object_boxes = {}
        self.inspect_floor = False
        self.create_subscription(PointCloud2, '/yoloe_multi_text_prompt/pointcloud_semantic', self.floor_geometry_callback, qos_profile_sensor_data)
        
        # 状态变量
        self.latest_detections = []  # 最新的检测结果
        self.detection_received_at = 0.0
        self.detection_lock = threading.Lock()
        self.last_motion_status = None
        
        self.declare_parameter('verify_physical_pick', True)
        self.verify_physical = self.get_parameter('verify_physical_pick').value
        self.truth = None
        self.truth_at = 0.0
        self.completed_classes = set()
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.pub_target_bounds = self.create_publisher(CollisionObject, '/pick_target_bounds', 10)
        self.declare_parameter('physical_state_topic', '/simulation/debug/object_states')
        self.create_subscription(String, self.get_parameter('physical_state_topic').value, self.truth_callback, 10)

        # 启动逻辑线程
        self.logic_thread = threading.Thread(target=self.run_logic_loop, daemon=True)
        self.logic_thread.start()
        
        self.get_logger().info("=== Pick and Place Demo - Real-time YOLOE Edition ===")
        target_names = [self.class_id_to_name.get(cid, f"unknown({cid})") for cid in self.target_classes]
        self.get_logger().info(f"Target classes: {self.target_classes} ({', '.join(target_names)})")
        self.get_logger().info(f"Min confidence: {self.min_confidence}")
    
    def load_class_mapping_from_config(self):
        """从YOLOE配置文件中加载类别ID到名称的映射"""
        from ament_index_python.packages import get_package_share_directory
        config_path = os.path.join(get_package_share_directory('yoloe_infer'), 'configs', 'config.yaml')
        configured = self.get_parameter('class_config').value
        if configured:
            config_path = configured

        class_id_to_name = {}
        
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    
                if 'classes' in config:
                    for cls in config['classes']:
                        if 'id' in cls and 'name' in cls:
                            class_id_to_name[cls['id']] = cls['name']
                            rgb = cls.get('color', [255,255,255])
                            self.class_rgb[cls['id']] = (rgb[0]<<16)|(rgb[1]<<8)|rgb[2]
                    
                    self.get_logger().info(f"Loaded {len(class_id_to_name)} class mappings from {config_path}")
                else:
                    self.get_logger().warn(f"No 'classes' section found in {config_path}")
            else:
                self.get_logger().warn(f"Config file not found: {config_path}, using empty mapping")
        except Exception as e:
            self.get_logger().error(f"Failed to load class mapping from {config_path}: {e}")
        
        return class_id_to_name

    def install_fixture_collisions(self):
        if not self.scene_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("Planning scene service unavailable")
        with open(self.get_parameter('fixture_config').value) as stream:
            config = yaml.safe_load(stream)
        obj = CollisionObject()
        obj.id = 'manipulation_work_table'
        obj.header.frame_id = config['frame']
        obj.pose.orientation.w = 1.0
        obj.operation = CollisionObject.ADD
        for box in config['boxes']:
            shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=box['size'])
            pose = Pose();pose.orientation.w = 1.0
            pose.position.x,pose.position.y,pose.position.z = box['position']
            obj.primitives.append(shape);obj.primitive_poses.append(pose)
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True;request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects.append(obj)
        future = self.scene_client.call_async(request)
        deadline = time.monotonic()+10.0
        while not future.done() and time.monotonic()<deadline:
            time.sleep(.1)
        if not future.done() or not future.result().success:
            raise RuntimeError("Failed to install known work-table collision geometry")
        self.pub_collision_object.publish(obj)
        self.get_logger().info("Installed source-model table collisions (tabletop, legs and braces)")

    def floor_geometry_callback(self, msg):
        """Segmented depth supplies planning payload bounds and floor long axes."""
        try:
            points = point_cloud2.read_points(msg, field_names=('x','y','z','rgb'), skip_nans=True)
            colors = points['rgb'].view(np.uint32) & 0x00ffffff
            tf = None
            for class_id in self.target_classes:
                selected = points[colors == self.class_rgb.get(class_id)]
                if len(selected) < 30:
                    continue
                xyz = np.column_stack([selected[name] for name in ('x','y','z')]).astype(float)
                if msg.header.frame_id != 'odom':
                    if tf is None:
                        try:
                            tf = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
                        except Exception:
                            tf = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time())
                            age = abs((tf.header.stamp.sec-msg.header.stamp.sec)+(tf.header.stamp.nanosec-msg.header.stamp.nanosec)*1e-9)
                            if age > .15:
                                return
                    q = tf.transform.rotation
                    x,y,z,w = q.x,q.y,q.z,q.w
                    rotation = np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)], [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)], [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
                    t = tf.transform.translation
                    xyz = xyz @ rotation.T + [t.x,t.y,t.z]
                floor = class_id in self.floor_classes
                xyz = xyz[(xyz[:,2] > (-.02 if floor else .65)) & (xyz[:,2] < (.2 if floor else 1.2))]
                if len(xyz) < 30:
                    continue
                lo, hi = np.quantile(xyz, [.01,.99], axis=0)
                self.object_boxes[class_id] = (time.monotonic(), ((lo+hi)/2).tolist(), (hi-lo+.04).tolist())
                if floor:
                    _, vectors = np.linalg.eigh(np.cov(xyz[:,:2].T))
                    self.floor_axes[class_id] = (time.monotonic(), vectors[:,-1])
        except Exception:
            # A just-published image can precede its TF sample; use the next frame.
            pass

    def prepare_payload(self, class_id, geometry):
        """Planning-only attachment, using depth bounds in the actual hand frame."""
        _, center, size = geometry
        world = PoseStamped()
        world.header.frame_id = 'odom'
        world.pose.orientation.w = 1.0
        world.pose.position.x,world.pose.position.y,world.pose.position.z = center
        local = self.tf_buffer.transform(world, 'fr3_hand')
        attached = AttachedCollisionObject()
        attached.link_name = 'fr3_hand'
        attached.touch_links = ['fr3_hand','fr3_leftfinger','fr3_rightfinger','camera_link','fr3_link7']
        attached.object.id = f'held_object_{class_id}'
        attached.object.header.frame_id = 'fr3_hand'
        attached.object.pose.orientation.w = 1.0
        attached.object.operation = CollisionObject.ADD
        attached.object.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size))
        attached.object.primitive_poses.append(local.pose)
        return attached

    def update_payload(self, attached, remove=False):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        obj = copy.deepcopy(attached)
        if remove:
            obj.object.operation = CollisionObject.REMOVE
        request.scene.robot_state.attached_collision_objects.append(obj)
        if remove:
            # MoveIt detachment inserts the last attached pose into the world.
            # This object is physically falling; perception will observe its new
            # location. Remove the obsolete hand-relative planning placeholder.
            world_remove = CollisionObject()
            world_remove.id = obj.object.id
            world_remove.operation = CollisionObject.REMOVE
            request.scene.world.collision_objects.append(world_remove)
        future = self.scene_client.call_async(request)
        deadline = time.monotonic()+5.0
        while not future.done() and time.monotonic()<deadline:
            time.sleep(.05)
        if not future.done() or not future.result().success:
            raise RuntimeError("Failed to update planning payload")
        self.get_logger().info(f"Planning payload {'removed' if remove else 'attached'}: {obj.object.id}")

    def truth_callback(self, msg):
        try:
            self.truth = json.loads(msg.data)
            self.truth_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def physical_snapshot(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.truth and time.monotonic() - self.truth_at < 2.0:
                return self.truth
            time.sleep(0.1)
        raise RuntimeError("No fresh physical verification telemetry; refusing unverified success")

    def status_callback(self, msg):
        """机械臂状态回调"""
        self.last_motion_status = msg.data
    
    def detections_callback(self, msg):
        """检测结果回调 - 实时更新检测结果"""
        with self.detection_lock:
            self.latest_detections = msg.detections
            self.detection_received_at = time.monotonic()
            self.get_logger().debug(f"Received {len(msg.detections)} detections")
       
    def get_latest_detection(self, class_id: int, timeout: float = None) -> Detection3D:
        """获取指定类别的最新检测结果"""
        if timeout is None:
            timeout = self.detection_timeout
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.detection_lock:
                # 查找匹配的检测结果，只使用近期收到的数据。
                detections = self.latest_detections if time.monotonic() - self.detection_received_at <= self.detection_timeout else []
                for det in detections:
                    if not det.results:
                        continue
                    
                    # 检查类别ID和置信度
                    det_class_id = int(det.results[0].hypothesis.class_id)
                    confidence = det.results[0].hypothesis.score
                    
                    if det_class_id == class_id and confidence >= self.min_confidence:
                        class_name = self.class_id_to_name.get(class_id, f"unknown({class_id})")
                        self.get_logger().info(
                            f"Found target '{class_name}' (id={class_id}) with confidence {confidence:.3f} "
                            f"at ({det.bbox.center.position.x:.3f}, "
                            f"{det.bbox.center.position.y:.3f}, "
                            f"{det.bbox.center.position.z:.3f})"
                        )
                        return det
            
            time.sleep(0.1)  # 等待新检测结果
        
        return None

    def get_stable_detection(self, class_id: int, collection_time: float = 2.0):
        """
        Collect detections and use Density-Based Clustering (Majority Vote) to find the most robust pose.
        Rejects outliers and handles multi-modal noise.
        """
        self.get_logger().info(f"Collecting detections for {collection_time}s (Clustering)...")
        candidates = []
        seen = set()
        
        start_time = time.time()
        while time.time() - start_time < collection_time:
            det = self.get_latest_detection(class_id, timeout=0.1)
            if det and det.results:
                stamp = (det.header.stamp.sec, det.header.stamp.nanosec)
                if stamp in seen:
                    time.sleep(0.1)
                    continue
                seen.add(stamp)
                pose = det.results[0].pose.pose
                candidates.append({
                    'det': det,
                    'p': [pose.position.x, pose.position.y, pose.position.z],
                    'q': [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                })
            time.sleep(0.1)
            
        n = len(candidates)
        if n == 0:
            self.get_logger().warn(f"No candidates collected for class {class_id}")
            return None
        self.get_logger().info(f"Collected {n} candidates.")
        
        # Parameters for clustering
        THRESH_POS = 0.05  # 5cm
        THRESH_ROT = 0.1   # Approx 25 degrees (1 - |dot|)
        
        # 1. Compute Neighbors (Density)
        # For each point, count how many neighbors it has within the threshold
        max_neighbors = -1
        best_seed_idx = -1
        
        # Pre-compute neighbor lists to avoid O(N^3) later, though N is small (~20)
        neighbors = [[] for _ in range(n)]
        
        for i in range(n):
            for j in range(n):
                if i == j:
                    neighbors[i].append(j) # Include self
                    continue
                
                # Pos Dist
                p1, p2 = candidates[i]['p'], candidates[j]['p']
                d_pos = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)
                
                # Rot Dist (1 - |dot|)
                q1, q2 = candidates[i]['q'], candidates[j]['q']
                dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]
                d_rot = 1.0 - abs(dot)
                
                if d_pos < THRESH_POS and d_rot < THRESH_ROT:
                    neighbors[i].append(j)
                    
            if len(neighbors[i]) > max_neighbors:
                max_neighbors = len(neighbors[i])
                best_seed_idx = i
                
        # 2. Identify Dominant Cluster
        # The cluster is defined by the neighbors of the best seed
        cluster_indices = neighbors[best_seed_idx]
        cluster_size = len(cluster_indices)
        self.get_logger().info(f"Dominant cluster size: {cluster_size}/{n} (Seed matches {cluster_size-1} others)")
        
        # 3. Compute Mean of Cluster (Refined Statistics)
        sum_x, sum_y, sum_z = 0.0, 0.0, 0.0
        
        # Ref Quaternion Logic
        ref_q = candidates[best_seed_idx]['q']
        acc_q = [0.0, 0.0, 0.0, 0.0]
        
        for idx in cluster_indices:
            c = candidates[idx]
            # Pos
            sum_x += c['p'][0]
            sum_y += c['p'][1]
            sum_z += c['p'][2]
            
            # Rot
            q = c['q']
            dot = ref_q[0]*q[0] + ref_q[1]*q[1] + ref_q[2]*q[2] + ref_q[3]*q[3]
            sign = 1.0 if dot >= 0 else -1.0
            acc_q[0] += sign * q[0]
            acc_q[1] += sign * q[1]
            acc_q[2] += sign * q[2]
            acc_q[3] += sign * q[3]
            
        mean_p = [sum_x/cluster_size, sum_y/cluster_size, sum_z/cluster_size]
        
        norm = math.sqrt(sum(x*x for x in acc_q))
        mean_q = [x/norm for x in acc_q]
        
        self.get_logger().info(f"Cluster Mean Pos: ({mean_p[0]:.3f}, {mean_p[1]:.3f}, {mean_p[2]:.3f})")
        
        # 4. Select Best Candidate from Cluster
        best_det = None
        min_score = float('inf')
        
        w_pos = 1.0
        w_rot = 0.5
        
        for idx in cluster_indices:
            c = candidates[idx]
            
            # Distance to Cluster Mean
            dp = c['p']
            dist_p = math.sqrt((dp[0]-mean_p[0])**2 + (dp[1]-mean_p[1])**2 + (dp[2]-mean_p[2])**2)
            
            q = c['q']
            dot = mean_q[0]*q[0] + mean_q[1]*q[1] + mean_q[2]*q[2] + mean_q[3]*q[3]
            dist_q = 1.0 - abs(dot)
            
            score = w_pos * dist_p + w_rot * dist_q
            if score < min_score:
                min_score = score
                best_det = c['det']
                
        if best_det:
             p = best_det.results[0].pose.pose.position
             self.get_logger().info(f"Selected best detection from cluster. Score: {min_score:.4f}. Pos: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")

        return best_det
    
    # Orientation helpers removed to strictly enforce GraspNet/Pose usage
    
    def send_arm_pose(self, x: float, y: float, z: float, orientation: Quaternion, timeout: float = 45.0, cartesian=False) -> bool:
        """发送机械臂位姿命令"""
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = orientation
        
        self.get_logger().info(f"Moving arm to: Pos({x:.3f}, {y:.3f}, {z:.3f}) Rot(xyzw)({orientation.x:.3f}, {orientation.y:.3f}, {orientation.z:.3f}, {orientation.w:.3f})")
        
        self.last_motion_status = None
        (self.pub_cartesian if cartesian else self.pub_arm).publish(pose)
        
        # Publish Visualization Marker
        marker = Marker()
        marker.header = pose.header
        marker.ns = "target_poses"
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.1  # Arrow length
        marker.scale.y = 0.01 # Arrow width
        marker.scale.z = 0.01 # Arrow height
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.pub_vis.publish(marker)
        
        # 等待执行结果（带超时）
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.last_motion_status:
                if "SUCCESS" in self.last_motion_status:
                    self.get_logger().info("Arm motion SUCCESS")
                    return True
                else:
                    self.get_logger().error(f"Arm motion failed: {self.last_motion_status}")
                    return False
            time.sleep(0.1)
        
        self.get_logger().error(f"Arm motion timeout after {timeout}s")
        return False
    
    def control_gripper(self, position: float, timeout: float = 20.0) -> bool:
        return execute_gripper(self, self.gripper_client, position, timeout)

    
    def control_inference(self, enable: bool) -> bool:
        """Gate YOLOE clouds and GraspNet while keeping YOLOE and depth inference active."""
        success = True
        request = SetBool.Request()
        request.data = enable
        
        if enable:
            with self.detection_lock:
                self.latest_detections = []
                self.detection_received_at = 0.0
        action = "Enabling" if enable else "Disabling"
        self.get_logger().info(f">>> {action} YOLOE point clouds and GraspNet inference...")
        
        # YOLOE keeps detecting and rendering; only its cloud outputs are gated.
        if self.yoloe_cloud_client.wait_for_service(timeout_sec=1.0):
            try:
                future = self.yoloe_cloud_client.call_async(request)
                while not future.done():
                    time.sleep(0.05)
                response = future.result()
                if response.success:
                    self.get_logger().info(f"YOLOE: {response.message}")
                else:
                    self.get_logger().warn(f"YOLOE control failed: {response.message}")
                    success = False
            except Exception as e:
                self.get_logger().error(f"YOLOE control error: {e}")
                success = False
        else:
            self.get_logger().warn("YOLOE point cloud control service not available")
        
        # 控制 GraspNet
        if self.graspnet_inference_client.wait_for_service(timeout_sec=1.0):
            try:
                future = self.graspnet_inference_client.call_async(request)
                while not future.done():
                    time.sleep(0.05)
                response = future.result()
                if response.success:
                    self.get_logger().info(f"GraspNet: {response.message}")
                else:
                    self.get_logger().warn(f"GraspNet control failed: {response.message}")
                    success = False
            except Exception as e:
                self.get_logger().error(f"GraspNet control error: {e}")
                success = False
        else:
            self.get_logger().warn("GraspNet inference control service not available")
        
        # Geometry must keep flowing for occupancy hit/miss updates, including
        # when LSM supplies depth instead of the simulator.
        if self.depth_inference_client.wait_for_service(timeout_sec=1.0):
            try:
                depth_request = SetBool.Request()
                depth_request.data = True
                future = self.depth_inference_client.call_async(depth_request)
                while not future.done():
                    time.sleep(0.05)
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Depth: {response.message}")
                else:
                    self.get_logger().warn(f"Depth control failed: {response.message}")
                    success = False
            except Exception as e:
                self.get_logger().error(f"Depth control error: {e}")
                success = False
        else:
            self.get_logger().warn("Depth inference control service not available")
        
        return success
    

    
    def perform_pick_and_place(self, pose, class_id: int) -> bool:
        """执行抓取和放置序列 - Using GraspNet Pose"""
        self.get_logger().info("=== Starting Pick and Place Sequence ===")
        initial_truth = self.physical_snapshot() if self.verify_physical else None
        object_name = CLASS_OBJECTS[class_id]
        geometry = self.object_boxes.get(class_id)
        if geometry is None or time.monotonic()-geometry[0] > self.detection_timeout:
            self.get_logger().error("No fresh segmented depth bounds for planning payload")
            return False
        target_box = CollisionObject()
        target_box.header.frame_id = 'odom'
        target_box.id = f'pick_target_bounds_{class_id}'
        target_box.pose.orientation.w = 1.0
        target_box.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=geometry[2]))
        bounds_pose = Pose()
        bounds_pose.orientation.w = 1.0
        bounds_pose.position.x, bounds_pose.position.y, bounds_pose.position.z = geometry[1]
        target_box.primitive_poses.append(bounds_pose)
        self.pub_target_bounds.publish(target_box)
        payload = None
        payload_installed = False
        
        # 暂停 YOLOE 点云和 GraspNet；YOLOE 检测与画面继续输出
        self.control_inference(False)
        
        # 设置点云过滤，避免与目标物体碰撞
        self.get_logger().info(f"Filtering cloud for class_id={class_id}")
        self.pub_cloud_filter.publish(Int32(data=class_id))
        self.remove_collision_object(class_id)
        time.sleep(3.0)  # Allow a complete 1 Hz planning-scene export, even below real time.

        try:
            if not self.control_gripper(0.06):
                self.get_logger().error("Failed to open gripper before grasp")
                return False
            
            q = pose.orientation
            approach = [1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y+q.w*q.z), 2*(q.x*q.z-q.w*q.y)]
            if not self.send_arm_pose(pose.position.x - .08*approach[0], pose.position.y - .08*approach[1], pose.position.z - .08*approach[2], q):
                self.get_logger().error("Failed to reach pregrasp pose")
                return False
            # 移动到目标位置
            self.get_logger().info("Step 1: Moving to target position...")
            # Use Pose from GraspNet
            if not self.send_arm_pose(pose.position.x, pose.position.y, pose.position.z, pose.orientation, cartesian=True):
                self.get_logger().error("Failed to move to target position")
                return False
                        
            # 关闭夹爪
            self.get_logger().info("Step 2: Closing gripper...")
            gripper_result = self.control_gripper(0.005)
            if not gripper_result:
                self.get_logger().error("Failed to close gripper")
                return False            
            time.sleep(0.5)
            
            # 移除CollisionObject - 物体已抓取
            self.get_logger().info("Removing collision object from octomap...")
            self.remove_collision_object(class_id)
            
            # Cache the measured box relative to the gripping hand before lift.
            payload = self.prepare_payload(class_id, geometry)
            measured_center = PoseStamped()
            measured_center.header.frame_id = 'odom'
            measured_center.pose.orientation.w = 1.0
            measured_center.pose.position.x, measured_center.pose.position.y, measured_center.pose.position.z = geometry[1]
            center_in_tcp = self.tf_buffer.transform(measured_center, 'fingers_center').pose.position
            # Clear the support surface vertically before retracting sideways.
            if not self.send_arm_pose(pose.position.x, pose.position.y, pose.position.z + .15, pose.orientation, cartesian=True):
                self.get_logger().error("Failed vertical lift")
                return False
            if self.verify_physical and not lifted(initial_truth, self.physical_snapshot(), object_name):
                self.get_logger().error("Physical vertical lift failed")
                return False
            # Install after clearing the support, so padding never creates a
            # spurious initial penetration into the table/floor.
            self.update_payload(payload)
            payload_installed = True
            time.sleep(1.5) # Allow mapper to replace world voxels inside the attached body.
            # 抬起:XY方向往base回收80%距离 + Z轴固定1m高度
            self.get_logger().info("Step 3: Lifting object...")
            try:
                # 获取base_footprint在odom下的位置
                base_transform = self.tf_buffer.lookup_transform(
                    'odom', 'base_footprint', rclpy.time.Time())
                base_x = base_transform.transform.translation.x
                base_y = base_transform.transform.translation.y
                
                # 计算从当前位置到base_footprint的XY方向,回收80%距离
                dx = base_x - pose.position.x
                dy = base_y - pose.position.y
                
                # XY方向回收80%
                retreat_x = pose.position.x + dx * 0.8
                retreat_y = pose.position.y + dy * 0.8
                
                # Preserve the achieved vertical clearance during retraction.
                # Lowering a high table grasp back to 1 m loses its lift margin.
                retreat_z = max(1.0, pose.position.z + .15)
                
                self.get_logger().info(f"Retreating 80% towards base without lowering payload: ({retreat_x:.3f}, {retreat_y:.3f}, {retreat_z:.3f})")
                
                if not self.send_arm_pose(retreat_x, retreat_y, retreat_z, pose.orientation):
                    self.get_logger().error("Failed to lift object")
                    return False
            except Exception as e:
                self.get_logger().error(f"Failed to get base_footprint transform: {e}")
                # Keep the same lift clearance if base TF is unavailable.
                if not self.send_arm_pose(pose.position.x, pose.position.y, max(1.0, pose.position.z + .15), pose.orientation):
                    self.get_logger().error("Failed to lift object (fallback)")
                    return False
            
            lift_truth = self.physical_snapshot() if self.verify_physical else None
            if self.verify_physical and not lifted(initial_truth, lift_truth, object_name):
                self.get_logger().error("Physical grasp failed: object did not lift with the hand")
                return False
            self.get_logger().info("Physical lift verified")

            # 移动到放置位置
            self.get_logger().info("Step 4: Moving to place position...")
            
            p = self.POSES['Place']
            # Preserve the object's tilt and only yaw toward the bin; rolling the
            # hand into the old fixed place pose tipped objects onto the rim.
            yaw = math.atan2(p['pos'][1]-base_y, p['pos'][0]-base_x) - math.atan2(approach[1], approach[0])
            if class_id in self.get_parameter('elongated_place_classes').value:
                # Long objects fit across the square bin's diagonal, rather
                # than bridging its opposite parallel rims.
                yaw += self.get_parameter('elongated_place_yaw').value
            a, b = math.sin(yaw/2), math.cos(yaw/2)
            g = pose.orientation
            q = Quaternion(x=b*g.x-a*g.y, y=a*g.x+b*g.y, z=b*g.z+a*g.w, w=b*g.w-a*g.z)
            # Center the perceived payload over the bin, rather than centering
            # only the TCP (off-center grasps otherwise land near the rim).
            offset = Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([center_in_tcp.x,center_in_tcp.y,center_in_tcp.z])
            place_x, place_y = p['pos'][0]-offset[0], p['pos'][1]-offset[1]
            if not self.send_arm_pose(place_x, place_y, p['pos'][2], q):
                self.get_logger().error("Failed to move to place position")
                return False
            
            time.sleep(1.5)  # Let the loaded arm settle before releasing.
            if self.verify_physical and not retained(lift_truth, self.physical_snapshot(), object_name):
                self.get_logger().error("Physical transport failed: object slipped from gripper")
                return False

            # 打开夹爪（放置）
            self.get_logger().info("Step 5: Opening gripper to place...")
            # A single full stroke can sweep the mug handle sideways. First
            # release pressure in small increments and let the object drop clear.
            opening = getattr(self, 'last_gripper_position', .005)
            step = self.get_parameter('release_step').value
            pause = self.get_parameter('release_pause').value
            for increment in range(1, 5):
                if not self.control_gripper(min(.06, opening + step*increment)):
                    self.get_logger().error("Failed gradual gripper release")
                    return False
                time.sleep(pause)
            if not self.control_gripper(0.06):
                self.get_logger().error("Failed to open gripper")
                return False
            time.sleep(0.5)
            self.update_payload(payload, remove=True)
            payload_installed = False
            if self.verify_physical:
                deadline = time.monotonic() + 15.0
                settled = False
                inside_since = None
                while time.monotonic() < deadline:
                    state = self.physical_snapshot()
                    if in_bin(state, object_name):
                        if inside_since is None:
                            inside_since = state["simulation_time"]
                        if state["simulation_time"] - inside_since >= 1.5:
                            settled = True
                            break
                    else:
                        inside_since = None
                    time.sleep(0.2)
                if not settled:
                    self.get_logger().error("Physical placement failed: object is outside the bin")
                    return False
                self.get_logger().info(f"Physical placement verified: {object_name} inside bin")

            # 返回初始位置
            if not self.perform_go_home():
                self.get_logger().error("Failed to return home after placement")
                return False
            
            self.get_logger().info("=== Pick and Place Sequence Complete! ===")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Pick and place failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
        finally:
            self.control_gripper(0.06)
            if payload_installed:
                self.update_payload(payload, remove=True)
            target_box.operation = CollisionObject.REMOVE
            self.pub_target_bounds.publish(target_box)
            # 清除过滤
            self.get_logger().info("Clearing cloud filter")
            self.pub_cloud_filter.publish(Int32(data=-1))
            # 确保移除collision object
            self.remove_collision_object(class_id)
            # 恢复 YOLOE 点云发布和 GraspNet 推理
            self.control_inference(True)
    
    def remove_collision_object(self, class_id: int):
        """移除CollisionObject从规划场景"""
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = 'odom'
        collision_obj.header.stamp = self.get_clock().now().to_msg()
        collision_obj.id = f'target_object_{class_id}'
        collision_obj.operation = CollisionObject.REMOVE
        
        self.pub_collision_object.publish(collision_obj)
        self.get_logger().info(f"Sent REMOVE operation for collision object: {collision_obj.id}")
        time.sleep(0.5)  # 等待移除生效
    
    def run_logic_loop(self):
        """主逻辑循环"""
        round_number = 0
        first_round = True
        
        # 等待节点初始化
        time.sleep(2.0)
        try:
            self.install_fixture_collisions()
        except Exception as error:
            self.get_logger().error(f"Fixture setup failed: {error}")
            return
        while rclpy.ok():
            if set(self.target_classes) <= self.completed_classes:
                self.get_logger().info("All objects physically picked and placed; demo complete")
                return
            round_number += 1
            
            self.get_logger().info(f"\n{'#'*60}")
            self.get_logger().info(f"### Round {round_number} ###")
            self.get_logger().info(f"{'#'*60}")
            
            self.inspect_floor = False
            # Phase 1: Table Objects
            self.get_logger().info("\n>>> PHASE 1: Table Objects")
            if not self.perform_go_home():
                self.get_logger().warn("Home failed; retrying before scan and grasp")
                time.sleep(2.0)
                continue
            time.sleep(2.0)
            if not self.perform_scan():
                self.get_logger().warn("Scan incomplete; retrying before grasp")
                time.sleep(2.0)
                continue
            time.sleep(1.0)
            
            self.process_targets(self.table_classes, "Table")
            
            self.inspect_floor = True
            # Phase 2: Floor Objects
            self.get_logger().info("\n>>> PHASE 2: Floor Objects")
            
            p = self.POSES['PickFloor']
            q = Quaternion(x=p['rot'][0], y=p['rot'][1], z=p['rot'][2], w=p['rot'][3])
            if not self.send_arm_pose(p['pos'][0], p['pos'][1], p['pos'][2], q):
                self.get_logger().warn("Floor observation pose failed; retrying from home")
                continue
            time.sleep(3.0)
            
            self.process_targets(self.floor_classes, "Floor")
            
            self.get_logger().info(f"\n--- Round {round_number} complete ---")
            time.sleep(1.0)

    def process_targets(self, class_ids, zone_name):
        """处理指定列表的目标"""
        picked_count = 0
        for class_id in class_ids:
            if not rclpy.ok():
                break
                
            if class_id in self.completed_classes:
                continue
            class_name = self.class_id_to_name.get(class_id, f"unknown({class_id})")
            self.get_logger().info(f"\n--- [{zone_name}] Looking for '{class_name}' (id={class_id}) ---")
            
            if zone_name == "Floor":
                p = self.POSES['PickFloor']
                q = Quaternion(x=p['rot'][0], y=p['rot'][1], z=p['rot'][2], w=p['rot'][3])
                if not self.send_arm_pose(*p['pos'], q):
                    self.get_logger().warn("Cannot observe floor target from current pose")
                    continue
                with self.detection_lock:
                    self.latest_detections = []
                    self.detection_received_at = 0.0
                time.sleep(3.0)
            # 获取检测结果 (Stable Strategy)
            detection = self.get_stable_detection(class_id, collection_time=5.0)

            if detection is None and zone_name == "Floor":
                p = self.POSES['PickFloor']
                q = Quaternion(x=p['rot'][0], y=p['rot'][1], z=p['rot'][2], w=p['rot'][3])
                for offset in (.12, -.12):
                    self.get_logger().info(f"Searching floor from camera offset {offset:+.2f} m")
                    if self.send_arm_pose(p['pos'][0]+offset, p['pos'][1], p['pos'][2], q):
                        time.sleep(2.0)
                        detection = self.get_stable_detection(class_id, collection_time=5.0)
                        if detection is not None:
                            break
            if detection is None:
                self.get_logger().warn(f"No detection found for '{class_name}'")
                continue
            
            # 提取坐标
            pos = detection.bbox.center.position
            x, y, z = pos.x, pos.y, pos.z
            
            self.get_logger().info(f"Target '{class_name}' detected at: ({x:.3f}, {y:.3f}, {z:.3f})")
            
            # Extract Pose from detection (GraspNet Result)
            if not detection.results:
                 self.get_logger().warn("Detection has no results/pose available")
                 continue
            
            grasp_pose = copy.deepcopy(detection.results[0].pose.pose)
            q = grasp_pose.orientation
            if zone_name == "Table" and class_id in self.get_parameter('thin_object_classes').value:
                # Keep the predicted yaw/roll, elevate the wrist to clear the
                # table with link6 while reaching small upright objects.
                roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                pitch = self.get_parameter('thin_object_approach_pitch').value
                cr,sr,cp,sp,cy,sy = math.cos(roll/2),math.sin(roll/2),math.cos(pitch/2),math.sin(pitch/2),math.cos(yaw/2),math.sin(yaw/2)
                q = Quaternion(x=sr*cp*cy-cr*sp*sy, y=cr*sp*cy+sr*cp*sy, z=cr*cp*sy-sr*sp*cy, w=cr*cp*cy+sr*sp*sy)
                grasp_pose.orientation = q
            if zone_name == "Floor" and class_id in self.get_parameter('top_grasp_classes').value and class_id in self.floor_axes:
                observed_at, long_axis = self.floor_axes[class_id]
                if time.monotonic() - observed_at <= self.detection_timeout:
                    geometry = self.object_boxes.get(class_id)
                    if geometry is None or time.monotonic() - geometry[0] > self.detection_timeout:
                        self.get_logger().warn("No fresh depth center for top grasp")
                        continue
                    # A PCA top approach requires its corresponding depth center;
                    # the original GraspNet position belongs to a different pose.
                    grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z = geometry[1]
                    base = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time()).transform.translation
                    radial = np.array([grasp_pose.position.x-base.x, grasp_pose.position.y-base.y])
                    if long_axis @ radial < 0:
                        long_axis = -long_axis
                    # Approach mostly from above; close across the narrow side.
                    # This keeps the wrist nearer the shoulder within arm reach.
                    ax = np.array([.2*long_axis[0], .2*long_axis[1], -math.sqrt(.96)])
                    ay = np.array([-long_axis[1],long_axis[0],0.])
                    az = np.cross(ax,ay)
                    # Convert orthonormal columns to quaternion via tf utilities.
                    xyzw = Rotation.from_matrix(np.column_stack([ax,ay,az])).as_quat()
                    q = Quaternion(x=float(xyzw[0]),y=float(xyzw[1]),z=float(xyzw[2]),w=float(xyzw[3]))
                    grasp_pose.orientation = q
                    self.get_logger().info(f"Floor top grasp from depth PCA axis: {long_axis.tolist()}")
            approach = [1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y+q.w*q.z), 2*(q.x*q.z-q.w*q.y)]
            # Extended fingertips project 45.5 mm beyond this GraspNet frame.
            # Lift the TCP for ground contacts, and avoid over-inserting into
            # narrow objects while keeping their surface inside the pads.
            backoff = self.get_parameter('floor_tcp_backoff').value if zone_name == "Floor" else (self.get_parameter('thin_object_tcp_backoff').value if class_id in self.get_parameter('thin_object_classes').value else 0.0)
            grasp_pose.position.x -= backoff*approach[0]
            grasp_pose.position.y -= backoff*approach[1]
            grasp_pose.position.z -= backoff*approach[2]
            self.get_logger().info(f"TCP backoff={backoff:.3f} m")
            
            # 执行抓取和放置
            if self.perform_pick_and_place(grasp_pose, class_id):
                picked_count += 1
                self.completed_classes.add(class_id)
                self.get_logger().info(f"Successfully picked '{class_name}'!")
                # Pick 成功后返回Home
                self.perform_go_home()
            else:
                self.get_logger().error(f"Failed to pick '{class_name}'")
                self.perform_go_home()
        
        return picked_count
        
        self.get_logger().info("\n" + "#"*60)
        self.get_logger().info("### All picking complete! ###")
        self.get_logger().info("#"*60)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
