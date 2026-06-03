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
from moveit_msgs.msg import CollisionObject
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand, FollowJointTrajectory
from builtin_interfaces.msg import Duration
import math
import time
import threading
import yaml
import os
from collections import deque

class PickAndPlaceDemo(Node):
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
        self.pub_vis = self.create_publisher(Marker, '/target_pose_marker', 10)
        self.marker_id = 0
        self.gripper_client = ActionClient(self, GripperCommand, '/fr3_gripper_controller/gripper_cmd')
        
        # 关节控制
        self.arm_joint_client = ActionClient(self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')
        self.arm_joint_names = [f'fr3_joint{i}' for i in range(1, 8)]

        # 点云过滤控制
        self.pub_cloud_filter = self.create_publisher(Int32, 'set_cloud_filter', 10)
        
        # CollisionObject发布器用于清除octomap
        self.pub_collision_object = self.create_publisher(CollisionObject, 'collision_object', 10)
   
        # 机器人动作服务
        self.go_home_client = self.create_client(Trigger, '/robot_actions/go_home')
        self.scan_client = self.create_client(Trigger, '/robot_actions/scan')
        
        # 推理控制服务客户端
        self.yoloe_inference_client = self.create_client(SetBool, '/yoloe_multi_text_prompt/enable_inference')
        self.graspnet_inference_client = self.create_client(SetBool, '/graspnet_node/enable_inference')
        self.depth_inference_client = self.create_client(SetBool, '/stereo_matching_node/enable_inference')
        
        # 状态订阅
        self.create_subscription(String, '/arm_command/status', self.status_callback, 10)
        
        # 配置参数
        self.declare_parameter('table_classes', [3, 2, 9])  # book, cup, coke
        self.declare_parameter('floor_classes', [4, 6])     # bottle, shoe
        self.declare_parameter('min_confidence', 0.25)
        self.declare_parameter('detection_timeout', 5.0)
        
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
                'pos': [-0.02, -0.76, 0.61],
                'rot': [0.29, 0.28, -0.66, 0.64]
            },
            # 'palce':{
            #     'pos': [0.66, -0.03, -0.02],
            #     'rot': [-0.01, 0.31, 0.02, 0.95]
            # },
        }
        
        # 从YOLOE配置文件读取类别ID到名称的映射
        self.class_id_to_name = self.load_class_mapping_from_config()
        
        # 状态变量
        self.latest_detections = []  # 最新的检测结果
        self.detection_lock = threading.Lock()
        self.last_motion_status = None
        
        # 启动逻辑线程
        self.logic_thread = threading.Thread(target=self.run_logic_loop, daemon=True)
        self.logic_thread.start()
        
        self.get_logger().info("=== Pick and Place Demo - Real-time YOLOE Edition ===")
        target_names = [self.class_id_to_name.get(cid, f"unknown({cid})") for cid in self.target_classes]
        self.get_logger().info(f"Target classes: {self.target_classes} ({', '.join(target_names)})")
        self.get_logger().info(f"Min confidence: {self.min_confidence}")
    
    def load_class_mapping_from_config(self):
        """从YOLOE配置文件中加载类别ID到名称的映射"""
        try:
            from ament_index_python.packages import get_package_share_directory
            yoloe_pkg_dir = get_package_share_directory('yoloe_infer')
            config_path = os.path.join(yoloe_pkg_dir, 'configs', 'config.yaml')
        except:
            config_path = "src/yoloe_infer/configs/config.yaml"
        
        # 如果文件不存在，尝试使用包路径
        if not os.path.exists(config_path):
            try:
                from ament_index_python.packages import get_package_share_directory
                yoloe_pkg_dir = get_package_share_directory('yoloe_infer')
                config_path = os.path.join(yoloe_pkg_dir, 'configs', 'config.yaml')
            except:
                pass
        
        class_id_to_name = {}
        
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    
                if 'classes' in config:
                    for cls in config['classes']:
                        if 'id' in cls and 'name' in cls:
                            class_id_to_name[cls['id']] = cls['name']
                    
                    self.get_logger().info(f"Loaded {len(class_id_to_name)} class mappings from {config_path}")
                else:
                    self.get_logger().warn(f"No 'classes' section found in {config_path}")
            else:
                self.get_logger().warn(f"Config file not found: {config_path}, using empty mapping")
        except Exception as e:
            self.get_logger().error(f"Failed to load class mapping from {config_path}: {e}")
        
        return class_id_to_name

    def status_callback(self, msg):
        """机械臂状态回调"""
        self.last_motion_status = msg.data
    
    def detections_callback(self, msg):
        """检测结果回调 - 实时更新检测结果"""
        with self.detection_lock:
            self.latest_detections = msg.detections
            self.get_logger().debug(f"Received {len(msg.detections)} detections")
       
    def get_latest_detection(self, class_id: int, timeout: float = None) -> Detection3D:
        """获取指定类别的最新检测结果"""
        if timeout is None:
            timeout = self.detection_timeout
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.detection_lock:
                # 查找匹配的检测结果
                for det in self.latest_detections:
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
        
        start_time = time.time()
        while time.time() - start_time < collection_time:
            det = self.get_latest_detection(class_id, timeout=0.1)
            if det and det.results:
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
    
    def send_arm_pose(self, x: float, y: float, z: float, orientation: Quaternion, timeout: float = 30.0) -> bool:
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
        self.pub_arm.publish(pose)
        
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
    
    def control_gripper(self, position: float, timeout: float = 10.0) -> bool:
        """控制夹爪"""
        self.get_logger().info(f"Setting gripper to {position}")
        
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 10.0
        
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Gripper server not available")
            return False
        
        send_goal_future = self.gripper_client.send_goal_async(goal)
        start_time = time.time()
        while not send_goal_future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error("Gripper send goal timeout")
                return False
            time.sleep(0.1)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False
        
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error("Gripper result timeout")
                return False
            time.sleep(0.1)
        
        result_future.result()
        self.get_logger().info(f"Gripper set to {position}")
        time.sleep(0.5)
        return True
    
    def perform_scan(self) -> bool:
        """执行扫描"""
        self.get_logger().info(">>> Triggering Scan Sequence...")
        
        if not self.scan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Scan service not available")
            return False
        
        req = Trigger.Request()
        future = self.scan_client.call_async(req)
        while not future.done():
            time.sleep(0.1)
        
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Scan Success: {res.message}")
                return True
            else:
                self.get_logger().error(f"Scan Failed: {res.message}")
                return False
        except Exception as e:
            self.get_logger().error(f"Scan call failed: {e}")
            return False
    
    def control_inference(self, enable: bool) -> bool:
        """控制所有推理节点的启用/禁用状态"""
        success = True
        request = SetBool.Request()
        request.data = enable
        
        action = "Enabling" if enable else "Disabling"
        self.get_logger().info(f">>> {action} inference nodes...")
        
        # 控制 YOLOE
        if self.yoloe_inference_client.wait_for_service(timeout_sec=1.0):
            try:
                future = self.yoloe_inference_client.call_async(request)
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
            self.get_logger().warn("YOLOE inference control service not available")
        
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
        
        # 控制深度推理
        if self.depth_inference_client.wait_for_service(timeout_sec=1.0):
            try:
                future = self.depth_inference_client.call_async(request)
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
    
    def perform_go_home(self) -> bool:
        """返回初始位置 (使用 Robot Actions Service)"""
        self.get_logger().info(">>> Triggering Go Home...")
        
        if not self.go_home_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Go Home service not available")
            return False
            
        req = Trigger.Request()
        future = self.go_home_client.call_async(req)
        while not future.done():
            time.sleep(0.1)
            
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Go Home Success: {res.message}")
                return True
            else:
                self.get_logger().error(f"Go Home Failed: {res.message}")
                return False
        except Exception as e:
            self.get_logger().error(f"Go Home call failed: {e}")
            return False

    def move_to_joints(self, positions: list) -> bool:
        """移动到指定关节位置"""
        self.get_logger().info(f"Moving to joints: {positions}")
        
        if not self.arm_joint_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Arm joint action server not available")
            return False
            
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=3, nanosec=0)
        goal.trajectory.points.append(point)
        
        future = self.arm_joint_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Joint goal rejected")
            return False
            
        res_future = goal_handle.get_result_async()
        while not res_future.done():
            time.sleep(0.1)
            
        res = res_future.result()
        if res.result.error_code == 0:
            self.get_logger().info("Joint move SUCCESS")
            return True
        else:
            self.get_logger().error(f"Joint move failed with error code: {res.result.error_code}")
            return False
    
    def perform_pick_and_place(self, pose, class_id: int) -> bool:
        """执行抓取和放置序列 - Using GraspNet Pose"""
        self.get_logger().info("=== Starting Pick and Place Sequence ===")
        
        # 禁用推理节点，节省计算资源
        self.control_inference(False)
        
        # 设置点云过滤，避免与目标物体碰撞
        self.get_logger().info(f"Filtering cloud for class_id={class_id}")
        self.pub_cloud_filter.publish(Int32(data=class_id))
        time.sleep(1)  # 等待过滤生效和地图更新

        try:
            self.control_gripper(0.06)
            
            # 移动到目标位置
            self.get_logger().info("Step 1: Moving to target position...")
            # Use Pose from GraspNet
            if not self.send_arm_pose(pose.position.x, pose.position.y, pose.position.z, pose.orientation):
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
            
            # 抬起:XY方向往base回收50%距离 + Z轴固定1m高度
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
                
                # Z轴固定为1m
                retreat_z = 1.0
                
                self.get_logger().info(f"Retreating 50% towards base at Z=1m: ({retreat_x:.3f}, {retreat_y:.3f}, {retreat_z:.3f})")
                
                if not self.send_arm_pose(retreat_x, retreat_y, retreat_z, pose.orientation):
                    self.get_logger().error("Failed to lift object")
                    return False
            except Exception as e:
                self.get_logger().error(f"Failed to get base_footprint transform: {e}")
                # Fallback: Z轴固定1m
                if not self.send_arm_pose(pose.position.x, pose.position.y, 1.0, pose.orientation):
                    self.get_logger().error("Failed to lift object (fallback)")
                    return False
            
            # 移动到放置位置
            self.get_logger().info("Step 4: Moving to place position...")
            
            p = self.POSES['Place']
            q = Quaternion(x=p['rot'][0], y=p['rot'][1], z=p['rot'][2], w=p['rot'][3])
            
            if not self.send_arm_pose(p['pos'][0], p['pos'][1], p['pos'][2], q): # Move to place area with object held firmly
                self.get_logger().error("Failed to move to place position")
                return False
            
            # 打开夹爪（放置）
            self.get_logger().info("Step 5: Opening gripper to place...")
            if not self.control_gripper(0.06):
                self.get_logger().error("Failed to open gripper")
                return False
            time.sleep(0.5)
            
            # 返回初始位置
            self.perform_go_home()
            
            self.get_logger().info("=== Pick and Place Sequence Complete! ===")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Pick and place failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
        finally:
            # 清除过滤
            self.get_logger().info("Clearing cloud filter")
            self.pub_cloud_filter.publish(Int32(data=-1))
            # 确保移除collision object
            self.remove_collision_object(class_id)
            # 重新启用推理节点
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
        
        while rclpy.ok():
            round_number += 1
            
            self.get_logger().info(f"\n{'#'*60}")
            self.get_logger().info(f"### Round {round_number} ###")
            self.get_logger().info(f"{'#'*60}")
            
            # Phase 1: Table Objects
            self.get_logger().info("\n>>> PHASE 1: Table Objects")
            self.perform_go_home()
            time.sleep(2.0)
            self.perform_scan()
            time.sleep(1.0)
            
            self.process_targets(self.table_classes, "Table")
            
            # Phase 2: Floor Objects
            self.get_logger().info("\n>>> PHASE 2: Floor Objects")
            
            p = self.POSES['PickFloor']
            q = Quaternion(x=p['rot'][0], y=p['rot'][1], z=p['rot'][2], w=p['rot'][3])
            self.send_arm_pose(p['pos'][0], p['pos'][1], p['pos'][2], q)
            
            time.sleep(1.0)
            
            self.process_targets(self.floor_classes, "Floor")
            
            self.get_logger().info(f"\n--- Round {round_number} complete ---")
            time.sleep(1.0)

    def process_targets(self, class_ids, zone_name):
        """处理指定列表的目标"""
        picked_count = 0
        for class_id in class_ids:
            if not rclpy.ok():
                break
                
            class_name = self.class_id_to_name.get(class_id, f"unknown({class_id})")
            self.get_logger().info(f"\n--- [{zone_name}] Looking for '{class_name}' (id={class_id}) ---")
            
            # 获取检测结果 (Stable Strategy)
            detection = self.get_stable_detection(class_id, collection_time=2.0)
            
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
            
            grasp_pose = detection.results[0].pose.pose
            
            # 执行抓取和放置
            if self.perform_pick_and_place(grasp_pose, class_id):
                picked_count += 1
                self.get_logger().info(f"Successfully picked '{class_name}'!")
                # Pick 成功后返回Home
                self.perform_go_home()
            else:
                self.get_logger().error(f"Failed to pick '{class_name}'")
        
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
