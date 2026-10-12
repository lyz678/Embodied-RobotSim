#!/usr/bin/env python3
"""
LLM Robot Agent Server
使用 Qwen3 自然语言控制机器人：导航、场景识别、物体抓取
端口: 8889   前端: http://localhost:8889
"""

import asyncio, threading, time, math, base64, io, json, os, traceback, logging, copy, sys
from pathlib import Path
from typing import Optional, List, Dict, Any, Set
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Int32, Bool
from vision_msgs.msg import Detection3DArray
from control_msgs.action import GripperCommand, FollowJointTrajectory
from std_srvs.srv import Trigger, SetBool
from moveit_msgs.msg import CollisionObject
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # Register PoseStamped transforms for Buffer.transform.
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry

import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
import uvicorn
from openai import OpenAI
from model_config import ModelConfig, create_client
from fetch_policy import explicit_fetch, stable_grasp, approach_axis
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'manipulation/x_bot_manipulation/scripts'))
from gripper_action import control_gripper

# ─────────────────────────── 配置 ──────────────────────────────────────────

AGENT_PORT  = 8889
MODEL_CONFIG = ModelConfig.from_env()
AGENT_MODEL = MODEL_CONFIG.agent_model
VL_MODEL = MODEL_CONFIG.vision_model
API_KEY = MODEL_CONFIG.api_key
API_BASE = MODEL_CONFIG.base_url

LOCATIONS: Dict[str, Dict] = {
    "厨房": {"x": 3.76,  "y": 2.22,  "yaw_deg": 6.0},
    "客厅": {"x": 1.44,  "y": 3.32,  "yaw_deg": 66.9},
    "起点": {"x": 0.0,   "y": 0.0,   "yaw_deg": 0.0},
    "书房": {"x": -1.5,  "y": 2.0,   "yaw_deg": 90.0},
    "卧室": {"x": -2.5,  "y": -1.5,  "yaw_deg": 180.0},
}

OBJECTS: Dict[str, int] = {
    "可乐": 9, "杯子": 2, "书": 3, "瓶子": 4,
}

TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "navigate_to_location",
            "description": "导航机器人到指定的预设地点（厨房、客厅、书房、卧室、起点）",
            "parameters": {
                "type": "object",
                "properties": {
                    "location": {
                        "type": "string",
                        "description": f"目标地点名称，可选: {', '.join(LOCATIONS.keys())}",
                        "enum": list(LOCATIONS.keys()),
                    }
                },
                "required": ["location"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "pick_object",
            "description": "扫描并夹取指定物体（需先导航到物体附近）",
            "parameters": {
                "type": "object",
                "properties": {
                    "object_name": {
                        "type": "string",
                        "description": f"要抓取的物体名称，可选: {', '.join(OBJECTS.keys())}",
                        "enum": list(OBJECTS.keys()),
                    }
                },
                "required": ["object_name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "move_robot",
            "description": "直接控制机器人底盘移动（前进/后退/左转/右转/停止）",
            "parameters": {
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": ["forward", "backward", "turn_left", "turn_right", "stop"],
                        "description": "移动方向",
                    },
                    "speed":    {"type": "number", "description": "速度 m/s (0.05–0.5)", "default": 0.2},
                    "duration": {"type": "number", "description": "持续时间（秒）",       "default": 1.0},
                },
                "required": ["direction"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "recognize_scene",
            "description": "使用视觉AI识别机器人当前所在房间（厨房/客厅/书房/卧室等）",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "arm_go_home",
            "description": "将机械臂归位到安全初始位置",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop_robot",
            "description": "立即停止机器人所有移动",
            "parameters": {"type": "object", "properties": {}},
        },
    },
]

SYSTEM_PROMPT = f"""你是一个智能家庭服务机器人的AI助手，可以用自然语言控制机器人完成各种任务。

机器人能力：
- 导航到指定房间（厨房、客厅、书房、卧室、起点）
- 识别并抓取物体（可乐、杯子、书、瓶子等）
- 直接底盘移动控制（前进/后退/左转/右转）
- 识别当前所在场景/房间
- 机械臂归位

预设导航点：
- 厨房：X=3.76m, Y=2.22m（可获取饮料等物品）
- 客厅：X=1.44m, Y=3.32m
- 起点：X=0.0, Y=0.0

任务执行规则：
1. 理解用户意图，将复杂任务分解为步骤
2. 按步骤依次调用工具
3. 每步完成后报告进度
4. 任务结束后给出简洁总结
5. 导航和抓取耗时较长（30~90秒），请在执行前告知用户
"""

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("llm_agent")

# ─────────────────────────── 工具函数 ──────────────────────────────────────

def _yaw_to_quaternion(yaw_deg: float) -> Quaternion:
    yaw = math.radians(yaw_deg)
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

# ─────────────────────────── ROS2 节点 ─────────────────────────────────────

class LLMAgentNode(Node):
    """封装所有 ROS2 机器人控制能力"""

    def __init__(self):
        super().__init__("llm_agent_node")

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # 底盘 cmd_vel
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        # 机械臂
        self.pub_arm = self.create_publisher(PoseStamped, "/arm_command/pose", 10)
        self.pub_cartesian = self.create_publisher(PoseStamped, "/arm_command/cartesian_pose", 10)
        self.gripper_client   = ActionClient(self, GripperCommand,        "/fr3_gripper_controller/gripper_cmd")
        self.arm_joint_client = ActionClient(self, FollowJointTrajectory, "/fr3_arm_controller/follow_joint_trajectory")

        # 点云 & Octomap
        self.pub_cloud_filter  = self.create_publisher(Int32,            "set_cloud_filter",  10)
        self.pub_collision_obj = self.create_publisher(CollisionObject,  "collision_object",  10)

        # 服务客户端
        self.go_home_client  = self.create_client(Trigger, "/robot_actions/go_home")
        self.scan_client     = self.create_client(Trigger, "/robot_actions/scan")
        self.yoloe_client    = self.create_client(SetBool, "/yoloe_multi_text_prompt/enable_inference")
        self.graspnet_client = self.create_client(SetBool, "/graspnet_node/enable_inference")

        # 状态
        self._arm_status: Optional[str] = None
        self._latest_detections         = []
        self._detections_received = -float('inf')
        self._detection_generation = 0
        self._detection_lock = threading.Lock()
        self._finger_position = None
        self._joints_received = -float('inf')
        self._base_speed = (float('inf'),float('inf'))
        self._base_pose = None
        self._odom_received = -float('inf')
        self._nav_handle = None
        self._nav_future = None
        self._nav_cancel_requested = False
        self._camera_image: Optional[Image] = None
        self.robot_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.current_scene = "未知"
        self._localization_ready = False
        self._localization_received = -float('inf')

        self.create_subscription(Bool, "/localization/ready", self._cb_localization, 10)
        self.create_subscription(String,          "/arm_command/status",       self._cb_arm_status,   10)
        self.create_subscription(Detection3DArray, "/graspnet/detections_3d",  self._cb_detections,   10)
        self.create_subscription(Image,            "/x_bot/camera_left/image_raw", self._cb_camera,   1)
        self.create_subscription(Odometry,         "/odom",                    self._cb_odom,         10)
        self.create_subscription(JointState, '/joint_states', self._cb_joints, 10)

        self.get_logger().info("LLM Agent Node ready")

    # ── 回调 ──────────────────────────────────────────────────────────────

    def _cb_arm_status(self, msg):  self._arm_status = msg.data
    def _cb_detections(self, msg):
        with self._detection_lock:
            self._latest_detections = copy.deepcopy(msg.detections)
            for det in self._latest_detections:
                if not det.header.frame_id:det.header=copy.deepcopy(msg.header)
            self._detections_received=time.monotonic()
            self._detection_generation+=1

    def _cb_joints(self,msg):
        if 'fr3_finger_joint1' in msg.name:
            self._finger_position=float(msg.position[msg.name.index('fr3_finger_joint1')])
            self._joints_received=time.monotonic()

    def _cb_localization(self, msg):
        self._localization_ready = msg.data
        self._localization_received = time.monotonic()

    def localization_ready(self):
        return self._localization_ready and time.monotonic() - self._localization_received < .5

    def _cb_camera(self, msg):
        self._camera_image = msg

    def _cb_odom(self, msg):
        self._odom_received=time.monotonic()
        self._base_speed=(math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y),abs(msg.twist.twist.angular.z))
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self._base_pose = (p.x, p.y, math.atan2(siny, cosy))
        self.robot_pose = {"x": round(p.x, 2), "y": round(p.y, 2),
                           "yaw": round(math.degrees(math.atan2(siny, cosy)), 1)}

    # ── 图像转 base64 ───────────────────────────────────────────────────────

    def get_camera_base64(self) -> Optional[str]:
        img = self._camera_image
        if img is None:
            return None
        try:
            from PIL import Image as PILImage
            h, w = img.height, img.width
            data = np.frombuffer(bytes(img.data), dtype=np.uint8)
            enc  = img.encoding
            if enc == "bgr8":
                arr = data.reshape(h, w, 3)[:, :, ::-1]
            elif enc == "rgb8":
                arr = data.reshape(h, w, 3)
            else:
                return None
            pil = PILImage.fromarray(arr)
            pil.thumbnail((640, 640))
            buf = io.BytesIO()
            pil.save(buf, format="JPEG", quality=75)
            return base64.b64encode(buf.getvalue()).decode("utf-8")
        except Exception as e:
            self.get_logger().warn(f"Camera convert: {e}")
            return None

    # ── 导航（通过 asyncio.Future 返回结果）────────────────────────────────

    def send_nav_goal(self, x: float, y: float, yaw_deg: float,
                      future: asyncio.Future, loop: asyncio.AbstractEventLoop):
        self._nav_future=future
        self._nav_cancel_requested=False
        def resolve(value):
            def apply():
                if not future.done():future.set_result(value)
            loop.call_soon_threadsafe(apply)
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            resolve((False, "Nav2 服务不可用"))
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x  = float(x)
        goal.pose.pose.position.y  = float(y)
        goal.pose.pose.orientation = _yaw_to_quaternion(yaw_deg)

        def on_goal(f):
            handle = f.result()
            if future.cancelled() or (self._nav_future is future and self._nav_cancel_requested):
                if handle.accepted:handle.cancel_goal_async()
                resolve((False,'导航已取消'))
                return
            self._nav_handle=handle
            if not handle.accepted:
                resolve((False, "目标被拒绝"))
                return
            handle.get_result_async().add_done_callback(on_result)

        def on_result(rf):
            ok  = rf.result().status == 4
            msg = "导航成功" if ok else f"导航失败(status={rf.result().status})"
            if self._nav_future is future:self._nav_handle=None
            resolve((ok, msg))

        self.nav_client.send_goal_async(goal).add_done_callback(on_goal)

    def cancel_navigation(self):
        self._nav_cancel_requested=True
        if self._nav_handle and self._nav_handle.accepted:self._nav_handle.cancel_goal_async()
        self.pub_vel.publish(Twist())

    # ── 服务调用（同步，在线程池中使用）────────────────────────────────────

    def call_trigger(self, client, timeout=20.0) -> bool:
        if not client.wait_for_service(timeout_sec=2.0):
            return False
        f = client.call_async(Trigger.Request())
        dl = time.time() + timeout
        while not f.done() and time.time() < dl:
            time.sleep(0.05)
        return f.done() and f.result().success

    def set_inference(self, enable: bool):
        req = SetBool.Request(); req.data = enable
        for c in [self.yoloe_client, self.graspnet_client]:
            if c.wait_for_service(timeout_sec=0.5):
                f = c.call_async(req)
                dl = time.time() + 3.0
                while not f.done() and time.time() < dl:
                    time.sleep(0.05)

    # ── 夹爪（同步）─────────────────────────────────────────────────────────

    def gripper_sync(self, pos: float, timeout=20.0) -> bool:
        return control_gripper(self,self.gripper_client,pos,timeout)

    # ── 机械臂姿态（同步）──────────────────────────────────────────────────

    def arm_pose_sync(self, x, y, z, orientation: Quaternion, timeout=35.0, cartesian=False) -> bool:
        pose = PoseStamped()
        pose.header.frame_id  = "odom"
        pose.header.stamp     = self.get_clock().now().to_msg()
        pose.pose.position.x  = float(x)
        pose.pose.position.y  = float(y)
        pose.pose.position.z  = float(z)
        pose.pose.orientation = orientation
        self._arm_status = None
        (self.pub_cartesian if cartesian else self.pub_arm).publish(pose)
        dl = time.time() + timeout
        while time.time() < dl:
            if self._arm_status:
                return self._arm_status == "SUCCESS"
            time.sleep(0.1)
        return False

    # ── 检测聚类 ────────────────────────────────────────────────────────────

    def get_best_detection(self, class_id: int, collect_sec=4.0):
        candidates=[];seen=set();generation=-1;start=time.monotonic()
        while time.monotonic()-start<collect_sec:
            with self._detection_lock:
                fresh=self._detections_received>=start and time.monotonic()-self._detections_received<.75
                rows=copy.deepcopy(self._latest_detections) if fresh and generation!=self._detection_generation else []
                generation=self._detection_generation
            for det in rows:
                if not det.results or not det.header.frame_id:continue
                result=det.results[0]
                try:matched=int(result.hypothesis.class_id)==class_id
                except (ValueError,TypeError):continue
                stamp=(det.header.stamp.sec,det.header.stamp.nanosec)
                if matched and result.hypothesis.score>=.25 and stamp not in seen:
                    age=(self.get_clock().now().nanoseconds-(stamp[0]*10**9+stamp[1]))/1e9
                    if not -.1<=age<=1.:continue
                    seen.add(stamp)
                    pose=PoseStamped();pose.header=det.header;pose.pose=result.pose.pose
                    try:
                        if pose.header.frame_id!='odom':pose=self.tf_buffer.transform(pose,'odom')
                    except Exception:continue
                    det.header=pose.header;det.results[0].pose.pose=pose.pose;candidates.append(det)
            time.sleep(.05)
        best=stable_grasp(candidates)
        self.get_logger().info(f'Grasp candidates: class={class_id}, unique_fresh={len(candidates)}, stable={best is not None}')
        return best

    def base_stopped(self, timeout=5.):
        # LIO's differentiated linear velocity is noisy even while stationary.
        # Require a fresh, stable pose window as well as a low speed bound.
        end=time.monotonic()+timeout;window=[]
        while time.monotonic()<end:
            now=time.monotonic();pose=self._base_pose
            if now-self._odom_received>=.5 or pose is None or max(self._base_speed)>=.06:
                window=[]
            else:
                window.append((now,pose))
                window=[row for row in window if now-row[0]<=1.2]
                first=window[0][1]
                stable=all(math.hypot(p[0]-first[0],p[1]-first[1])<.01 and
                    abs(math.atan2(math.sin(p[2]-first[2]),math.cos(p[2]-first[2])))<.015 for _,p in window)
                if not stable:window=[(now,pose)]
                elif now-window[0][0]>=1.:return True
            time.sleep(.05)
        self.get_logger().warn(f'Base settling timed out: speed={self._base_speed}')
        return False

    def grasp_contact(self):
        return time.monotonic()-self._joints_received<.75 and self._finger_position is not None and .008<self._finger_position<.055

    def verify_arrival(self, location):
        self.pub_vel.publish(Twist())
        if not self.base_stopped():
            return False, '到达后底座未停稳，取消抓取'
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            q = tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            error = math.hypot(tf.transform.translation.x-location['x'], tf.transform.translation.y-location['y'])
            angle = abs(math.atan2(math.sin(yaw-math.radians(location['yaw_deg'])), math.cos(yaw-math.radians(location['yaw_deg']))))
            detail = f'停车误差 {error:.3f} m / {math.degrees(angle):.1f}°'
            self.get_logger().info(detail)
            if not -.1 <= age <= .5:
                return False, '到达位置变换过期，取消抓取'
            return (True, detail) if error <= .12 and angle <= .15 else (False, detail+'，取消抓取')
        except Exception as exc:
            return False, f'无法核验到达位置：{exc}'

    def pick_sequence(self, class_id: int, progress_cb=None) -> tuple:
        stage='stop_base'
        def prog(msg):
            self.get_logger().info(f'PICK class={class_id} stage={stage}: {msg}')
            if progress_cb:progress_cb(msg)
        def fail(message):
            return False,f'夹取失败[{stage}]：{message}'
        try:
            self.pub_vel.publish(Twist())
            if not self.base_stopped():return fail('底座尚未停稳或里程计过期')
            stage='home';prog('机械臂归位...')
            if not self.call_trigger(self.go_home_client,timeout=45.):return fail('归位失败')
            stage='open'
            if not self.gripper_sync(.06):return fail('夹爪未能打开')
            self.set_inference(True)
            stage='detect';prog('收集新鲜、稳定的指定类别抓取位姿...')
            det=self.get_best_detection(class_id)
            if det is None:
                stage='scan';prog('未获得稳定目标，扫描场景...')
                if not self.call_trigger(self.scan_client,timeout=45.):return fail('扫描失败')
                stage='detect';det=self.get_best_detection(class_id)
            if det is None:return fail('没有连续稳定的新鲜目标位姿')
            grasp=det.results[0].pose.pose
            axis=approach_axis(grasp.orientation)
            self.set_inference(False)
            self.pub_cloud_filter.publish(Int32(data=class_id))
            obj=CollisionObject();obj.header.frame_id='odom';obj.id=f'target_object_{class_id}';obj.operation=CollisionObject.REMOVE
            self.pub_collision_obj.publish(obj);time.sleep(2.)
            stage='pregrasp';prog('移至抓取位前8cm...')
            xyz=np.array([grasp.position.x,grasp.position.y,grasp.position.z]);pre=xyz-.08*axis
            if not self.arm_pose_sync(*pre,grasp.orientation):return fail('预抓取规划或执行失败')
            stage='approach';prog('沿抓取轴直线接近...')
            if not self.arm_pose_sync(*xyz,grasp.orientation,cartesian=True):return fail('直线接近失败')
            stage='close';prog('闭合夹爪...')
            if not self.gripper_sync(.005):return fail('夹爪闭合失败')
            time.sleep(.4)
            if not self.grasp_contact():return fail('夹爪已空闭合或关节观测过期')
            stage='lift';prog('先竖直抬高12cm...')
            if not self.arm_pose_sync(xyz[0],xyz[1],xyz[2]+.12,grasp.orientation,cartesian=True):return fail('抬起失败')
            if not self.grasp_contact():return fail('抬起后夹爪空闭合，可能脱落')
            stage='return_home';prog('夹持返回Home...')
            if not self.call_trigger(self.go_home_client,timeout=45.):return fail('持物返回失败')
            stage='hold';prog('保持夹持并检查2秒...')
            end=time.monotonic()+2.
            while time.monotonic()<end:
                if not self.grasp_contact():return fail('返回后失去夹持或关节数据过期')
                time.sleep(.05)
            return True,'机械臂已完成抓取动作、抬起和回Home，夹爪保持夹持'
        except Exception as exc:
            self.get_logger().error(f'PICK exception stage={stage}: {exc}')
            return fail(str(exc))
        finally:
            self.pub_cloud_filter.publish(Int32(data=-1))
            self.set_inference(True)

    # ── cmd_vel 直接移动 ────────────────────────────────────────────────────

    def move_direct(self, direction: str, speed: float, duration: float):
        msg = Twist()
        if   direction == "forward":    msg.linear.x  =  min(speed, 0.5)
        elif direction == "backward":   msg.linear.x  = -min(speed, 0.5)
        elif direction == "turn_left":  msg.angular.z =  min(speed, 1.0)
        elif direction == "turn_right": msg.angular.z = -min(speed, 1.0)
        # stop: all zero
        end = time.time() + duration
        while time.time() < end:
            self.pub_vel.publish(msg)
            time.sleep(0.1)
        self.pub_vel.publish(Twist())


# ─────────────────────────── Robot Controller（async 封装） ─────────────────

class RobotController:
    def __init__(self, node: LLMAgentNode, loop: asyncio.AbstractEventLoop):
        self._node   = node
        self._loop   = loop
        self._exec   = ThreadPoolExecutor(max_workers=4)
        self._pick_lock = asyncio.Lock()

    async def navigate(self, location: str) -> tuple:
        async with self._pick_lock:
            return await self._navigate(location)

    async def _navigate(self, location: str) -> tuple:
        if location not in LOCATIONS:
            return False, f"未知地点: {location}"
        loc = LOCATIONS[location]
        future: asyncio.Future = self._loop.create_future()
        # 在 ROS2 线程中发起导航
        self._loop.run_in_executor(
            self._exec,
            lambda: self._node.send_nav_goal(loc["x"], loc["y"], loc["yaw_deg"], future, self._loop)
        )
        try:
            result = await asyncio.wait_for(asyncio.shield(future), timeout=100.0)
            if not result[0]:
                return result
            return await self._loop.run_in_executor(self._exec, lambda: self._node.verify_arrival(loc))
        except asyncio.TimeoutError:
            future.cancel()
            self._node.cancel_navigation()
            return False, "导航超时"

    async def pick(self, object_name: str, progress_cb=None) -> tuple:
        if object_name not in OBJECTS:
            return False, f"未知物体: {object_name}"
        class_id = OBJECTS[object_name]
        async with self._pick_lock:
            return await self._loop.run_in_executor(
                self._exec,
                lambda: self._node.pick_sequence(class_id, progress_cb)
            )

    async def move(self, direction: str, speed: float = 0.2, duration: float = 1.0):
        if direction=='stop':
            self._node.cancel_navigation();return
        async with self._pick_lock:
            await self._loop.run_in_executor(self._exec,lambda:self._node.move_direct(direction,speed,duration))

    async def arm_go_home(self) -> bool:
        async with self._pick_lock:
            return await self._loop.run_in_executor(self._exec,lambda:self._node.call_trigger(self._node.go_home_client))

    async def stop(self):
        self._node.cancel_navigation()

    async def recognize_scene(self, vl_client: OpenAI) -> str:
        b64 = await self._loop.run_in_executor(self._exec, self._node.get_camera_base64)
        if b64 is None:
            return "无法获取相机图像"
        try:
            resp = await self._loop.run_in_executor(
                self._exec,
                lambda: vl_client.chat.completions.create(
                    model=VL_MODEL,
                    **MODEL_CONFIG.completion_options(),
                    messages=[{
                        "role": "user",
                        "content": [
                            {"type": "image_url",
                             "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                            {"type": "text",
                             "text": "请识别图像中的室内场景，判断这是哪个房间（厨房/客厅/书房/卧室/走廊/其他），"
                                     "并用一句话描述你看到的主要物品。格式：[房间名] 描述"},
                        ],
                    }],
                )
            )
            result = resp.choices[0].message.content.strip()
            # 提取房间名写入节点状态
            for room in ["厨房", "客厅", "书房", "卧室", "走廊"]:
                if room in result:
                    self._node.current_scene = room
                    break
            else:
                self._node.current_scene = "未知"
            return result
        except Exception as e:
            return f"场景识别失败: {e}"


# ─────────────────────────── Qwen3 Agent ───────────────────────────────────

class Qwen3Agent:
    def __init__(self, robot: RobotController):
        self._robot   = robot
        self._client  = create_client(MODEL_CONFIG) if API_KEY else None
        self._vl_client = create_client(MODEL_CONFIG) if API_KEY else None

    async def chat(self, user_msg: str, history: List[Dict], send: callable):
        """
        运行一轮对话，通过 send(event_dict) 向前端推送事件。
        """
        intent=explicit_fetch(user_msg,LOCATIONS,OBJECTS)
        if intent:
            location,object_name=intent
            for index,(name,args) in enumerate((('navigate_to_location',dict(location=location)),('pick_object',dict(object_name=object_name)))):
                identifier=f'fetch-{index}'
                await send(dict(type='tool_call',id=identifier,name=name,args=args,status='running'))
                ok,message=await self._execute_tool(name,args,send)
                await send(dict(type='tool_result',id=identifier,name=name,result=message,success=ok))
                log.info('FETCH stage=%s success=%s result=%s',name,ok,message)
                if not ok:
                    await send(dict(type='assistant',content=f'任务停止：{message}'));return
            await send(dict(type='assistant',content=f'已到{location}，{message}。'));return
        if self._client is None:
            await send({"type": "error", "content": "请设置 DASHSCOPE_API_KEY 并重启具身入口后发送指令。"})
            return
        # 判断是否需要先识别场景
        if any(k in user_msg for k in ["去", "拿", "到", "找"]):
            await send({"type": "status", "task": "正在识别当前场景..."})
            scene_desc = await self._robot.recognize_scene(self._vl_client)
            await send({"type": "status", "task": f"场景识别结果: {scene_desc}"})
            user_msg = f"当前场景是: {scene_desc}。\n用户指令: {user_msg}"

        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages.extend(MODEL_CONFIG.history(history))
        messages.append({"role": "user", "content": user_msg})

        loop = asyncio.get_event_loop()
        max_rounds = 8  # 防止无限循环

        for _ in range(max_rounds):
            # 调用 Qwen3
            try:
                response = await loop.run_in_executor(
                    None,
                    lambda: self._client.chat.completions.create(
                        model=AGENT_MODEL,
                        messages=messages,
                        tools=TOOLS,
                        tool_choice="auto",
                        **MODEL_CONFIG.completion_options(),
                    )
                )
            except Exception as e:
                await send({"type": "error", "content": f"Qwen3 API 错误: {e}"})
                return

            choice = response.choices[0]
            msg    = choice.message

            # 思考内容（若模型返回 reasoning_content）
            if hasattr(msg, "reasoning_content") and msg.reasoning_content:
                await send({"type": "thinking", "content": msg.reasoning_content})

            # 追加到历史
            messages.append({"role": "assistant", "content": msg.content or "",
                              "tool_calls": [tc.model_dump() for tc in msg.tool_calls] if msg.tool_calls else None})

            # 没有工具调用 → 最终回复
            if not msg.tool_calls:
                await send({"type": "assistant", "content": msg.content or ""})
                return

            # 执行工具调用
            for tc in msg.tool_calls:
                fn   = tc.function.name
                try:
                    args = json.loads(tc.function.arguments) if tc.function.arguments else {}
                    if not isinstance(args, dict):
                        raise ValueError("Tool arguments must be a JSON object")
                except (ValueError, TypeError) as exc:
                    messages.append({"role": "tool", "tool_call_id": tc.id,
                                     "content": f"工具参数无效，请修正 JSON 参数: {exc}"})
                    continue
                await send({"type": "tool_call", "id": tc.id, "name": fn, "args": args, "status": "running"})

                ok, result_text = await self._execute_tool(fn, args, send)

                await send({"type": "tool_result", "id": tc.id, "name": fn,
                            "result": result_text, "success": ok})

                # 工具结果追加到 messages
                messages.append({
                    "role": "tool",
                    "tool_call_id": tc.id,
                    "content": result_text,
                })

        await send({"type": "error", "content": "超过最大工具调用轮次"})

    async def _execute_tool(self, name: str, args: dict, send: callable) -> tuple:
        r = self._robot
        try:
            needs_localization = name in {"navigate_to_location", "pick_object", "arm_go_home"}
            needs_localization |= name == "move_robot" and args.get("direction") != "stop"
            if needs_localization and not r._node.localization_ready():
                return False, "定位未就绪或状态已过期，暂不能执行机器人动作；请检查激光、IMU、里程计和 /localization/ready"
            if name == "navigate_to_location":
                loc = args["location"]
                await send({"type": "status", "task": f"导航→{loc}"})
                ok, msg = await r.navigate(loc)
                self._navigation_failed=not ok
                if ok: self._robot._node.current_scene = loc
                return ok, msg

            elif name == "pick_object":
                if getattr(self,'_navigation_failed',False):
                    return False,'上次导航未成功，先完成导航再抓取'
                obj = args["object_name"]
                await send({"type": "status", "task": f"夹取{obj}"})

                async def prog(txt):
                    await send({"type": "progress", "content": txt})

                # 包装同步 progress_cb 为异步
                loop = asyncio.get_event_loop()
                def sync_prog(txt):
                    loop.call_soon_threadsafe(
                        asyncio.ensure_future,
                        prog(txt)
                    )
                ok, msg = await r.pick(obj, sync_prog)
                return ok, msg

            elif name == "move_robot":
                direction = args["direction"]
                speed     = float(args.get("speed",    0.2))
                duration  = float(args.get("duration", 1.0))
                await r.move(direction, speed, duration)
                return True, f"移动完成（{direction} {duration}s）"

            elif name == "recognize_scene":
                result = await r.recognize_scene(self._vl_client)
                await send({"type": "scene_update", "scene": self._robot._node.current_scene,
                            "description": result})
                return True, result

            elif name == "arm_go_home":
                ok = await r.arm_go_home()
                return ok, "机械臂已归位" if ok else "归位失败"

            elif name == "stop_robot":
                await r.stop()
                return True, "机器人已停止"

            else:
                return False, f"未知工具: {name}"

        except Exception as e:
            log.error(traceback.format_exc())
            return False, f"工具执行错误: {e}"


# ─────────────────────────── FastAPI App ───────────────────────────────────

app = FastAPI(title="LLM Robot Agent")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)
_agent: Optional[Qwen3Agent]    = None
_node:  Optional[LLMAgentNode]  = None
_connections: Set[WebSocket]    = set()


@app.get("/")
async def index():
    return RedirectResponse(os.getenv("WEB_UI_URL", "http://localhost:8888"))


@app.get("/api/status")
async def status():
    if _node is None:
        return {"error": "ROS2 not ready"}
    return {
        "llm_configured": bool(API_KEY),
        "llm_backend": MODEL_CONFIG.backend,
        "llm_model": AGENT_MODEL,
        "localization_ready": _node.localization_ready(),
        "pose":  _node.robot_pose,
        "scene": _node.current_scene,
        "locations": {k: {"x": v["x"], "y": v["y"]} for k, v in LOCATIONS.items()},
    }


@app.websocket("/ws/chat")
async def ws_chat(ws: WebSocket):
    await ws.accept()
    _connections.add(ws)
    history: List[Dict] = []

    async def send(event: dict):
        """推送事件到前端，顺便同步 status"""
        if _node:
            event["_pose"]  = _node.robot_pose
            event["_scene"] = _node.current_scene
        try:
            await ws.send_json(event)
        except Exception:
            pass

    try:
        while True:
            data = await ws.receive_json()
            if data.get("type") == "chat":
                user_msg = data.get("message", "").strip()
                if not user_msg:
                    continue
                # 记录用户消息到历史（不含工具轮次）
                await _agent.chat(user_msg, history, send)
                # 更新历史（仅保留 user/assistant 轮次，不超 20 轮）
                history.append({"role": "user",      "content": user_msg})
                if len(history) > 40:
                    history = history[-40:]
                await send({"type": "done"})

            elif data.get("type") == "scene_request":
                result = await _agent._robot.recognize_scene(_agent._vl_client)
                await send({"type": "scene_update",
                            "scene": _node.current_scene if _node else "未知",
                            "description": result})
                await send({"type": "done"})

    except WebSocketDisconnect:
        pass
    finally:
        _connections.discard(ws)


# ─────────────────────────── 主入口 ────────────────────────────────────────

def main():
    global _agent, _node

    rclpy.init()
    _node = LLMAgentNode()

    # ROS2 在后台线程运行
    ros_executor = MultiThreadedExecutor(num_threads=4)
    ros_executor.add_node(_node)
    ros_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    ros_thread.start()
    log.info("ROS2 executor started")

    # 获取 asyncio event loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    controller = RobotController(_node, loop)
    _agent     = Qwen3Agent(controller)
    _node.send_nav_goal  # ensure callable

    log.info(f"LLM Agent Server starting on http://0.0.0.0:{AGENT_PORT}")

    config = uvicorn.Config(app, host="0.0.0.0", port=AGENT_PORT,
                            log_level="warning", loop="asyncio")
    server = uvicorn.Server(config)
    loop.run_until_complete(server.serve())


if __name__ == "__main__":
    main()
