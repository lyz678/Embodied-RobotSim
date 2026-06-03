#!/usr/bin/env python3
"""
LLM Robot Agent Server
使用 Qwen3 自然语言控制机器人：导航、场景识别、物体抓取
端口: 8889   前端: http://localhost:8889
"""

import asyncio, threading, time, math, base64, io, json, os, traceback, logging
from typing import Optional, List, Dict, Any, Set
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Int32
from vision_msgs.msg import Detection3DArray
from control_msgs.action import GripperCommand, FollowJointTrajectory
from std_srvs.srv import Trigger, SetBool
from moveit_msgs.msg import CollisionObject
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
import uvicorn
from openai import OpenAI

# ─────────────────────────── 配置 ──────────────────────────────────────────

AGENT_PORT  = 8889
AGENT_MODEL = os.getenv("AGENT_MODEL", "qwen3.6-flash")
VL_MODEL    = os.getenv("VL_MODEL",    "qwen3.6-flash")
API_KEY     = os.getenv("DASHSCOPE_API_KEY", "")
API_BASE    = "https://dashscope.aliyuncs.com/compatible-mode/v1"

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
        self._camera_image: Optional[Image] = None
        self.robot_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.current_scene = "未知"

        self.create_subscription(String,          "/arm_command/status",       self._cb_arm_status,   10)
        self.create_subscription(Detection3DArray, "/graspnet/detections_3d",  self._cb_detections,   10)
        self.create_subscription(Image,            "/x_bot/camera_left/image_raw", self._cb_camera,   1)
        self.create_subscription(Odometry,         "/odom",                    self._cb_odom,         10)

        self.get_logger().info("LLM Agent Node ready")

    # ── 回调 ──────────────────────────────────────────────────────────────

    def _cb_arm_status(self, msg):  self._arm_status = msg.data
    def _cb_detections(self, msg):  self._latest_detections = msg.detections

    def _cb_camera(self, msg):
        self._camera_image = msg

    def _cb_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
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
            buf = io.BytesIO()
            pil.save(buf, format="JPEG", quality=75)
            return base64.b64encode(buf.getvalue()).decode("utf-8")
        except Exception as e:
            self.get_logger().warn(f"Camera convert: {e}")
            return None

    # ── 导航（通过 asyncio.Future 返回结果）────────────────────────────────

    def send_nav_goal(self, x: float, y: float, yaw_deg: float,
                      future: asyncio.Future, loop: asyncio.AbstractEventLoop):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            loop.call_soon_threadsafe(future.set_result, (False, "Nav2 服务不可用"))
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x  = float(x)
        goal.pose.pose.position.y  = float(y)
        goal.pose.pose.orientation = _yaw_to_quaternion(yaw_deg)

        def on_goal(f):
            handle = f.result()
            if not handle.accepted:
                loop.call_soon_threadsafe(future.set_result, (False, "目标被拒绝"))
                return
            handle.get_result_async().add_done_callback(on_result)

        def on_result(rf):
            ok  = rf.result().status == 4
            msg = "导航成功" if ok else f"导航失败(status={rf.result().status})"
            loop.call_soon_threadsafe(future.set_result, (ok, msg))

        self.nav_client.send_goal_async(goal).add_done_callback(on_goal)

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

    def gripper_sync(self, pos: float, timeout=12.0) -> bool:
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            return False
        goal = GripperCommand.Goal()
        goal.command.position   = float(pos)
        goal.command.max_effort = 10.0
        f  = self.gripper_client.send_goal_async(goal)
        dl = time.time() + timeout
        while not f.done() and time.time() < dl:
            time.sleep(0.1)
        if not f.done(): return False
        gh = f.result()
        if not gh.accepted: return False
        rf = gh.get_result_async()
        dl = time.time() + timeout
        while not rf.done() and time.time() < dl:
            time.sleep(0.1)
        time.sleep(0.3)
        return True

    # ── 机械臂姿态（同步）──────────────────────────────────────────────────

    def arm_pose_sync(self, x, y, z, orientation: Quaternion, timeout=35.0) -> bool:
        pose = PoseStamped()
        pose.header.frame_id  = "odom"
        pose.header.stamp     = self.get_clock().now().to_msg()
        pose.pose.position.x  = float(x)
        pose.pose.position.y  = float(y)
        pose.pose.position.z  = float(z)
        pose.pose.orientation = orientation
        self._arm_status = None
        self.pub_arm.publish(pose)
        dl = time.time() + timeout
        while time.time() < dl:
            if self._arm_status:
                return "SUCCESS" in self._arm_status
            time.sleep(0.1)
        return False

    # ── 检测聚类 ────────────────────────────────────────────────────────────

    def get_best_detection(self, class_id: int, collect_sec=3.0):
        seen, best = [], None
        start = time.time()
        while time.time() - start < collect_sec:
            for det in self._latest_detections:
                if not det.results: continue
                did  = int(det.results[0].hypothesis.class_id)
                conf = det.results[0].hypothesis.score
                if did == class_id and conf >= 0.25:
                    seen.append(det)
            time.sleep(0.1)
        if seen:
            best = seen[-1]
        return best

    # ── 完整夹取流程（同步，在线程中运行）─────────────────────────────────

    def pick_sequence(self, class_id: int, progress_cb=None) -> tuple:
        def prog(msg):
            if progress_cb: progress_cb(msg)
            else: self.get_logger().info(msg)

        self.set_inference(True); time.sleep(1.0)
        prog("机械臂归位...")
        self.call_trigger(self.go_home_client); time.sleep(1.5)
        prog("尝试直接检测目标物体...")
        det = self.get_best_detection(class_id, collect_sec=3.0)
        
        if det is None:
            prog("未发现目标，开始扫描场景...")
            if not self.call_trigger(self.scan_client, timeout=30.0):
                self.set_inference(False)
                return False, "扫描失败"
            time.sleep(1.0)
            prog("重新检测目标物体...")
            det = self.get_best_detection(class_id, collect_sec=3.0)

        if det is None:
            self.set_inference(False)
            return False, "未检测到目标物体"

        grasp = det.results[0].pose.pose
        self.set_inference(False)
        self.pub_cloud_filter.publish(Int32(data=class_id)); time.sleep(1.0)
        self.gripper_sync(0.06)

        prog(f"机械臂移至抓取位 ({grasp.position.x:.2f},{grasp.position.y:.2f},{grasp.position.z:.2f})...")
        if not self.arm_pose_sync(grasp.position.x, grasp.position.y, grasp.position.z, grasp.orientation):
            self.pub_cloud_filter.publish(Int32(data=-1))
            return False, "机械臂移动失败（规划错误）"

        prog("闭合夹爪...")
        if not self.gripper_sync(0.005):
            self.pub_cloud_filter.publish(Int32(data=-1))
            return False, "夹爪闭合失败"
        time.sleep(0.5)

        # 移除碰撞体
        obj = CollisionObject()
        obj.header.frame_id = "odom"
        obj.header.stamp    = self.get_clock().now().to_msg()
        obj.id              = f"target_object_{class_id}"
        obj.operation       = CollisionObject.REMOVE
        self.pub_collision_obj.publish(obj)

        # 抬起
        prog("抬起物体...")
        try:
            tf = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            dx = tf.transform.translation.x - grasp.position.x
            dy = tf.transform.translation.y - grasp.position.y
            cx = grasp.position.x + dx * 0.8
            cy = grasp.position.y + dy * 0.8
        except:
            cx, cy = grasp.position.x, grasp.position.y
        self.arm_pose_sync(cx, cy, 1.0, grasp.orientation)

        prog("机械臂归位（保持夹爪）...")
        self.call_trigger(self.go_home_client)
        self.gripper_sync(0.005)  # 重新夹紧
        self.pub_cloud_filter.publish(Int32(data=-1))
        return True, "夹取成功！"

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
            return await asyncio.wait_for(asyncio.shield(future), timeout=100.0)
        except asyncio.TimeoutError:
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
        await self._loop.run_in_executor(
            self._exec,
            lambda: self._node.move_direct(direction, speed, duration)
        )

    async def arm_go_home(self) -> bool:
        return await self._loop.run_in_executor(
            self._exec,
            lambda: self._node.call_trigger(self._node.go_home_client)
        )

    async def stop(self):
        self._node.pub_vel.publish(Twist())

    async def recognize_scene(self, vl_client: OpenAI) -> str:
        b64 = await self._loop.run_in_executor(self._exec, self._node.get_camera_base64)
        if b64 is None:
            return "无法获取相机图像"
        try:
            resp = await self._loop.run_in_executor(
                self._exec,
                lambda: vl_client.chat.completions.create(
                    model=VL_MODEL,
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
        self._client  = OpenAI(api_key=API_KEY, base_url=API_BASE)
        self._vl_client = OpenAI(api_key=API_KEY, base_url=API_BASE)

    async def chat(self, user_msg: str, history: List[Dict], send: callable):
        """
        运行一轮对话，通过 send(event_dict) 向前端推送事件。
        """
        # 判断是否需要先识别场景
        if any(k in user_msg for k in ["去", "拿", "到", "找"]):
            await send({"type": "status", "task": "正在识别当前场景..."})
            scene_desc = await self._robot.recognize_scene(self._vl_client)
            await send({"type": "status", "task": f"场景识别结果: {scene_desc}"})
            user_msg = f"当前场景是: {scene_desc}。\n用户指令: {user_msg}"

        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages.extend(history)
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
                args = json.loads(tc.function.arguments) if tc.function.arguments else {}
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
            if name == "navigate_to_location":
                loc = args["location"]
                await send({"type": "status", "task": f"导航→{loc}"})
                ok, msg = await r.navigate(loc)
                if ok: self._robot._node.current_scene = loc
                return ok, msg

            elif name == "pick_object":
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
    return FileResponse(os.path.join(os.path.dirname(__file__), "index.html"))


@app.get("/api/status")
async def status():
    if _node is None:
        return {"error": "ROS2 not ready"}
    return {
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
