/**
 * teleop.js - 键盘遥控控制器
 *
 * 与 teleop_keyboard.py 完全对应的 Web 版本
 *
 * 底盘 (geometry_msgs/Twist → /x_bot/cmd_vel):
 *   W/S - 前进/后退    A/D - 左转/右转
 *   Z/X - 线速 ±0.05   C/V - 角速 ±0.10
 *   Space - 急停
 *
 * 机械臂 (FollowJointTrajectory Action):
 *   1-7 - 选择关节     +/= 增加角度   -/_ 减小角度
 *   [/] - 步长 ±0.05
 *
 * 夹爪 (GripperCommand Action):
 *   O - 张开   P - 关闭
 *
 * 预设姿态 (PoseStamped → /arm_command/pose):
 *   H - 归零   I - Init   K - Pick Floor   L - Place
 */

class TeleopController {
  constructor(ros) {
    this.ros      = ros;
    this.enabled  = false;

    // ─── 底盘状态 ─────────────────────────────────
    this.linearSpeed  = 0.2;
    this.angularSpeed = 0.5;
    this._heldKeys    = new Set();
    this._moveTimer   = null;

    // ─── 机械臂状态 ───────────────────────────────
    this.selectedJoint  = 0;
    this.jointStep      = 0.1;
    this.jointPositions = [0.0, -1.5, 0.0, -2.3561, 0.0, 2.0, 0.7853];

    this.JOINT_LIMITS = [
      [-2.9007,  2.9007],
      [-1.8361,  1.8361],
      [-2.9007,  2.9007],
      [-3.0770, -0.1169],
      [-2.8763,  2.8763],
      [ 0.4398,  4.6216],
      [-3.0508,  3.0508]
    ];

    this.JOINT_NAMES = [
      'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
      'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
    ];

    // ─── 夹爪 ─────────────────────────────────────
    this.gripperOpen  = 0.04;
    this.gripperClose = 0.005;

    // ─── 预设姿态 ─────────────────────────────────
    this.PRESETS = {
      'i': { pos: [0.15,  0.00, 0.94], rot: [-0.00, 0.21,  0.01, 0.98], name: 'Init' },
      'k': { pos: [-0.01, 0.48, 0.51], rot: [-0.50, 0.48,  0.52, 0.50], name: 'Pick Floor' },
      'l': { pos: [-0.02,-0.76, 0.61], rot: [ 0.29, 0.28, -0.66, 0.64], name: 'Place' }
    };

    // ROS 发布者/服务客户端（connect() 后初始化）
    this._cmdVelPub      = null;
    this._posePub        = null;
    this._armTrajPub     = null;   // 机械臂：直接发布轨迹话题
    this._gripperGoalSvc = null;   // 夹爪：调用 action send_goal 服务
  }

  // ─── 连接 ROS ─────────────────────────────────

  connect() {
    if (!this.ros) return;

    // 底盘速度
    this._cmdVelPub = new ROSLIB.Topic({
      ros: this.ros, name: '/x_bot/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    // 预设姿态（Cartesian 空间）
    this._posePub = new ROSLIB.Topic({
      ros: this.ros, name: '/arm_command/pose',
      messageType: 'geometry_msgs/PoseStamped'
    });

    // 机械臂轨迹：直接发布 JointTrajectory 话题
    // joint_trajectory_controller 原生支持此话题，比 Action 更可靠
    this._armTrajPub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/fr3_arm_controller/joint_trajectory',
      messageType: 'trajectory_msgs/JointTrajectory'
    });

    // 订阅关节状态，初始化真实位置（避免与仿真当前状态不同步造成大幅跳动）
    this._syncJointStates();
  }

  /** 从 /joint_states 同步一次实际关节角度，之后由命令追踪 */
  _syncJointStates() {
    const sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/joint_states',
      messageType: 'sensor_msgs/JointState',
      queue_length: 1,
      throttle_rate: 0
    });

    const onMsg = msg => {
      sub.unsubscribe();
      let synced = false;
      for (let i = 0; i < this.JOINT_NAMES.length; i++) {
        const idx = msg.name.indexOf(this.JOINT_NAMES[i]);
        if (idx >= 0 && typeof msg.position[idx] === 'number') {
          this.jointPositions[i] = msg.position[idx];
          synced = true;
        }
      }
      if (synced) {
        console.log('[Teleop] 关节初始位置已同步:', this.jointPositions.map(v => v.toFixed(3)));
        this._updateArmDisplay();
      }
    };
    sub.subscribe(onMsg);
  }

  // ─── 启用/禁用 ─────────────────────────────────

  setEnabled(v) {
    this.enabled = v;
    if (!v) { this._stopBase(); this._heldKeys.clear(); }
    this._updateUI();
  }

  // ─── 底盘控制 ─────────────────────────────────

  _sendVelocity(lin, ang) {
    this._cmdVelPub?.publish(new ROSLIB.Message({
      linear:  { x: lin, y: 0, z: 0 },
      angular: { x: 0,   y: 0, z: ang }
    }));
  }

  _stopBase() {
    clearInterval(this._moveTimer);
    this._moveTimer = null;
    this._sendVelocity(0, 0);
  }

  _updateBaseMovement() {
    clearInterval(this._moveTimer);
    const h = this._heldKeys;
    let lin = 0, ang = 0;
    if (h.has('w')) lin += this.linearSpeed;
    if (h.has('s')) lin -= this.linearSpeed;
    if (h.has('a')) ang += this.angularSpeed;
    if (h.has('d')) ang -= this.angularSpeed;

    this._sendVelocity(lin, ang);
    if (lin !== 0 || ang !== 0) {
      this._moveTimer = setInterval(() => this._sendVelocity(lin, ang), 100);
    }
    this._updateVelDisplay(lin, ang);
  }

  // ─── 机械臂控制 ───────────────────────────────

  _sendArmTrajectory() {
    if (!this.ros || !this.ros.socket || this.ros.socket.readyState !== WebSocket.OPEN) {
      App.showToast('⚠ 机械臂未连接（rosbridge 未就绪）', 'warn', 2000);
      console.warn('[Teleop] WebSocket 未就绪，无法发送轨迹');
      return;
    }

    const positions = this.jointPositions.slice();
    const velocities = [0, 0, 0, 0, 0, 0, 0];
    console.log('[Teleop] 发送机械臂轨迹 (action_goal):', positions.map(v => v.toFixed(3)));

    // 主路：rosbridge 原生 send_action_goal → FollowJointTrajectory
    const msg = {
      op:          'send_action_goal',
      id:          'arm-' + Date.now(),
      action:      '/fr3_arm_controller/follow_joint_trajectory',
      action_type: 'control_msgs/action/FollowJointTrajectory',
      args: {
        trajectory: {
          joint_names: this.JOINT_NAMES,
          points: [{
            positions,
            velocities,
            accelerations: velocities.slice(),
            effort:        [],
            time_from_start: { sec: 2, nanosec: 0 }
          }]
        },
        path_tolerance:  [],
        goal_tolerance:  [],
        goal_time_tolerance: { sec: 1, nanosec: 0 }
      },
      feedback: false
    };
    this.ros.socket.send(JSON.stringify(msg));

    // 备用：同时发布 JointTrajectory 话题（非阻塞，兼容老版本控制器）
    if (this._armTrajPub) {
      this._armTrajPub.publish(new ROSLIB.Message({
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
        joint_names: this.JOINT_NAMES,
        points: [{
          positions,
          velocities,
          accelerations:   velocities.slice(),
          effort:          [],
          time_from_start: { sec: 2, nanosec: 0 }
        }]
      }));
    }

    this._updateArmDisplay();
  }

  adjustJoint(delta) {
    const i = this.selectedJoint;
    const [lo, hi] = this.JOINT_LIMITS[i];
    this.jointPositions[i] = Math.max(lo, Math.min(hi,
      this.jointPositions[i] + delta
    ));
    this._sendArmTrajectory();
    App.showToast(`J${i+1}: ${this.jointPositions[i].toFixed(3)} rad`, 'info', 1000);
  }

  homeArm() {
    this.jointPositions = [0.0, -1.5, 0.0, -2.3561, 0.0, 2.0, 0.7853];
    this._sendArmTrajectory();
    App.showToast('🏠 机械臂归零', 'success');
    this._flashPresetBtn('tp-home', ['tp-pose-i','tp-pose-k','tp-pose-l','tp-home'], 999999);
  }

  // ─── 夹爪控制 ─────────────────────────────────

  _sendGripper(position) {
    if (!this.ros || !this.ros.socket || this.ros.socket.readyState !== WebSocket.OPEN) {
      App.showToast('⚠ 夹爪未连接（rosbridge 未就绪）', 'warn', 2000);
      console.warn('[Teleop] WebSocket 未就绪，position =', position);
      return;
    }

    console.log('[Teleop] 发送夹爪 send_action_goal, position =', position);

    // 使用 rosbridge 原生 send_action_goal 操作协议（比 Service 或 ActionClient 更可靠）
    const msg = {
      op:          'send_action_goal',
      id:          'gripper-' + Date.now(),
      action:      '/fr3_gripper_controller/gripper_cmd',
      action_type: 'control_msgs/action/GripperCommand',
      args:        { command: { position: position, max_effort: 10.0 } },
      feedback:    false
    };
    this.ros.socket.send(JSON.stringify(msg));
  }

  openGripper() {
    this._sendGripper(this.gripperOpen);
    App.showToast('夹爪张开', 'info', 1500);
    // 夹爪状态持久显示（duration=999999 即永久，另一个按钮点击时会互斥清除）
    this._flashPresetBtn('tp-gripper-open', ['tp-gripper-open','tp-gripper-close'], 999999);
  }
  closeGripper() {
    this._sendGripper(this.gripperClose);
    App.showToast('夹爪关闭', 'info', 1500);
    this._flashPresetBtn('tp-gripper-close', ['tp-gripper-open','tp-gripper-close'], 999999);
  }

  // ─── 预设姿态 ─────────────────────────────────

  sendPreset(key) {
    const p = this.PRESETS[key.toLowerCase()];
    if (!p || !this._posePub) return;

    // 归一化四元数
    const r = p.rot;
    const n = Math.sqrt(r[0]**2 + r[1]**2 + r[2]**2 + r[3]**2);
    const [qx, qy, qz, qw] = r.map(v => v / n);

    this._posePub.publish(new ROSLIB.Message({
      header: { frame_id: 'odom', stamp: { secs: 0, nsecs: 0 } },
      pose: {
        position:    { x: p.pos[0], y: p.pos[1], z: p.pos[2] },
        orientation: { x: qx, y: qy, z: qz, w: qw }
      }
    }));
    App.showToast(`📍 运动到 ${p.name}`, 'success');
    const POSE_GROUP = ['tp-pose-i','tp-pose-k','tp-pose-l','tp-home'];
    const ID_MAP = { 'i': 'tp-pose-i', 'k': 'tp-pose-k', 'l': 'tp-pose-l' };
    if (ID_MAP[key]) this._flashPresetBtn(ID_MAP[key], POSE_GROUP, 999999);
  }

  // ─── 按钮高亮辅助 ─────────────────────────────

  /**
   * 点亮 id 对应按钮，duration ms 后自动熄灭；同 group 内其余按钮先灭。
   * 若按钮文字含 ○，点亮时替换为 ●，熄灭/其余按钮恢复 ○。
   */
  _flashPresetBtn(id, group = [], duration = 2000) {
    const setCircle = (el, filled) => {
      if (!el) return;
      el.textContent = el.textContent.replace(filled ? '○' : '●', filled ? '●' : '○');
    };
    group.forEach(gid => {
      if (gid === id) return;
      const el = document.getElementById(gid);
      if (el) { clearTimeout(el._flashTimer); el.classList.remove('active'); setCircle(el, false); }
    });
    const el = document.getElementById(id);
    if (!el) return;
    setCircle(el, true);
    el.classList.add('active');
    clearTimeout(el._flashTimer);
    el._flashTimer = setTimeout(() => {
      el.classList.remove('active');
      setCircle(el, false);
    }, duration);
  }

  // ─── 键盘事件处理 ─────────────────────────────

  /** 返回 true 表示已消费此按键 */
  handleKeyDown(e) {
    if (!this.enabled) return false;
    const k = e.key.toLowerCase();

    // 底盘移动（按住持续运动）
    if (['w', 'a', 's', 'd'].includes(k)) {
      if (!this._heldKeys.has(k)) {
        this._heldKeys.add(k);
        this._updateBaseMovement();
      }
      return true;
    }

    // 线速度调整
    if (k === 'z') { this.linearSpeed  = Math.max(0.05, +(this.linearSpeed  - 0.05).toFixed(2)); this._updateUI(); return true; }
    if (k === 'x') { this.linearSpeed  = Math.min(1.00, +(this.linearSpeed  + 0.05).toFixed(2)); this._updateUI(); return true; }
    if (k === 'c') { this.angularSpeed = Math.max(0.10, +(this.angularSpeed - 0.10).toFixed(2)); this._updateUI(); return true; }
    if (k === 'v') { this.angularSpeed = Math.min(2.00, +(this.angularSpeed + 0.10).toFixed(2)); this._updateUI(); return true; }

    // 关节选择 1-7
    if ('1234567'.includes(e.key) && e.key >= '1' && e.key <= '7') {
      this.selectedJoint = parseInt(e.key) - 1;
      this._updateArmDisplay();
      return true;
    }

    // 关节角度调整
    if (e.key === '+' || e.key === '=') { this.adjustJoint( this.jointStep); return true; }
    if (e.key === '-' || e.key === '_') { this.adjustJoint(-this.jointStep); return true; }

    // 步长调整
    if (e.key === '[') { this.jointStep = Math.max(0.01, +(this.jointStep - 0.05).toFixed(2)); this._updateArmDisplay(); return true; }
    if (e.key === ']') { this.jointStep = Math.min(0.50, +(this.jointStep + 0.05).toFixed(2)); this._updateArmDisplay(); return true; }

    // 夹爪
    if (k === 'o') { this.openGripper();  return true; }
    if (k === 'p') { this.closeGripper(); return true; }

    // 归零
    if (k === 'h') { this.homeArm(); return true; }

    // 预设姿态
    if (k in this.PRESETS) { this.sendPreset(k); return true; }

    return false;
  }

  handleKeyUp(e) {
    const k = e.key.toLowerCase();
    if (['w', 'a', 's', 'd'].includes(k)) {
      this._heldKeys.delete(k);
      this._updateBaseMovement();
    }
  }

  // ─── UI 更新 ──────────────────────────────────

  _updateUI() {
    const $ = id => document.getElementById(id);

    // 线/角速度数值
    $('tp-lin-val') && ($('tp-lin-val').textContent = this.linearSpeed.toFixed(2));
    $('tp-ang-val') && ($('tp-ang-val').textContent = this.angularSpeed.toFixed(2));

    // 关节信息
    this._updateArmDisplay();

    // 启用状态样式
    const banner = $('teleop-banner');
    if (banner) {
      banner.className = 'teleop-banner ' + (this.enabled ? 'teleop-on' : 'teleop-off');
      banner.textContent = this.enabled ? '⌨ 键盘遥控已启用' : '⌨ 键盘遥控已禁用';
    }

    // 关节按钮高亮
    for (let i = 0; i < 7; i++) {
      const btn = $(`tp-j${i+1}`);
      if (btn) btn.classList.toggle('active', i === this.selectedJoint);
    }
  }

  _updateArmDisplay() {
    const $ = id => document.getElementById(id);
    const i = this.selectedJoint;
    $('tp-joint-label') && ($('tp-joint-label').textContent = `J${i+1}`);
    $('tp-joint-val')   && ($('tp-joint-val').textContent   = this.jointPositions[i].toFixed(3));
    $('tp-step-val')    && ($('tp-step-val').textContent    = this.jointStep.toFixed(2));

    const [lo, hi] = this.JOINT_LIMITS[i];
    const pct = ((this.jointPositions[i] - lo) / (hi - lo) * 100).toFixed(1);
    $('tp-joint-bar') && ($('tp-joint-bar').style.width = pct + '%');

    for (let j = 0; j < 7; j++) {
      const btn = document.getElementById(`tp-j${j+1}`);
      if (btn) btn.classList.toggle('active', j === i);
    }
  }

  _updateVelDisplay(lin, ang) {
    const $ = id => document.getElementById(id);
    // 更新底盘方向指示
    ['w','a','s','d'].forEach(k => {
      const btn = $(`tp-btn-${k}`);
      if (btn) btn.classList.toggle('pressing', this._heldKeys.has(k));
    });
  }
}
