/**
 * main.js - 应用入口与全局协调
 *
 * 负责：
 *  - ROS WebSocket 连接（自动重连）
 *  - 初始化各子模块（NavMap / OctoMapViewer / CameraView）
 *  - 全局事件处理（UI 按钮、配置、键盘）
 *  - 速度话题订阅（显示线速度/角速度）
 *  - 急停发布
 */

// ─── 全局应用对象 ───────────────────────────────────────
const App = {
  ros:       null,
  navMap:    null,
  octomap:   null,
  camera:    null,
  connected: false,

  config: {
    rosbridgeUrl:    'ws://localhost:9090',
    videoServerUrl:  'http://localhost:8080',
    baseFrame:       'base_footprint',
    octomapTopic:    '/octomap_point_cloud_centers'
  },

  /** 显示短暂通知 */
  showToast(msg, type = 'info', duration = 3000) {
    const el = document.getElementById('toast');
    if (!el) return;
    el.textContent = msg;
    el.className   = `toast toast-${type}`;
    el.classList.remove('hidden');
    clearTimeout(el._timer);
    el._timer = setTimeout(() => el.classList.add('hidden'), duration);
  },

  /** 设置话题指示灯（3 秒无数据自动变暗） */
  setTopicActive(elemId, active) {
    const el = document.getElementById(elemId);
    if (!el) return;
    if (active) {
      el.classList.add('active');
      clearTimeout(el._dimTimer);
      el._dimTimer = setTimeout(() => el.classList.remove('active'), 3000);
    } else {
      el.classList.remove('active');
    }
  }
};

// 暴露到 window，供其他模块访问
window.App = App;

// ─── 状态栏 ─────────────────────────────────────────────

function setConnectionStatus(state) {
  const el = document.getElementById('ros-status');
  if (!el) return;
  el.className = 'status-badge';
  if (state === 'connected') {
    el.classList.add('status-connected');
    el.textContent = '● 已连接';
  } else if (state === 'connecting') {
    el.classList.add('status-connecting', 'pulsing');
    el.textContent = '● 连接中...';
  } else {
    el.classList.add('status-disconnected');
    el.textContent = '● 未连接';
  }
}

// ─── ROS 连接 ───────────────────────────────────────────

let _reconnectTimer = null;

function connectROS() {
  if (App.ros) {
    try { App.ros.close(); } catch(e) {}
    App.ros = null;
  }

  clearTimeout(_reconnectTimer);
  setConnectionStatus('connecting');

  App.ros = new ROSLIB.Ros({ url: App.config.rosbridgeUrl });

  App.ros.on('connection', () => {
    console.log('[ROS] 连接成功:', App.config.rosbridgeUrl);
    App.connected = true;
    setConnectionStatus('connected');
    App.showToast('✅ rosbridge 连接成功', 'success');
    _onRosConnected();
  });

  App.ros.on('error', err => {
    console.error('[ROS] 连接错误:', err);
    setConnectionStatus('disconnected');
  });

  App.ros.on('close', () => {
    console.warn('[ROS] 连接断开，5 秒后重试...');
    App.connected = false;
    setConnectionStatus('disconnected');
    App.showToast('⚠ rosbridge 断开，重试中...', 'warn');
    _reconnectTimer = setTimeout(connectROS, 5000);
  });
}

/** ROS 连接成功后启动所有订阅 */
function _onRosConnected() {
  if (App.navMap)  App.navMap.ros  = App.ros;
  if (App.octomap) App.octomap.ros = App.ros;
  if (App.camera)  App.camera.ros  = App.ros;
  if (App.teleop)  App.teleop.ros  = App.ros;

  if (App.navMap)  App.navMap.connect();
  if (App.camera)  App.camera.connect(App.config.videoServerUrl);
  if (App.octomap) App.octomap.connect(App.config.octomapTopic);
  if (App.teleop)  App.teleop.connect();

  _subscribeVelocity();
}

// ─── 速度订阅 ───────────────────────────────────────────

let _velSub = null;

function _subscribeVelocity() {
  if (_velSub) { try { _velSub.unsubscribe(); } catch(e) {} }

  _velSub = new ROSLIB.Topic({
    ros: App.ros,
    name: '/x_bot/cmd_vel',
    messageType: 'geometry_msgs/Twist',
    throttle_rate: 200,
    queue_length: 1
  });

  _velSub.subscribe(msg => {
    const lin = msg.linear.x;
    const ang = msg.angular.z;

    const linBar  = document.getElementById('vel-linear');
    const angBar  = document.getElementById('vel-angular');
    const linTxt  = document.getElementById('vel-linear-val');
    const angTxt  = document.getElementById('vel-angular-val');
    const hdrVel  = document.getElementById('robot-vel');

    const linPct = Math.min(100, Math.abs(lin) / 0.5 * 100);
    const angPct = Math.min(100, Math.abs(ang) / 1.0 * 100);

    if (linBar) linBar.style.width = linPct + '%';
    if (angBar) angBar.style.width = angPct + '%';
    if (linTxt) linTxt.textContent = lin.toFixed(2)  + ' m/s';
    if (angTxt) angTxt.textContent = ang.toFixed(2)  + ' rad/s';
    if (hdrVel) hdrVel.textContent = `速度: ${lin.toFixed(2)}m/s ${ang.toFixed(2)}r/s`;
  });
}

// ─── 急停 ───────────────────────────────────────────────

function publishEStop() {
  if (!App.ros || !App.connected) {
    App.showToast('⚠ 未连接到 ROS', 'error');
    return;
  }
  const pub = new ROSLIB.Topic({
    ros: App.ros,
    name: '/x_bot/cmd_vel',
    messageType: 'geometry_msgs/Twist'
  });
  pub.publish(new ROSLIB.Message({
    linear:  { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  }));
  App.showToast('⛔ 急停已发送', 'error');
}

// ─── 配置持久化 ─────────────────────────────────────────

function loadConfig() {
  try {
    const saved = localStorage.getItem('robotsim-config');
    if (saved) Object.assign(App.config, JSON.parse(saved));
  } catch(e) {}

  const $v = (id, val) => {
    const el = document.getElementById(id);
    if (el) el.value = val;
  };
  $v('cfg-rosbridge',     App.config.rosbridgeUrl);
  $v('cfg-video-server',  App.config.videoServerUrl);
  $v('cfg-base-frame',    App.config.baseFrame);
  $v('cfg-octomap-topic', App.config.octomapTopic);
}

function saveConfig() {
  App.config.rosbridgeUrl    = (document.getElementById('cfg-rosbridge')?.value     || '').trim();
  App.config.videoServerUrl  = (document.getElementById('cfg-video-server')?.value  || '').trim();
  App.config.baseFrame       = (document.getElementById('cfg-base-frame')?.value    || '').trim();
  App.config.octomapTopic    = (document.getElementById('cfg-octomap-topic')?.value || '').trim();
  localStorage.setItem('robotsim-config', JSON.stringify(App.config));
}

// ─── UI 事件 ────────────────────────────────────────────

function bindUI() {
  const $ = id => document.getElementById(id);

  // 配置面板
  $('btn-config')?.addEventListener('click', () => {
    $('config-panel')?.classList.remove('hidden');
  });
  $('btn-close-config')?.addEventListener('click', () => {
    $('config-panel')?.classList.add('hidden');
  });
  $('config-panel')?.addEventListener('click', e => {
    if (e.target === $('config-panel')) $('config-panel')?.classList.add('hidden');
  });
  $('btn-apply-config')?.addEventListener('click', () => {
    saveConfig();
    $('config-panel')?.classList.add('hidden');
    connectROS();
  });

  // 急停
  $('btn-estop')?.addEventListener('click', publishEStop);

  // 遥控启用开关
  $('teleop-enable')?.addEventListener('change', e => {
    App.teleop?.setEnabled(e.target.checked);
  });

  // D-Pad 按钮（鼠标/触控）— 不依赖键盘启用状态，只要 ROS 已连接就可用
  ['w','a','s','d'].forEach(k => {
    const btn = $(`tp-btn-${k}`);
    if (!btn) return;
    const press = () => {
      if (!App.teleop) return;
      btn.classList.add('pressing');
      App.teleop._heldKeys.add(k);
      App.teleop._updateBaseMovement();
    };
    const release = () => {
      if (!App.teleop) return;
      btn.classList.remove('pressing');
      App.teleop._heldKeys.delete(k);
      App.teleop._updateBaseMovement();
    };
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('touchstart', press,   { passive: true });
    btn.addEventListener('mouseup',    release);
    btn.addEventListener('mouseleave', release);
    btn.addEventListener('touchend',   release);
  });
  $('tp-btn-stop')?.addEventListener('click', () => App.teleop?._stopBase());

  // 速度调整按钮
  $('tp-lin-down')?.addEventListener('click', () => { if (App.teleop) { App.teleop.handleKeyDown({key:'z'}); App.teleop._updateUI(); } });
  $('tp-lin-up')  ?.addEventListener('click', () => { if (App.teleop) { App.teleop.handleKeyDown({key:'x'}); App.teleop._updateUI(); } });
  $('tp-ang-down')?.addEventListener('click', () => { if (App.teleop) { App.teleop.handleKeyDown({key:'c'}); App.teleop._updateUI(); } });
  $('tp-ang-up')  ?.addEventListener('click', () => { if (App.teleop) { App.teleop.handleKeyDown({key:'v'}); App.teleop._updateUI(); } });

  // 关节选择按钮
  for (let i = 1; i <= 7; i++) {
    $(`tp-j${i}`)?.addEventListener('click', () => {
      if (App.teleop) { App.teleop.selectedJoint = i - 1; App.teleop._updateArmDisplay(); }
    });
  }

  // 关节角度调整
  $('tp-joint-dec')?.addEventListener('click', () => App.teleop?.adjustJoint(-App.teleop.jointStep));
  $('tp-joint-inc')?.addEventListener('click', () => App.teleop?.adjustJoint( App.teleop.jointStep));

  // 步长调整
  $('tp-step-down')?.addEventListener('click', () => { if (App.teleop) { App.teleop.handleKeyDown({key:'['}); App.teleop._updateArmDisplay(); } });
  $('tp-step-up')  ?.addEventListener('click', () => { if (App.teleop) { App.teleop.handleKeyDown({key:']'}); App.teleop._updateArmDisplay(); } });

  // 夹爪：由 teleop.js 的 _flashPresetBtn 统一处理高亮+圆圈切换
  $('tp-gripper-open')?.addEventListener('click', () => App.teleop?.openGripper());
  $('tp-gripper-close')?.addEventListener('click', () => App.teleop?.closeGripper());

  // 预设姿态：由 teleop.js 的 _flashPresetBtn 统一处理高亮
  $('tp-pose-i')?.addEventListener('click', () => App.teleop?.sendPreset('i'));
  $('tp-pose-k')?.addEventListener('click', () => App.teleop?.sendPreset('k'));
  $('tp-pose-l')?.addEventListener('click', () => App.teleop?.sendPreset('l'));
  $('tp-home')  ?.addEventListener('click', () => App.teleop?.homeArm());

  // 设置导航目标（2D Nav Goal）
  const btnGoal = $('btn-set-goal');
  btnGoal?.addEventListener('click', () => {
    if (!App.navMap) return;
    const nowActive = !App.navMap.goalMode;
    App.navMap.setGoalMode(nowActive);
    btnGoal.classList.toggle('active', nowActive);
    if (nowActive) App.showToast('🎯 目标模式：点击拖动方向后释放鼠标', 'info');
  });

  // 取消导航
  $('btn-cancel-goal')?.addEventListener('click', () => {
    if (!App.navMap) return;
    App.navMap.cancelGoal();
    App.navMap.setGoalMode(false);
    btnGoal?.classList.remove('active');
    $('goal-x').textContent   = '--';
    $('goal-y').textContent   = '--';
    $('goal-yaw').textContent = '--';
  });

  // 适应地图视图
  $('btn-fit-map')?.addEventListener('click', () => {
    App.navMap?._fitToView();
  });

  // 显示/隐藏激光雷达
  $('btn-toggle-scan')?.addEventListener('click', function() {
    if (!App.navMap) return;
    const on = App.navMap.toggleScan();
    this.textContent = on ? '隐藏激光' : '显示激光';
  });

  // OctoMap 自动更新
  $('octomap-auto-update')?.addEventListener('change', e => {
    App.octomap?.setAutoUpdate(e.target.checked);
  });

  // OctoMap 稀疏体素 / 点云切换
  $('octomap-voxel-mode')?.addEventListener('change', e => {
    App.octomap?.setVoxelMode(e.target.checked);
  });

  // OctoMap 高度着色
  $('octomap-height-color')?.addEventListener('change', e => {
    App.octomap?.setHeightColor(e.target.checked);
  });

  // 重置 3D 视角
  $('btn-reset-3d-view')?.addEventListener('click', () => {
    App.octomap?.resetView();
  });

  // 清除 OctoMap 体素
  $('btn-clear-octomap')?.addEventListener('click', () => {
    App.octomap?.clearVoxels();
  });

  // 重置所有视图
  $('btn-reset-view')?.addEventListener('click', () => {
    App.navMap?._fitToView();
    App.octomap?.resetView();
  });

  // 保存地图提示
  $('btn-save-map')?.addEventListener('click', () => {
    App.showToast(
      '💾 终端执行: ros2 run nav2_map_server map_saver_cli -f ~/map',
      'info', 6000
    );
  });

  // 键盘快捷键（teleop 优先消费，否则导航快捷键）
  document.addEventListener('keydown', e => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    // Teleop 优先消费
    if (App.teleop?.handleKeyDown(e)) return;

    switch(e.key) {
      case 'Escape':
        if (App.navMap?.goalMode) {
          App.navMap.setGoalMode(false);
          btnGoal?.classList.remove('active');
        }
        break;
      case 'g': case 'G': btnGoal?.click(); break;
      case ' ':  e.preventDefault(); publishEStop(); break;
      case 'f': case 'F': App.navMap?._fitToView(); break;
      case 'r': case 'R': App.octomap?.resetView(); break;
    }
  });

  document.addEventListener('keyup', e => {
    App.teleop?.handleKeyUp(e);
  });
}

// ─── 入口 ────────────────────────────────────────────────

document.addEventListener('DOMContentLoaded', () => {
  console.log('[App] 🤖 RobotSim Dashboard 启动');

  // 初始化可调整布局
  window.App = window.App || {};
  window.App.resizable = new ResizableLayout();

  // 加载配置
  loadConfig();

  // 初始化子模块（ros 稍后注入）
  App.navMap  = new NavMap('nav-canvas', null);
  App.octomap = new OctoMapViewer('octomap-container', null);
  App.camera  = new CameraView(null);
  App.teleop  = new TeleopController(null);

  // 激活导航地图的交互（需在 DOM 挂载后）
  App.navMap.setupInteraction();

  // 绑定 UI
  bindUI();

  // 连接 ROS
  connectROS();

  // 快捷键提示
  console.log('[App] 快捷键: G=设置目标 | Esc=取消目标 | Space=急停 | F=适应地图 | R=重置3D视角');
});
