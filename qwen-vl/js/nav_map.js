/**
 * nav_map.js - 2D 导航地图模块
 *
 * 功能：
 *  - 订阅 /map 话题并渲染 OccupancyGrid
 *  - 通过 /tf, /tf_static 跟踪机器人位置
 *  - 订阅 /plan 显示导航路径
 *  - 订阅 /x_bot/scan 显示激光雷达
 *  - 鼠标交互：平移/缩放/设置 Nav2 目标（类 RViz 2D Nav Goal）
 *  - 发布 /goal_pose 话题
 */

class NavMap {
  constructor(canvasId, ros) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.ros = ros;

    // 地图数据
    this.mapInfo = null;
    this.mapBitmap = null;
    this.hasInitialView = false;

    // 视图变换
    this.viewScale = 2.0;
    this.panX = 0;
    this.panY = 0;

    // 机器人状态
    this.tfTree = {};
    this.robotPose = null;   // { x, y, yaw } 在 map 坐标系 (m)

    // 导航路径
    this.navPath = [];

    // 激光雷达点
    this.scanPoints = [];
    this.showScan = false;

    // 目标设置
    this.goalMode = false;
    this.goalStart = null;    // { cx, cy, mx, my } 鼠标按下时
    this.goalCurrent = null;  // { cx, cy } 当前鼠标位置
    this.lastGoal = null;     // { x, y, yaw }

    // 平移
    this.isPanning = false;
    this.panStart = null;

    // 发布者
    this.goalPub = null;

    this._subscribers = [];
    this._init();
  }

  _init() {
    this._resizeCanvas();
    window.addEventListener('resize', () => this._resizeCanvas());
    this._startRenderLoop();
  }

  // ─── 话题 ───────────────────────────────────────────

  connect() {
    if (!this.ros) { console.warn('[NavMap] ros 未设置，跳过连接'); return; }
    this._subscribers.forEach(s => { try { s.unsubscribe(); } catch(e) {} });
    this._subscribers = [];
    // 重建发布者（ros 实例更新后需要重建）
    this._setupPublishers();

    // /map
    const mapSub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
      throttle_rate: 3000,
      queue_length: 1
    });
    mapSub.subscribe(msg => {
      this._updateMap(msg);
      window.App && window.App.setTopicActive('topic-map', true);
    });
    this._subscribers.push(mapSub);

    // /tf
    const tfSub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/tf',
      messageType: 'tf2_msgs/TFMessage',
      throttle_rate: 50
    });
    tfSub.subscribe(msg => {
      msg.transforms.forEach(t => { this.tfTree[t.child_frame_id] = t; });
      window.App && window.App.setTopicActive('topic-tf', true);
    });
    this._subscribers.push(tfSub);

    // /tf_static
    const tfStaticSub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/tf_static',
      messageType: 'tf2_msgs/TFMessage'
    });
    tfStaticSub.subscribe(msg => {
      msg.transforms.forEach(t => { this.tfTree[t.child_frame_id] = t; });
    });
    this._subscribers.push(tfStaticSub);

    // /plan (Nav2 全局路径)
    const planSub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/plan',
      messageType: 'nav_msgs/Path',
      throttle_rate: 500,
      queue_length: 1
    });
    planSub.subscribe(msg => {
      this.navPath = msg.poses.map(p => ({
        x: p.pose.position.x,
        y: p.pose.position.y
      }));
    });
    this._subscribers.push(planSub);

    // /x_bot/scan
    const scanSub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/x_bot/scan',
      messageType: 'sensor_msgs/LaserScan',
      throttle_rate: 200,
      queue_length: 1
    });
    scanSub.subscribe(msg => {
      this._updateScan(msg);
      window.App && window.App.setTopicActive('topic-scan', true);
    });
    this._subscribers.push(scanSub);
  }

  _setupPublishers() {
    this.goalPub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/goal_pose',
      messageType: 'geometry_msgs/PoseStamped'
    });
    this.cancelPub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/goal_pose',
      messageType: 'geometry_msgs/PoseStamped'
    });
  }

  // ─── 地图数据处理 ─────────────────────────────────

  _decodeData(data) {
    if (typeof data === 'string') {
      // rosbridge 以 base64 传输大数组
      const bin = atob(data);
      const arr = new Int8Array(bin.length);
      for (let i = 0; i < bin.length; i++) arr[i] = bin.charCodeAt(i);
      return arr;
    }
    return new Int8Array(data);
  }

  _updateMap(msg) {
    this.mapInfo = msg.info;
    const W = msg.info.width;
    const H = msg.info.height;
    const data = this._decodeData(msg.data);

    const imgData = new ImageData(W, H);
    for (let row = 0; row < H; row++) {
      for (let col = 0; col < W; col++) {
        const srcIdx = row * W + col;
        // ROS 地图 row=0 在底部，Canvas y=0 在顶部 → 翻转 Y
        const dstRow = H - 1 - row;
        const dstIdx = (dstRow * W + col) * 4;
        const v = data[srcIdx];
        let r, g, b;
        if (v === -1) {
          r = g = b = 128;   // 未知: 中灰
        } else if (v === 0) {
          r = g = b = 220;   // 空闲: 浅灰
        } else {
          const d = Math.max(0, Math.round(220 - v * 2.2));
          r = g = b = d;     // 占用: 深色
        }
        imgData.data[dstIdx]     = r;
        imgData.data[dstIdx + 1] = g;
        imgData.data[dstIdx + 2] = b;
        imgData.data[dstIdx + 3] = 255;
      }
    }

    createImageBitmap(imgData).then(bmp => {
      this.mapBitmap = bmp;
      if (!this.hasInitialView) {
        this._fitToView();
        this.hasInitialView = true;
      }
      const overlay = document.getElementById('map-overlay');
      if (overlay) overlay.classList.add('hidden');
    });
  }

  _updateScan(msg) {
    if (!this.showScan) return;
    const robot = this._getRobotPose();
    if (!robot) return;

    const pts = [];
    const angle_min = msg.angle_min;
    const angle_increment = msg.angle_increment;
    const ranges = msg.ranges;

    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (!isFinite(r) || r < msg.range_min || r > msg.range_max) continue;
      const angle = angle_min + i * angle_increment + robot.yaw;
      pts.push({
        x: robot.x + r * Math.cos(angle),
        y: robot.y + r * Math.sin(angle)
      });
    }
    this.scanPoints = pts;
  }

  // ─── TF 变换 ──────────────────────────────────────

  _qMul(a, b) {
    return {
      w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
      x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
      y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
      z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
  }

  _qRotVec(q, v) {
    const qv = this._qMul(q, { x: v.x, y: v.y, z: v.z || 0, w: 0 });
    const r = this._qMul(qv, { x: -q.x, y: -q.y, z: -q.z, w: q.w });
    return { x: r.x, y: r.y, z: r.z };
  }

  _qToYaw(q) {
    return Math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
  }

  _getRobotPose() {
    // 查找 base_footprint → odom → map 链
    const baseFrame = (window.App && window.App.config.baseFrame) || 'base_footprint';
    const bfTf = this.tfTree[baseFrame];
    if (!bfTf) return null;

    const bfTrans = bfTf.transform.translation;
    const bfRot   = bfTf.transform.rotation;
    const parent  = bfTf.header.frame_id;

    if (parent === 'map') {
      return { x: bfTrans.x, y: bfTrans.y, yaw: this._qToYaw(bfRot) };
    }

    // 尝试 parent -> map
    const parentTf = this.tfTree[parent];
    if (!parentTf) return null;
    if (parentTf.header.frame_id !== 'map') return null;

    const pTrans = parentTf.transform.translation;
    const pRot   = parentTf.transform.rotation;

    // 将 base_footprint 位置变换到 map 坐标系
    const rotated = this._qRotVec(pRot, bfTrans);
    const worldPos = {
      x: pTrans.x + rotated.x,
      y: pTrans.y + rotated.y,
      z: pTrans.z + rotated.z
    };
    const worldRot = this._qMul(pRot, bfRot);

    return { x: worldPos.x, y: worldPos.y, yaw: this._qToYaw(worldRot) };
  }

  // ─── 坐标变换 ─────────────────────────────────────

  // 地图坐标 (m) → Canvas 坐标 (px)
  _m2c(mx, my) {
    if (!this.mapInfo) return { x: 0, y: 0 };
    const orig = this.mapInfo.origin.position;
    const res  = this.mapInfo.resolution;
    const H    = this.mapInfo.height;
    const col  = (mx - orig.x) / res;
    const rowB = (my - orig.y) / res;
    const rowT = H - 1 - rowB;
    return {
      x: col  * this.viewScale + this.panX,
      y: rowT * this.viewScale + this.panY
    };
  }

  // Canvas 坐标 (px) → 地图坐标 (m)
  _c2m(cx, cy) {
    if (!this.mapInfo) return { x: 0, y: 0 };
    const orig = this.mapInfo.origin.position;
    const res  = this.mapInfo.resolution;
    const H    = this.mapInfo.height;
    const col  = (cx - this.panX) / this.viewScale;
    const rowT = (cy - this.panY) / this.viewScale;
    const rowB = H - 1 - rowT;
    return {
      x: col  * res + orig.x,
      y: rowB * res + orig.y
    };
  }

  _fitToView() {
    if (!this.mapInfo) return;
    const W = this.canvas.width;
    const H = this.canvas.height;
    const scaleX = W / this.mapInfo.width;
    const scaleY = H / this.mapInfo.height;
    this.viewScale = Math.min(scaleX, scaleY) * 0.9;
    this.panX = (W - this.mapInfo.width  * this.viewScale) / 2;
    this.panY = (H - this.mapInfo.height * this.viewScale) / 2;
  }

  // ─── 发布 Nav2 目标 ───────────────────────────────

  publishGoal(x, y, yaw) {
    if (!this.goalPub || !this.ros) {
      App.showToast('⚠ ROS 未连接，无法发送目标', 'error');
      return;
    }
    const hz = yaw / 2;
    const msg = new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { secs: 0, nsecs: 0 } },
      pose: {
        position:    { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: Math.sin(hz), w: Math.cos(hz) }
      }
    });
    this.goalPub.publish(msg);
    this.lastGoal = { x, y, yaw };

    // 更新 UI
    const fmt = v => v.toFixed(2);
    const el = id => document.getElementById(id);
    if (el('goal-x'))   el('goal-x').textContent   = fmt(x);
    if (el('goal-y'))   el('goal-y').textContent   = fmt(y);
    if (el('goal-yaw')) el('goal-yaw').textContent = (yaw * 180 / Math.PI).toFixed(1);
    window.App && window.App.showToast(`🎯 目标已发送 (${fmt(x)}, ${fmt(y)})`, 'success');
  }

  cancelGoal() {
    this.lastGoal = null;
    this.navPath = [];
    window.App && window.App.showToast('✕ 导航已取消', 'warn');
  }

  // ─── 交互 ─────────────────────────────────────────

  _resizeCanvas() {
    const container = this.canvas.parentElement;
    if (!container) return;
    const r = container.getBoundingClientRect();
    if (r.width > 0 && r.height > 0) {
      this.canvas.width  = r.width;
      this.canvas.height = r.height;
    }
  }

  setupInteraction() {
    const c = this.canvas;

    // 滚轮缩放（以鼠标为中心）
    c.addEventListener('wheel', e => {
      e.preventDefault();
      const r = c.getBoundingClientRect();
      const mx = e.clientX - r.left;
      const my = e.clientY - r.top;
      const zoom = e.deltaY < 0 ? 1.15 : 0.87;
      this.panX = mx - (mx - this.panX) * zoom;
      this.panY = my - (my - this.panY) * zoom;
      this.viewScale *= zoom;
      this._updateScaleBadge();
    }, { passive: false });

    // 鼠标按下
    c.addEventListener('mousedown', e => {
      const r = c.getBoundingClientRect();
      const cx = e.clientX - r.left;
      const cy = e.clientY - r.top;
      const mp = this._c2m(cx, cy);

      if (this.goalMode && e.button === 0) {
        this.goalStart   = { cx, cy, mx: mp.x, my: mp.y };
        this.goalCurrent = { cx, cy };
        c.style.cursor = 'crosshair';
      } else if (e.button === 0) {
        this.isPanning = true;
        this.panStart = { cx, cy, px: this.panX, py: this.panY };
        c.style.cursor = 'grabbing';
      }
    });

    // 鼠标移动
    c.addEventListener('mousemove', e => {
      const r = c.getBoundingClientRect();
      const cx = e.clientX - r.left;
      const cy = e.clientY - r.top;

      if (this.goalStart) {
        this.goalCurrent = { cx, cy };
      } else if (this.isPanning && this.panStart) {
        this.panX = this.panStart.px + cx - this.panStart.cx;
        this.panY = this.panStart.py + cy - this.panStart.cy;
      }
    });

    // 鼠标松开
    c.addEventListener('mouseup', e => {
      const r = c.getBoundingClientRect();
      const cx = e.clientX - r.left;
      const cy = e.clientY - r.top;

      if (this.goalStart && this.goalMode) {
        const dx = cx - this.goalStart.cx;
        const dy = cy - this.goalStart.cy;
        // 拖动方向决定朝向；短拖动则使用 0 或机器人当前朝向
        const dist = Math.sqrt(dx * dx + dy * dy);
        let yaw;
        if (dist < 8) {
          yaw = this.robotPose ? this.robotPose.yaw : 0;
        } else {
          yaw = Math.atan2(-dy, dx);  // canvas y 轴翻转
        }
        this.publishGoal(this.goalStart.mx, this.goalStart.my, yaw);
        this.goalStart   = null;
        this.goalCurrent = null;
      }

      if (this.isPanning) {
        this.isPanning = false;
        this.panStart  = null;
        c.style.cursor = this.goalMode ? 'crosshair' : 'default';
      }
    });

    // 右键取消目标设置
    c.addEventListener('contextmenu', e => {
      e.preventDefault();
      if (this.goalMode) {
        this.goalStart   = null;
        this.goalCurrent = null;
      }
    });
  }

  setGoalMode(active) {
    this.goalMode = active;
    this.canvas.style.cursor = active ? 'crosshair' : 'default';
    if (!active) {
      this.goalStart   = null;
      this.goalCurrent = null;
    }
  }

  toggleScan() {
    this.showScan = !this.showScan;
    if (!this.showScan) this.scanPoints = [];
    return this.showScan;
  }

  _updateScaleBadge() {
    const el = document.getElementById('map-scale');
    if (el && this.mapInfo) {
      const mPerPx = this.mapInfo.resolution / this.viewScale;
      el.textContent = `${mPerPx.toFixed(3)} m/px`;
    }
  }

  // ─── 渲染 ─────────────────────────────────────────

  _startRenderLoop() {
    const loop = () => {
      this._draw();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  _draw() {
    const ctx = this.ctx;
    const W   = this.canvas.width;
    const H   = this.canvas.height;

    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, W, H);

    if (!this.mapBitmap) {
      ctx.fillStyle = '#475569';
      ctx.font = '14px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('等待地图数据... (检查 rosbridge 连接)', W / 2, H / 2);
      return;
    }

    // ── 绘制地图 ──
    ctx.save();
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(
      this.mapBitmap,
      this.panX, this.panY,
      this.mapInfo.width  * this.viewScale,
      this.mapInfo.height * this.viewScale
    );
    ctx.restore();

    // ── 绘制激光雷达点 ──
    if (this.showScan && this.scanPoints.length > 0) {
      ctx.fillStyle = 'rgba(255, 200, 0, 0.7)';
      for (const pt of this.scanPoints) {
        const cp = this._m2c(pt.x, pt.y);
        ctx.fillRect(cp.x - 1, cp.y - 1, 3, 3);
      }
    }

    // ── 绘制导航路径 ──
    if (this.navPath.length > 1) {
      ctx.beginPath();
      ctx.strokeStyle = '#00ff88';
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 3]);
      let first = true;
      for (const pt of this.navPath) {
        const cp = this._m2c(pt.x, pt.y);
        first ? ctx.moveTo(cp.x, cp.y) : ctx.lineTo(cp.x, cp.y);
        first = false;
      }
      ctx.stroke();
      ctx.setLineDash([]);

      // 终点标记
      const last = this.navPath[this.navPath.length - 1];
      const lcp  = this._m2c(last.x, last.y);
      ctx.beginPath();
      ctx.arc(lcp.x, lcp.y, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#00ff88';
      ctx.fill();
    }

    // ── 绘制上次目标 ──
    if (this.lastGoal) {
      const gcp = this._m2c(this.lastGoal.x, this.lastGoal.y);
      const yaw  = this.lastGoal.yaw;
      const sz   = 12;

      ctx.save();
      ctx.translate(gcp.x, gcp.y);

      ctx.beginPath();
      ctx.arc(0, 0, sz, 0, Math.PI * 2);
      ctx.strokeStyle = '#ff6b00';
      ctx.lineWidth = 2;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);

      // 方向箭头
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(sz * 1.8 * Math.cos(-yaw), sz * 1.8 * Math.sin(-yaw));
      ctx.strokeStyle = '#ff6b00';
      ctx.lineWidth = 2;
      ctx.stroke();

      ctx.restore();
    }

    // ── 绘制机器人 ──
    this.robotPose = this._getRobotPose();
    if (this.robotPose) {
      const rp  = this._m2c(this.robotPose.x, this.robotPose.y);
      const yaw = this.robotPose.yaw;
      const sz  = Math.max(10, this.viewScale * 2.5);

      ctx.save();
      ctx.translate(rp.x, rp.y);
      ctx.rotate(-yaw);  // canvas y 轴翻转，取反

      // 机器人圆体
      ctx.beginPath();
      ctx.arc(0, 0, sz, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(37,99,235,0.85)';
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.stroke();

      // 方向指示线
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(sz * 1.6, 0);
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2.5;
      ctx.stroke();

      // 箭头头部
      ctx.beginPath();
      ctx.moveTo(sz * 1.6, 0);
      ctx.lineTo(sz * 1.2, -sz * 0.4);
      ctx.lineTo(sz * 1.2,  sz * 0.4);
      ctx.closePath();
      ctx.fillStyle = '#fff';
      ctx.fill();

      ctx.restore();

      // 更新姿态 UI
      const fmt = v => v.toFixed(2);
      const el  = id => document.getElementById(id);
      if (el('pose-x'))   el('pose-x').textContent   = fmt(this.robotPose.x);
      if (el('pose-y'))   el('pose-y').textContent   = fmt(this.robotPose.y);
      if (el('pose-yaw')) el('pose-yaw').textContent = (yaw * 180 / Math.PI).toFixed(1);
      const hdr = document.getElementById('robot-pos');
      if (hdr) hdr.textContent = `位置: (${fmt(this.robotPose.x)}, ${fmt(this.robotPose.y)})`;
    }

    // ── 绘制目标设置预览箭头 ──
    if (this.goalStart && this.goalCurrent) {
      const sx = this.goalStart.cx;
      const sy = this.goalStart.cy;
      const ex = this.goalCurrent.cx;
      const ey = this.goalCurrent.cy;
      const dx = ex - sx;
      const dy = ey - sy;
      const dist = Math.sqrt(dx * dx + dy * dy);

      // 目标位置圆
      ctx.beginPath();
      ctx.arc(sx, sy, 10, 0, Math.PI * 2);
      ctx.strokeStyle = '#ff6b00';
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.setLineDash([]);

      // 方向箭头
      if (dist > 8) {
        const angle = Math.atan2(dy, dx);
        const ahLen = 16;

        ctx.beginPath();
        ctx.moveTo(sx, sy);
        ctx.lineTo(ex, ey);
        ctx.strokeStyle = 'rgba(255,107,0,0.9)';
        ctx.lineWidth = 3;
        ctx.stroke();

        // 箭头头
        ctx.beginPath();
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex - ahLen * Math.cos(angle - 0.4), ey - ahLen * Math.sin(angle - 0.4));
        ctx.lineTo(ex - ahLen * Math.cos(angle + 0.4), ey - ahLen * Math.sin(angle + 0.4));
        ctx.closePath();
        ctx.fillStyle = '#ff6b00';
        ctx.fill();
      }
    }

    // ── 目标模式提示条 ──
    if (this.goalMode) {
      ctx.fillStyle = 'rgba(255,107,0,0.15)';
      ctx.fillRect(0, 0, W, 30);
      ctx.fillStyle = 'rgba(255,107,0,0.6)';
      ctx.fillRect(0, 0, W, 2);
      ctx.fillStyle = '#ff9955';
      ctx.font = 'bold 13px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('🎯  点击拖动以设置导航目标方向，右键取消', W / 2, 20);
    }

    this._updateScaleBadge();
  }
}
