/**
 * camera.js - Gazebo 相机视图模块
 *
 * 显示策略（优先级）：
 *  1. web_video_server MJPEG 流（效率最高，需要 web_video_server 节点）
 *  2. rosbridge 原始 sensor_msgs/Image（rgb8/bgr8，直接订阅，无需额外节点）
 *
 * 相机话题：/x_bot/camera_left/image_raw（rgb8，640×640，~8fps）
 */

class CameraView {
  constructor(ros) {
    this.ros            = ros;
    this.imgEl          = document.getElementById('camera-img');
    this.canvasEl       = document.getElementById('camera-canvas');
    this.overlayEl      = document.getElementById('camera-overlay');
    this.fpsEl          = document.getElementById('camera-fps');
    this.topicSelect    = document.getElementById('camera-topic-select');

    this.currentTopic   = '/x_bot/camera_left/image_raw';
    this.videoServerUrl = 'http://localhost:8080';
    this._rosSub        = null;
    this._mode          = 'none';
    this._frameCount    = 0;
    this._lastFpsTime   = Date.now();
    this._ctx           = this.canvasEl?.getContext('2d') ?? null;

    if (this.topicSelect) {
      this.topicSelect.addEventListener('change', e => this.switchTopic(e.target.value));
    }
  }

  // ─── 对外接口 ──────────────────────────────────────

  connect(videoServerUrl) {
    if (videoServerUrl) this.videoServerUrl = videoServerUrl;
    this._tryMjpeg();
  }

  switchTopic(topic) {
    this.currentTopic = topic;
    this._tryMjpeg();
  }

  // ─── 方案 1：web_video_server MJPEG ───────────────

  _tryMjpeg() {
    this._stopSub();
    if (!this.imgEl) { this._tryRaw(); return; }

    this._mode = 'mjpeg';
    this._showImg();
    this._setOverlay('连接 web_video_server...');

    const url = `${this.videoServerUrl}/stream` +
                `?topic=${encodeURIComponent(this.currentTopic)}` +
                `&type=mjpeg&quality=75`;
    this.imgEl.src = '';

    // 5 秒超时 → 切换到原始图像模式
    const timer = setTimeout(() => {
      console.warn('[Camera] web_video_server 超时，切换到原始图像模式');
      this._tryRaw();
    }, 5000);

    this.imgEl.onload = () => {
      clearTimeout(timer);
      this._hideOverlay();
      this._mode = 'mjpeg';
      window.App?.setTopicActive('topic-camera', true);
      // MJPEG 流持续刷新，用定时器估算 fps
      clearInterval(this._fpsTimer);
      this._fpsTimer = setInterval(() => {
        if (this._mode !== 'mjpeg') { clearInterval(this._fpsTimer); return; }
        if (this.imgEl?.complete && this.imgEl?.naturalWidth > 0) {
          this._countFrame();
        }
      }, 80);
    };

    this.imgEl.onerror = () => {
      clearTimeout(timer);
      console.warn('[Camera] web_video_server 不可用，切换到原始图像模式');
      this._tryRaw();
    };

    this.imgEl.src = url;
  }

  // ─── 方案 2：rosbridge 原始 sensor_msgs/Image ─────

  _tryRaw() {
    this._stopSub();
    this._mode = 'raw';
    this._showCanvas();
    this._setOverlay('订阅原始相机图像...');

    let received = false;

    this._rosSub = new ROSLIB.Topic({
      ros:           this.ros,
      name:          this.currentTopic,
      messageType:   'sensor_msgs/Image',
      throttle_rate: 150,   // ~6fps（localhost 带宽充足）
      queue_length:  1
    });

    this._rosSub.subscribe(msg => {
      if (!received) {
        received = true;
        this._hideOverlay();
        console.log(`[Camera] rosbridge 原始图像 ` +
                    `(${msg.encoding}, ${msg.width}×${msg.height})`);
      }
      this._decodeRaw(msg);
    });

    // 5 秒无数据 → 显示提示
    const watchdog = setTimeout(() => {
      if (!received) this._setOverlay('⚠ 相机话题无数据，请检查 Gazebo 是否运行');
    }, 5000);

    this._watchdog = watchdog;
  }

  // ─── 原始图像解码（rgb8 / bgr8 / rgba8 / bgra8）──

  _decodeRaw(msg) {
    if (!this._ctx || !this.canvasEl) return;

    const W        = msg.width;
    const H        = msg.height;
    const step     = msg.step;
    const encoding = msg.encoding ?? 'rgb8';

    // 解码 base64 → Uint8Array
    let raw;
    if (typeof msg.data === 'string') {
      const bin = atob(msg.data);
      raw = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) raw[i] = bin.charCodeAt(i);
    } else {
      raw = new Uint8Array(msg.data);
    }

    const isBGR    = encoding === 'bgr8'  || encoding === 'bgra8';
    const hasAlpha = encoding === 'rgba8' || encoding === 'bgra8';
    const ch       = hasAlpha ? 4 : 3;

    if (this.canvasEl.width !== W)  this.canvasEl.width  = W;
    if (this.canvasEl.height !== H) this.canvasEl.height = H;

    const imgData = this._ctx.createImageData(W, H);
    const pix     = imgData.data;

    for (let y = 0; y < H; y++) {
      for (let x = 0; x < W; x++) {
        const s = y * step + x * ch;
        const d = (y * W + x) * 4;
        if (isBGR) {
          pix[d]     = raw[s + 2];
          pix[d + 1] = raw[s + 1];
          pix[d + 2] = raw[s];
        } else {
          pix[d]     = raw[s];
          pix[d + 1] = raw[s + 1];
          pix[d + 2] = raw[s + 2];
        }
        pix[d + 3] = hasAlpha ? raw[s + 3] : 255;
      }
    }

    this._ctx.putImageData(imgData, 0, 0);
    this._countFrame();
    window.App?.setTopicActive('topic-camera', true);
  }

  // ─── FPS 计数 ─────────────────────────────────────

  _countFrame() {
    this._frameCount++;
    const now  = Date.now();
    const diff = now - this._lastFpsTime;
    if (diff >= 1500) {
      if (this.fpsEl)
        this.fpsEl.textContent = (this._frameCount * 1000 / diff).toFixed(1) + ' fps';
      this._frameCount  = 0;
      this._lastFpsTime = now;
    }
  }

  // ─── 辅助 ─────────────────────────────────────────

  _stopSub() {
    clearInterval(this._fpsTimer);
    clearTimeout(this._watchdog);
    if (this._rosSub) {
      try { this._rosSub.unsubscribe(); } catch(e) {}
      this._rosSub = null;
    }
    if (this.imgEl) { this.imgEl.onload = null; this.imgEl.onerror = null; }
  }

  _showImg() {
    this.imgEl?.classList.remove('hidden');
    this.canvasEl?.classList.add('hidden');
  }

  _showCanvas() {
    this.imgEl?.classList.add('hidden');
    this.canvasEl?.classList.remove('hidden');
  }

  _setOverlay(text) {
    if (!this.overlayEl) return;
    this.overlayEl.classList.remove('hidden');
    const span = this.overlayEl.querySelector('span');
    if (span) span.textContent = text;
    else this.overlayEl.textContent = text;
  }

  _hideOverlay() {
    this.overlayEl?.classList.add('hidden');
  }

  disconnect() {
    this._stopSub();
    if (this.imgEl) this.imgEl.src = '';
    this._setOverlay('已断开');
    if (this.fpsEl) this.fpsEl.textContent = '-- fps';
  }
}
