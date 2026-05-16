/**
 * octomap.js - 3D OctoMap 可视化模块（Three.js）
 *
 * 数据来源（可切换）：
 *   主选：/octomap_point_cloud_centers  (sensor_msgs/PointCloud2)
 *         → 完整点云，保留 YOLOE 原始 RGB 颜色，14k+ 点
 *   备选：/occupied_cells_vis_array     (visualization_msgs/MarkerArray)
 *         → 仅含少量粗粒度体素
 *
 * 颜色模式：
 *   - 原始颜色：从 PointCloud2 rgb 字段解码（packed uint32: 0x00RRGGBB）
 *   - 高度着色：基于 Z 值彩虹映射（蓝低→红高）
 *
 * 渲染方式：THREE.Points（BufferGeometry）高效渲染大规模点云
 */

class OctoMapViewer {
  constructor(containerId, ros) {
    this.container   = document.getElementById(containerId);
    this.ros         = ros;
    this.autoUpdate  = true;
    this.heightColor = false;   // 默认使用原始颜色
    this._sub        = null;
    this._topicName  = '/octomap_point_cloud_centers';
    this._msgType    = 'sensor_msgs/PointCloud2';
    this._pointCloud = null;    // THREE.Points
    this._totalVoxels = 0;
    this._lastUpdate  = null;
    this._resolution  = 0.10;   // 默认体素尺寸 (m)

    this._initThreeJS();
    this._addSceneObjects();
    this._startRenderLoop();

    const ro = new ResizeObserver(() => this._onResize());
    ro.observe(this.container);
  }

  // ─── Three.js 初始化 ─────────────────────────────

  _initThreeJS() {
    const W = this.container.clientWidth  || 800;
    const H = this.container.clientHeight || 280;

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(W, H);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0x060810);
    this.container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.FogExp2(0x060810, 0.025);

    // 相机：Z 轴朝上（ROS 惯例）
    this.camera = new THREE.PerspectiveCamera(55, W / H, 0.02, 200);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(8, 8, 5);
    this.camera.lookAt(0, 0, 1);

    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0, 1);
    this.controls.enableDamping   = true;
    this.controls.dampingFactor   = 0.08;
    this.controls.minDistance     = 0.3;
    this.controls.maxDistance     = 80;
    this.controls.screenSpacePanning = false;
    this.controls.update();
  }

  _addSceneObjects() {
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.8));
    const dir = new THREE.DirectionalLight(0xffffff, 0.5);
    dir.position.set(5, 5, 10);
    this.scene.add(dir);

    // 坐标轴
    this.scene.add(new THREE.AxesHelper(1.5));

    // XY 地面网格
    const grid = new THREE.GridHelper(20, 20, 0x1a2840, 0x0d1520);
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);
  }

  // ─── ROS 订阅 ─────────────────────────────────────

  connect(topicName) {
    if (!this.ros) { console.warn('[OctoMap] ros 未设置'); return; }
    if (this._sub) { try { this._sub.unsubscribe(); } catch(e) {} }

    // 根据话题名推断消息类型
    if (topicName && topicName.includes('point_cloud')) {
      this._topicName = topicName;
      this._msgType   = 'sensor_msgs/PointCloud2';
    } else if (topicName && topicName.includes('vis_array')) {
      this._topicName = topicName;
      this._msgType   = 'visualization_msgs/MarkerArray';
    } else {
      // 默认优先 PointCloud2
      this._topicName = topicName || '/octomap_point_cloud_centers';
      this._msgType   = 'sensor_msgs/PointCloud2';
    }

    console.log(`[OctoMap] 订阅 ${this._topicName} (${this._msgType})`);

    this._sub = new ROSLIB.Topic({
      ros:          this.ros,
      name:         this._topicName,
      messageType:  this._msgType,
      throttle_rate: 1500,
      queue_length:  1
    });

    this._sub.subscribe(msg => {
      if (!this.autoUpdate) return;
      if (this._msgType === 'sensor_msgs/PointCloud2') {
        this._updateFromPointCloud2(msg);
      } else {
        this._updateFromMarkerArray(msg);
      }
      window.App?.setTopicActive('topic-octomap', true);
    });
  }

  // ─── PointCloud2 解码与渲染（主路径）───────────────

  _decodePointCloud2(msg) {
    const nPts     = msg.width * msg.height;
    const step     = msg.point_step;

    // 解析字段偏移
    let xOff = 0, yOff = 4, zOff = 8, rgbOff = -1;
    for (const f of msg.fields) {
      if (f.name === 'x')   xOff   = f.offset;
      if (f.name === 'y')   yOff   = f.offset;
      if (f.name === 'z')   zOff   = f.offset;
      if (f.name === 'rgb') rgbOff = f.offset;
    }

    // 解码 base64 or 普通数组
    let raw;
    if (typeof msg.data === 'string') {
      const bin = atob(msg.data);
      raw = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) raw[i] = bin.charCodeAt(i);
    } else {
      raw = new Uint8Array(msg.data);
    }

    const positions = new Float32Array(nPts * 3);
    const colors    = new Float32Array(nPts * 3);
    const view      = new DataView(raw.buffer);

    let zMin =  Infinity;
    let zMax = -Infinity;

    for (let i = 0; i < nPts; i++) {
      const base = i * step;
      const x = view.getFloat32(base + xOff, true);
      const y = view.getFloat32(base + yOff, true);
      const z = view.getFloat32(base + zOff, true);

      if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;

      positions[i * 3]     = x;
      positions[i * 3 + 1] = y;
      positions[i * 3 + 2] = z;

      zMin = Math.min(zMin, z);
      zMax = Math.max(zMax, z);

      // 原始颜色（packed XYZRGB: 0x00RRGGBB）
      if (rgbOff >= 0) {
        const packed = view.getUint32(base + rgbOff, true);
        colors[i * 3]     = ((packed >> 16) & 0xFF) / 255;
        colors[i * 3 + 1] = ((packed >> 8)  & 0xFF) / 255;
        colors[i * 3 + 2] = (packed          & 0xFF) / 255;
      } else {
        colors[i * 3] = colors[i * 3 + 1] = colors[i * 3 + 2] = 0.8;
      }
    }

    return { positions, colors, count: nPts, zMin, zMax, rgbOff };
  }

  _applyHeightColors(colors, positions, count, zMin, zMax) {
    const zRange = (zMax - zMin) || 1;
    const tmp = new THREE.Color();
    for (let i = 0; i < count; i++) {
      const z = positions[i * 3 + 2];
      const t = (z - zMin) / zRange;
      tmp.setHSL((1 - t) * 0.667, 1.0, 0.55);
      colors[i * 3]     = tmp.r;
      colors[i * 3 + 1] = tmp.g;
      colors[i * 3 + 2] = tmp.b;
    }
  }

  _updateFromPointCloud2(msg) {
    const decoded = this._decodePointCloud2(msg);
    const { positions, colors, count, zMin, zMax } = decoded;

    if (this.heightColor) {
      this._applyHeightColors(colors, positions, count, zMin, zMax);
    }
    // 否则直接使用点云原始 RGB 颜色（来自 YOLOE 彩色点云）

    this._renderPoints(positions, colors, count);
    this._totalVoxels = count;
    this._lastUpdate  = new Date();
    this._updateStats(zMin, zMax);
  }

  _renderPoints(positions, colors, count) {
    // 移除旧点云
    if (this._pointCloud) {
      this.scene.remove(this._pointCloud);
      this._pointCloud.geometry.dispose();
      this._pointCloud.material.dispose();
      this._pointCloud = null;
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions.slice(0, count * 3), 3));
    geo.setAttribute('color',    new THREE.BufferAttribute(colors.slice(0, count * 3),    3));

    // 点大小根据分辨率自适应
    const pointSize = Math.max(0.03, this._resolution * 0.8);
    const mat = new THREE.PointsMaterial({
      size:         pointSize,
      vertexColors: true,
      sizeAttenuation: true,
      depthWrite:   false,
      transparent:  true,
      opacity:      0.92
    });

    this._pointCloud = new THREE.Points(geo, mat);
    this.scene.add(this._pointCloud);
  }

  // ─── MarkerArray 兜底路径 ─────────────────────────

  _updateFromMarkerArray(msg) {
    if (this._pointCloud) {
      this.scene.remove(this._pointCloud);
      this._pointCloud.geometry.dispose();
      this._pointCloud.material.dispose();
      this._pointCloud = null;
    }
    if (this._meshes) {
      this._meshes.forEach(m => { this.scene.remove(m); m.geometry.dispose(); m.material.dispose(); });
    }
    this._meshes = [];

    if (!msg.markers) return;

    let zMin =  Infinity;
    let zMax = -Infinity;
    msg.markers.forEach(mk => {
      if (mk.points) mk.points.forEach(p => {
        zMin = Math.min(zMin, p.z);
        zMax = Math.max(zMax, p.z);
      });
    });
    if (!isFinite(zMin)) { zMin = 0; zMax = 3; }

    const dummy  = new THREE.Object3D();
    const _color = new THREE.Color();
    let total    = 0;

    msg.markers.forEach(marker => {
      if (!marker.points || marker.points.length === 0 || marker.action === 3) return;
      const count   = marker.points.length;
      const res     = marker.scale?.x ?? 0.1;
      this._resolution = Math.min(this._resolution, res);

      const geo = new THREE.BoxGeometry(res * 0.85, res * 0.85, res * 0.85);
      const mat = new THREE.MeshLambertMaterial();
      const mesh = new THREE.InstancedMesh(geo, mat, count);

      const hasClr = marker.colors?.length === count;

      marker.points.forEach((pt, i) => {
        dummy.position.set(pt.x, pt.y, pt.z);
        dummy.updateMatrix();
        mesh.setMatrixAt(i, dummy.matrix);

        if (this.heightColor) {
          const t = (pt.z - zMin) / ((zMax - zMin) || 1);
          _color.setHSL((1 - t) * 0.667, 1.0, 0.55);
        } else if (hasClr) {
          const c = marker.colors[i];
          _color.setRGB(c.r, c.g, c.b);
        } else {
          const mc = marker.color;
          _color.setRGB(mc?.r ?? 0.5, mc?.g ?? 0.5, mc?.b ?? 0.5);
        }
        mesh.setColorAt(i, _color);
      });

      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
      this.scene.add(mesh);
      this._meshes.push(mesh);
      total += count;
    });

    this._totalVoxels = total;
    this._lastUpdate  = new Date();
    this._updateStats(zMin, zMax);
  }

  // ─── 统计更新 ─────────────────────────────────────

  _updateStats(zMin, zMax) {
    const $ = id => document.getElementById(id);
    $('octo-voxels') && ($('octo-voxels').textContent = this._totalVoxels.toLocaleString());
    $('octo-time')   && ($('octo-time').textContent   = this._lastUpdate?.toLocaleTimeString() ?? '--');
    $('octomap-voxel-count') && ($('octomap-voxel-count').textContent = `${this._totalVoxels.toLocaleString()} 点`);

    // 颜色模式提示
    const modeEl = $('octomap-color-mode');
    if (modeEl) {
      modeEl.textContent = this.heightColor ? '高度着色' : '原始颜色';
    }
  }

  // ─── 公共控制方法 ─────────────────────────────────

  setAutoUpdate(v) { this.autoUpdate = v; }

  setHeightColor(v) {
    this.heightColor = v;
  }

  resetView() {
    this.camera.position.set(8, 8, 5);
    this.controls.target.set(0, 0, 1);
    this.camera.up.set(0, 0, 1);
    this.controls.update();
  }

  clearVoxels() {
    if (this._pointCloud) {
      this.scene.remove(this._pointCloud);
      this._pointCloud.geometry.dispose();
      this._pointCloud.material.dispose();
      this._pointCloud = null;
    }
    if (this._meshes) {
      this._meshes.forEach(m => { this.scene.remove(m); m.geometry.dispose(); m.material.dispose(); });
      this._meshes = [];
    }
    this._totalVoxels = 0;
    this._updateStats(0, 0);
    window.App?.showToast('OctoMap 已清除', 'warn');
  }

  // ─── 渲染循环 ─────────────────────────────────────

  _onResize() {
    const W = this.container.clientWidth;
    const H = this.container.clientHeight;
    if (!W || !H) return;
    this.camera.aspect = W / H;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(W, H);
  }

  _startRenderLoop() {
    const loop = () => {
      requestAnimationFrame(loop);
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    };
    loop();
  }
}
