/**
 * octomap.js - 3D OctoMap 可视化模块（Three.js）
 *
 * 渲染模式（voxelMode）：
 *   true  → 稀疏体素 InstancedMesh + BoxGeometry（默认）
 *   false → 点云 THREE.Points
 *
 * 颜色模式（heightColor）：
 *   false → 原始 RGB（来自 PointCloud2 rgb 字段，默认）
 *   true  → 高度彩虹映射
 */

class OctoMapViewer {
  constructor(containerId, ros) {
    this.container    = document.getElementById(containerId);
    this.ros          = ros;
    this.autoUpdate   = true;
    this.heightColor  = false;  // 默认原始颜色
    this.voxelMode    = true;   // 默认稀疏体素
    this._sub         = null;
    this._topicName   = '/octomap_point_cloud_centers';
    this._msgType     = 'sensor_msgs/PointCloud2';
    this._pointCloud  = null;   // THREE.Points（点云模式）
    this._voxelMesh   = null;   // THREE.InstancedMesh（体素模式）
    this._meshes      = [];     // MarkerArray 模式
    this._totalVoxels = 0;
    this._lastUpdate  = null;
    this._resolution  = 0.10;

    this._initThreeJS();
    this._addSceneObjects();
    this._startRenderLoop();

    const ro = new ResizeObserver(() => this._onResize());
    ro.observe(this.container);
  }

  // ─── Three.js 初始化 ──────────────────────────────────

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

    this.camera = new THREE.PerspectiveCamera(55, W / H, 0.02, 200);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(8, 8, 5);
    this.camera.lookAt(0, 0, 1);

    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0, 1);
    this.controls.enableDamping    = true;
    this.controls.dampingFactor    = 0.08;
    this.controls.minDistance      = 0.3;
    this.controls.maxDistance      = 80;
    this.controls.screenSpacePanning = false;
    this.controls.update();
  }

  _addSceneObjects() {
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.8));
    const dir = new THREE.DirectionalLight(0xffffff, 0.5);
    dir.position.set(5, 5, 10);
    this.scene.add(dir);
    this.scene.add(new THREE.AxesHelper(1.5));
    const grid = new THREE.GridHelper(20, 20, 0x1a2840, 0x0d1520);
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);
  }

  // ─── ROS 订阅 ────────────────────────────────────────

  connect(topicName) {
    if (!this.ros) { console.warn('[OctoMap] ros 未设置'); return; }
    if (this._sub) { try { this._sub.unsubscribe(); } catch(e) {} }

    if (topicName && topicName.includes('point_cloud')) {
      this._topicName = topicName;
      this._msgType   = 'sensor_msgs/PointCloud2';
    } else if (topicName && topicName.includes('vis_array')) {
      this._topicName = topicName;
      this._msgType   = 'visualization_msgs/MarkerArray';
    } else {
      this._topicName = topicName || '/octomap_point_cloud_centers';
      this._msgType   = 'sensor_msgs/PointCloud2';
    }

    console.log(`[OctoMap] 订阅 ${this._topicName} (${this._msgType})`);

    this._sub = new ROSLIB.Topic({
      ros:           this.ros,
      name:          this._topicName,
      messageType:   this._msgType,
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

  // ─── PointCloud2 解码 ────────────────────────────────

  _decodePointCloud2(msg) {
    const nPts = msg.width * msg.height;
    const step = msg.point_step;

    let xOff = 0, yOff = 4, zOff = 8, rgbOff = -1;
    for (const f of msg.fields) {
      if (f.name === 'x')   xOff   = f.offset;
      if (f.name === 'y')   yOff   = f.offset;
      if (f.name === 'z')   zOff   = f.offset;
      if (f.name === 'rgb') rgbOff = f.offset;
    }

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
    let zMin =  Infinity, zMax = -Infinity;

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

      if (rgbOff >= 0) {
        const packed = view.getUint32(base + rgbOff, true);
        colors[i * 3]     = ((packed >> 16) & 0xFF) / 255;
        colors[i * 3 + 1] = ((packed >> 8)  & 0xFF) / 255;
        colors[i * 3 + 2] = ( packed        & 0xFF) / 255;
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
      const t = (positions[i * 3 + 2] - zMin) / zRange;
      tmp.setHSL((1 - t) * 0.667, 1.0, 0.55);
      colors[i * 3]     = tmp.r;
      colors[i * 3 + 1] = tmp.g;
      colors[i * 3 + 2] = tmp.b;
    }
  }

  // ─── PointCloud2 主渲染路径 ──────────────────────────

  _updateFromPointCloud2(msg) {
    const { positions, colors, count, zMin, zMax } = this._decodePointCloud2(msg);

    if (this.heightColor) {
      this._applyHeightColors(colors, positions, count, zMin, zMax);
    }

    if (this.voxelMode) {
      this._renderVoxels(positions, colors, count);
    } else {
      this._renderPoints(positions, colors, count);
    }

    this._totalVoxels = count;
    this._lastUpdate  = new Date();
    this._updateStats(zMin, zMax);
  }

  // ─── 稀疏体素渲染（InstancedMesh + BoxGeometry） ─────

  _renderVoxels(positions, colors, count) {
    this._clearVoxelMesh();
    this._clearPointCloud();
    if (count === 0) return;

    const size = this._resolution * 0.85;
    const geo  = new THREE.BoxGeometry(size, size, size);
    const mat  = new THREE.MeshLambertMaterial();
    const mesh = new THREE.InstancedMesh(geo, mat, count);
    mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);

    const dummy = new THREE.Object3D();
    const col   = new THREE.Color();

    for (let i = 0; i < count; i++) {
      dummy.position.set(
        positions[i * 3],
        positions[i * 3 + 1],
        positions[i * 3 + 2]
      );
      dummy.updateMatrix();
      mesh.setMatrixAt(i, dummy.matrix);
      col.setRGB(colors[i * 3], colors[i * 3 + 1], colors[i * 3 + 2]);
      mesh.setColorAt(i, col);
    }

    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    this._voxelMesh = mesh;
    this.scene.add(mesh);
  }

  // ─── 点云渲染（THREE.Points） ────────────────────────

  _renderPoints(positions, colors, count) {
    this._clearPointCloud();
    this._clearVoxelMesh();

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions.slice(0, count * 3), 3));
    geo.setAttribute('color',    new THREE.BufferAttribute(colors.slice(0, count * 3),    3));

    const mat = new THREE.PointsMaterial({
      size:            Math.max(0.03, this._resolution * 0.8),
      vertexColors:    true,
      sizeAttenuation: true,
      depthWrite:      false,
      transparent:     true,
      opacity:         0.92
    });

    this._pointCloud = new THREE.Points(geo, mat);
    this.scene.add(this._pointCloud);
  }

  // ─── MarkerArray 兜底路径 ────────────────────────────

  _updateFromMarkerArray(msg) {
    this._clearPointCloud();
    this._clearVoxelMesh();
    this._meshes.forEach(m => { this.scene.remove(m); m.geometry.dispose(); m.material.dispose(); });
    this._meshes = [];
    if (!msg.markers) return;

    let zMin =  Infinity, zMax = -Infinity;
    msg.markers.forEach(mk => {
      if (mk.points) mk.points.forEach(p => {
        zMin = Math.min(zMin, p.z);
        zMax = Math.max(zMax, p.z);
      });
    });
    if (!isFinite(zMin)) { zMin = 0; zMax = 3; }

    const dummy = new THREE.Object3D();
    const _col  = new THREE.Color();
    let total   = 0;

    msg.markers.forEach(marker => {
      if (!marker.points || marker.points.length === 0 || marker.action === 3) return;
      const cnt = marker.points.length;
      const res = marker.scale?.x ?? 0.1;
      this._resolution = Math.min(this._resolution, res);

      const geo  = new THREE.BoxGeometry(res * 0.85, res * 0.85, res * 0.85);
      const mat  = new THREE.MeshLambertMaterial();
      const mesh = new THREE.InstancedMesh(geo, mat, cnt);
      const hasClr = marker.colors?.length === cnt;

      marker.points.forEach((pt, i) => {
        dummy.position.set(pt.x, pt.y, pt.z);
        dummy.updateMatrix();
        mesh.setMatrixAt(i, dummy.matrix);

        if (this.heightColor) {
          const t = (pt.z - zMin) / ((zMax - zMin) || 1);
          _col.setHSL((1 - t) * 0.667, 1.0, 0.55);
        } else if (hasClr) {
          const c = marker.colors[i];
          _col.setRGB(c.r, c.g, c.b);
        } else {
          const mc = marker.color;
          _col.setRGB(mc?.r ?? 0.5, mc?.g ?? 0.5, mc?.b ?? 0.5);
        }
        mesh.setColorAt(i, _col);
      });

      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
      this.scene.add(mesh);
      this._meshes.push(mesh);
      total += cnt;
    });

    this._totalVoxels = total;
    this._lastUpdate  = new Date();
    this._updateStats(zMin, zMax);
  }

  // ─── 内部清理辅助 ────────────────────────────────────

  _clearVoxelMesh() {
    if (this._voxelMesh) {
      this.scene.remove(this._voxelMesh);
      this._voxelMesh.geometry.dispose();
      this._voxelMesh.material.dispose();
      this._voxelMesh = null;
    }
  }

  _clearPointCloud() {
    if (this._pointCloud) {
      this.scene.remove(this._pointCloud);
      this._pointCloud.geometry.dispose();
      this._pointCloud.material.dispose();
      this._pointCloud = null;
    }
  }

  // ─── 统计更新 ────────────────────────────────────────

  _updateStats(zMin, zMax) {
    const $ = id => document.getElementById(id);
    $('octo-voxels')         && ($('octo-voxels').textContent         = this._totalVoxels.toLocaleString());
    $('octo-time')           && ($('octo-time').textContent           = this._lastUpdate?.toLocaleTimeString() ?? '--');
    $('octomap-voxel-count') && ($('octomap-voxel-count').textContent = `${this._totalVoxels.toLocaleString()} ${this.voxelMode ? '体素' : '点'}`);
    const modeEl = $('octomap-color-mode');
    if (modeEl) modeEl.textContent = this.heightColor ? '高度着色' : '原始颜色';
  }

  // ─── 公共控制方法 ────────────────────────────────────

  setAutoUpdate(v)  { this.autoUpdate  = v; }
  setHeightColor(v) { this.heightColor = v; }
  setVoxelMode(v)   { this.voxelMode   = v; }

  resetView() {
    this.camera.position.set(8, 8, 5);
    this.controls.target.set(0, 0, 1);
    this.camera.up.set(0, 0, 1);
    this.controls.update();
  }

  clearVoxels() {
    this._clearVoxelMesh();
    this._clearPointCloud();
    this._meshes.forEach(m => { this.scene.remove(m); m.geometry.dispose(); m.material.dispose(); });
    this._meshes = [];
    this._totalVoxels = 0;
    this._updateStats(0, 0);
    window.App?.showToast('OctoMap 已清除', 'warn');
  }

  // ─── 渲染循环 ────────────────────────────────────────

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
