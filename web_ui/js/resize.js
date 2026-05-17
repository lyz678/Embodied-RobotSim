/**
 * resize.js - 面板大小可调整
 *
 * 手柄说明：
 *  resize-col-1   左列（相机+遥控）宽度                [左右拖动]
 *  resize-row     底部条高度（上区 ↕ 底部条）           [上下拖动]
 *  resize-col-mid 底部条内导航地图 ↔ OctoMap 宽度      [左右拖动]
 *
 * 功能：布局持久化（localStorage）、双击恢复默认
 */

class ResizableLayout {
  constructor() {
    this.defaults = {
      col1:     480,   // 列1（相机）宽 px
      col2:     340,   // 列2（遥控）宽 px
      colChat:  450,   // 右侧（对话）宽 px
      bottomH:  380,   // 底部条高 px
      navBasis: null,  // 导航地图宽 px（null = CSS 55% 决定）
    };
    this.state = { ...this.defaults };

    this._load();
    this._applyAll();
    this._initHandles();
    this._observeNavPanel();
  }

  // ─── 应用布局变量 ─────────────────────────────

  _applyAll() {
    const r = document.documentElement;
    r.style.setProperty('--col1',     this.state.col1    + 'px');
    r.style.setProperty('--col2',     this.state.col2    + 'px');
    r.style.setProperty('--col-chat', this.state.colChat + 'px');
    r.style.setProperty('--bottom-h', this.state.bottomH + 'px');
    if (this.state.navBasis !== null) {
      const nav = document.getElementById('panel-navmap');
      if (nav) nav.style.flexBasis = this.state.navBasis + 'px';
    }
  }

  // ─── 持久化 ───────────────────────────────────

  _load() {
    try {
      const s = JSON.parse(localStorage.getItem('panel-layout-v7') || '{}');
      if (typeof s.col1     === 'number') this.state.col1     = Math.max(180, s.col1);
      if (typeof s.col2     === 'number') this.state.col2     = Math.max(200, s.col2);
      if (typeof s.colChat  === 'number') this.state.colChat  = Math.max(240, s.colChat);
      if (typeof s.bottomH  === 'number') this.state.bottomH  = Math.max(180, s.bottomH);
      if (typeof s.navBasis === 'number') this.state.navBasis = s.navBasis;
    } catch(e) {}
  }

  _save() {
    localStorage.setItem('panel-layout-v7', JSON.stringify(this.state));
  }

  // ─── 初始化手柄 ───────────────────────────────

  _initHandles() {
    // 列1宽（相机）
    this._colHandle(
      document.getElementById('resize-col-1'),
      () => this.state.col1,
      v  => {
        this.state.col1 = v;
        document.documentElement.style.setProperty('--col1', v + 'px');
      },
      { min: 180, max: 800, def: this.defaults.col1 }
    );

    // 列2宽（遥控）
    this._colHandle(
      document.getElementById('resize-col-2'),
      () => this.state.col2,
      v  => {
        this.state.col2 = v;
        document.documentElement.style.setProperty('--col2', v + 'px');
      },
      { min: 200, max: 600, def: this.defaults.col2 }
    );

    // 右侧对话栏宽（拖动方向反转：向右拖 = 变窄）
    this._colHandle(
      document.getElementById('resize-col-3'),
      () => this.state.colChat,
      v  => {
        this.state.colChat = v;
        document.documentElement.style.setProperty('--col-chat', v + 'px');
      },
      { min: 240, max: 800, def: this.defaults.colChat, invert: true }
    );

    // 底部条高度
    this._rowHandle(
      document.getElementById('resize-row'),
      () => this.state.bottomH,
      v  => {
        this.state.bottomH = v;
        document.documentElement.style.setProperty('--bottom-h', v + 'px');
        this._notifyResize();
      },
      { min: 180, max: 800, def: this.defaults.bottomH }
    );

    // 底部条内：导航地图 ↔ OctoMap
    this._midHandle(document.getElementById('resize-col-mid'));
  }

  // ── 左右列宽手柄 ──────────────────────────────

  _colHandle(el, getter, setter, { min, max, def, invert = false }) {
    if (!el) return;
    el.title = '拖动调整宽度，双击恢复默认';

    el.addEventListener('dblclick', () => {
      setter(def);
      this._save();
      this._notifyResize();
    });

    el.addEventListener('mousedown', e => {
      e.preventDefault();
      const x0  = e.clientX;
      const v0  = getter();
      el.classList.add('dragging');
      document.body.classList.add('resizing');

      const move = ev => {
        const dv = invert ? -(ev.clientX - x0) : (ev.clientX - x0);
        setter(Math.round(Math.max(min, Math.min(max, v0 + dv))));
        this._notifyResize();
      };
      const up = () => {
        el.classList.remove('dragging');
        document.body.classList.remove('resizing');
        document.removeEventListener('mousemove', move);
        document.removeEventListener('mouseup',   up);
        this._save();
      };
      document.addEventListener('mousemove', move);
      document.addEventListener('mouseup',   up);
    });
  }

  // ── 上下高度手柄 ──────────────────────────────

  _rowHandle(el, getter, setter, { min, max, def }) {
    if (!el) return;
    el.title = '拖动调整底部高度，双击恢复默认';

    el.addEventListener('dblclick', () => {
      setter(def);
      this._save();
    });

    el.addEventListener('mousedown', e => {
      e.preventDefault();
      const y0 = e.clientY;
      const v0 = getter();
      el.classList.add('dragging');
      document.body.classList.add('resizing-row');

      const move = ev => {
        // 向上拖（dy < 0）→ 底部条变高
        const dy = ev.clientY - y0;
        setter(Math.round(Math.max(min, Math.min(max, v0 - dy))));
      };
      const up = () => {
        el.classList.remove('dragging');
        document.body.classList.remove('resizing-row');
        document.removeEventListener('mousemove', move);
        document.removeEventListener('mouseup',   up);
        this._save();
      };
      document.addEventListener('mousemove', move);
      document.addEventListener('mouseup',   up);
    });
  }

  // ── 底部条内部分割：nav ↔ octomap ─────────────

  _midHandle(el) {
    if (!el) return;
    const navEl = document.getElementById('panel-navmap');
    if (!navEl) return;

    el.title = '拖动调整导航地图/OctoMap 宽度，双击恢复均分';

    el.addEventListener('dblclick', () => {
      navEl.style.flexBasis = '';
      this.state.navBasis = null;
      this._save();
      this._notifyResize();
    });

    el.addEventListener('mousedown', e => {
      e.preventDefault();
      const x0   = e.clientX;
      const w0   = navEl.getBoundingClientRect().width;
      const area = document.getElementById('bottom-area');
      const maxW = area ? area.clientWidth - 185 : 999;
      el.classList.add('dragging');
      document.body.classList.add('resizing');

      const move = ev => {
        const newW = Math.round(Math.max(180, Math.min(maxW, w0 + ev.clientX - x0)));
        navEl.style.flexBasis = newW + 'px';
        this.state.navBasis = newW;
        this._notifyResize();
      };
      const up = () => {
        el.classList.remove('dragging');
        document.body.classList.remove('resizing');
        document.removeEventListener('mousemove', move);
        document.removeEventListener('mouseup',   up);
        this._save();
      };
      document.addEventListener('mousemove', move);
      document.addEventListener('mouseup',   up);
    });
  }

  // ─── 通知子组件 ───────────────────────────────

  _notifyResize() {
    window.App?.navMap?._resizeCanvas?.();
  }

  _observeNavPanel() {
    const nav = document.getElementById('panel-navmap');
    if (!nav) return;
    new ResizeObserver(() => window.App?.navMap?._resizeCanvas?.()).observe(nav);
  }
}
