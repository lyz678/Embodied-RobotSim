/**
 * chat_agent.js - Qwen3 自然语言对话（连接 llm_agent WebSocket）
 */

const ChatAgent = {
  ws: null,
  busy: false,
  reconnectTimer: null,
  agentUrl: 'ws://localhost:8889/ws/chat',

  init() {
    const saved = localStorage.getItem('agent-ws-url');
    if (saved) this.agentUrl = saved;
    this._bindUI();
    this._showWelcome();
    this.connect();
  },

  _bindUI() {
    const sendBtn = document.getElementById('btn-chat-send');
    const input   = document.getElementById('chat-input');
    const sceneBtn = document.getElementById('btn-scene-detect');

    sendBtn?.addEventListener('click', () => this.send());
    input?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        this.send();
      }
    });

    document.querySelectorAll('.chat-chip').forEach(btn => {
      btn.addEventListener('click', () => {
        const msg = btn.getAttribute('data-msg');
        if (input) input.value = msg;
        this.send();
      });
    });

    sceneBtn?.addEventListener('click', () => this.requestScene());

    const cfgInput = document.getElementById('cfg-agent-ws');
    if (cfgInput) {
      cfgInput.value = this.agentUrl;
      document.getElementById('btn-apply-config')?.addEventListener('click', () => {
        const v = cfgInput.value.trim();
        if (v) {
          this.agentUrl = v;
          localStorage.setItem('agent-ws-url', v);
          this.connect();
        }
      }, { capture: true });
    }
  },

  connect() {
    if (this.ws) {
      try { this.ws.close(); } catch (_) {}
      this.ws = null;
    }
    this._setWsStatus('connecting');
    try {
      this.ws = new WebSocket(this.agentUrl);
    } catch (e) {
      this._setWsStatus('disconnected');
      this._appendSystem(`⚠️ 无法连接 Agent: ${e.message}`);
      this._scheduleReconnect();
      return;
    }

    this.ws.onopen = () => {
      this._setWsStatus('connected');
      this._appendSystem('✅ 已连接 Qwen3 Agent');
    };

    this.ws.onclose = () => {
      this._setWsStatus('disconnected');
      this.busy = false;
      this._appendSystem('🔴 Agent 连接断开，5 秒后重试...');
      this._scheduleReconnect();
    };

    this.ws.onerror = () => {
      this._setWsStatus('disconnected');
    };

    this.ws.onmessage = (ev) => {
      try {
        const data = JSON.parse(ev.data);
        this._handleEvent(data);
      } catch (e) {
        console.warn('chat parse error', e);
      }
    };
  },

  _scheduleReconnect() {
    clearTimeout(this.reconnectTimer);
    this.reconnectTimer = setTimeout(() => this.connect(), 5000);
  },

  _setWsStatus(state) {
    const el = document.getElementById('agent-ws-status');
    if (!el) return;
    el.className = 'status-badge';
    if (state === 'connected') {
      el.classList.add('status-connected');
      el.textContent = '● Agent';
    } else if (state === 'connecting') {
      el.classList.add('status-connecting', 'pulsing');
      el.textContent = '● 连接中';
    } else {
      el.classList.add('status-disconnected');
      el.textContent = '● 未连接';
    }
  },

  _handleEvent(ev) {
    if (ev._scene) {
      const sceneEl = document.getElementById('agent-scene');
      if (sceneEl) sceneEl.textContent = `场景: ${ev._scene}`;
    }

    switch (ev.type) {
      case 'assistant':
        this._removeTyping();
        this._appendMsg('assistant', ev.content || '');
        break;
      case 'thinking':
        this._appendSystem(`💭 ${ev.content}`);
        break;
      case 'tool_call':
        this._appendTool(ev.name, ev.args, 'running', ev.id);
        break;
      case 'tool_result':
        this._updateTool(ev.id, ev.success, ev.result);
        break;
      case 'progress':
        this._appendProgress(ev.content);
        break;
      case 'status':
        this._appendSystem(`▶ ${ev.task || ''}`);
        break;
      case 'scene_update':
        if (ev.scene) {
          const sceneEl = document.getElementById('agent-scene');
          if (sceneEl) sceneEl.textContent = `场景: ${ev.scene}`;
        }
        this._appendMsg('assistant', ev.description || `当前场景: ${ev.scene}`);
        break;
      case 'error':
        this._removeTyping();
        this._appendError(ev.content);
        this.busy = false;
        break;
      case 'done':
        this._removeTyping();
        this.busy = false;
        break;
      default:
        break;
    }
  },

  send() {
    const input = document.getElementById('chat-input');
    const text = (input?.value || '').trim();
    if (!text || this.busy) return;

    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      window.App?.showToast?.('Agent 未连接，请确认已启动 agent_server', 'warn');
      this.connect();
      return;
    }

    this.busy = true;
    this._appendMsg('user', text);
    if (input) input.value = '';
    this._showTyping();

    this.ws.send(JSON.stringify({ type: 'chat', message: text }));
  },

  requestScene() {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      window.App?.showToast?.('Agent 未连接', 'warn');
      return;
    }
    this.busy = true;
    this._showTyping();
    this.ws.send(JSON.stringify({ type: 'scene_request' }));
  },

  _messagesEl() {
    return document.getElementById('chat-messages');
  },

  _scrollBottom() {
    const el = this._messagesEl();
    if (el) el.scrollTop = el.scrollHeight;
  },

  _appendMsg(role, text) {
    const el = this._messagesEl();
    if (!el || !text) return;
    const div = document.createElement('div');
    div.className = `chat-msg chat-msg-${role}`;
    div.textContent = text;
    el.appendChild(div);
    this._scrollBottom();
  },

  _appendSystem(text) {
    const el = this._messagesEl();
    if (!el) return;
    const div = document.createElement('div');
    div.className = 'chat-msg chat-msg-system';
    div.textContent = text;
    el.appendChild(div);
    this._scrollBottom();
  },

  _showWelcome() {
    const el = this._messagesEl();
    if (!el) return;
    const div = document.createElement('div');
    div.className = 'chat-msg chat-msg-system chat-msg-welcome';
    div.innerHTML = '🤖 Qwen3 智能助手已就绪。可以说：&ldquo;帮我去厨房拿瓶可乐&rdquo;或使用下方快捷指令。';
    el.appendChild(div);
  },

  _appendError(text) {
    const el = this._messagesEl();
    if (!el) return;
    const div = document.createElement('div');
    div.className = 'chat-msg chat-msg-error';
    div.textContent = text;
    el.appendChild(div);
    this._scrollBottom();
  },

  _appendProgress(text) {
    const el = this._messagesEl();
    if (!el) return;
    let last = el.querySelector('.chat-msg-progress:last-of-type');
    if (last) {
      last.textContent = `⏳ ${text}`;
    } else {
      last = document.createElement('div');
      last.className = 'chat-msg chat-msg-progress';
      last.textContent = `⏳ ${text}`;
      el.appendChild(last);
    }
    this._scrollBottom();
  },

  _appendTool(name, args, status, id) {
    const el = this._messagesEl();
    if (!el) return;
    const div = document.createElement('div');
    div.className = `chat-msg chat-msg-tool ${status}`;
    div.dataset.toolId = id || '';
    const argStr = args ? JSON.stringify(args) : '';
    div.textContent = `🔧 ${name}(${argStr}) …`;
    el.appendChild(div);
    this._scrollBottom();
  },

  _updateTool(id, success, result) {
    const el = this._messagesEl();
    if (!el) return;
    const div = el.querySelector(`[data-tool-id="${id}"]`);
    if (!div) {
      this._appendSystem(`${success ? '✓' : '✗'} ${result}`);
      return;
    }
    div.className = `chat-msg chat-msg-tool ${success ? 'ok' : 'fail'}`;
    div.textContent = `${success ? '✓' : '✗'} ${result}`;
    this._scrollBottom();
  },

  _showTyping() {
    this._removeTyping();
    const el = this._messagesEl();
    if (!el) return;
    const div = document.createElement('div');
    div.id = 'chat-typing';
    div.className = 'chat-msg chat-msg-assistant chat-typing';
    div.textContent = '思考中';
    el.appendChild(div);
    this._scrollBottom();
  },

  _removeTyping() {
    document.getElementById('chat-typing')?.remove();
  },
};

document.addEventListener('DOMContentLoaded', () => {
  ChatAgent.init();
  window.ChatAgent = ChatAgent;
});
