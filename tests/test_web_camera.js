const vm = require('vm'), fs = require('fs'), assert = require('assert');
const timers = new Map();
let nextTimer = 1, subscriptions = 0;
const elements = new Map();
for (const id of ['camera-img', 'camera-canvas', 'camera-overlay', 'camera-fps']) {
  elements.set(id, { src: '', textContent: '', classList: { add() {}, remove() {} },
    getContext: () => ({}), querySelector: () => null });
}
const context = {
  document: { getElementById: id => elements.get(id) ?? null },
  window: {}, console, Date, encodeURIComponent,
  setTimeout: fn => { const id = nextTimer++; timers.set(id, fn); return id; },
  clearTimeout: id => timers.delete(id), clearInterval() {},
  ROSLIB: { Topic: class {
    subscribe() { subscriptions++; }
    unsubscribe() { subscriptions--; }
  } },
};
vm.createContext(context);
vm.runInContext(fs.readFileSync('src/embodied/web_ui/js/camera.js', 'utf8') +
  ';globalThis.CameraView = CameraView;', context);
const camera = new context.CameraView({});
camera.connect('http://localhost:8080');
const img = elements.get('camera-img');
assert(img.src.includes('topic=/x_bot/camera_left/image_raw'));
assert(!img.src.includes('%2F'));
assert(img.src.includes('qos_profile=sensor_data'));
img.onload();
assert.equal(timers.size, 0);
assert.equal(elements.get('camera-fps').textContent, 'MJPEG');
camera.switchTopic('/yoloe_multi_text_prompt/image_result');
camera.switchTopic('/x_bot/camera_left/image_raw');
assert.equal(timers.size, 1, 'Switching topics cancels the previous fallback timer');
img.onerror();
assert.equal(subscriptions, 1);
assert.equal(camera._mode, 'raw');
assert.equal(img.src, '', 'Fallback closes the failed stream');
camera.disconnect();
assert.equal(subscriptions, 0);
assert.equal(timers.size, 0);
console.log('MJPEG topic compatibility, sensor QoS, honest FPS and fallback cleanup PASS');
