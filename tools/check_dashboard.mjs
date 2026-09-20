// Headless smoke test for web/index.html.
//
// The dashboard and the firmware's telemetry encoder are two halves of one
// contract, and nothing in a C++ build checks the JavaScript half. This runs
// the page's real script against a real frame produced by the real encoder
// (test/test_telemetry prints one), with just enough of a DOM stubbed out for
// it to execute.
//
// What it is actually looking for: a getElementById that names an element the
// page does not have, a telemetry field the dashboard reads under the wrong
// name, and any exception thrown on the render path. Canvas output is not
// inspected -- only that drawing happened.
//
// Usage: node tools/check_dashboard.mjs <frame.json>

import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const html = readFileSync(new URL('../web/index.html', import.meta.url), 'utf8');
const frameJson = readFileSync(process.argv[2], 'utf8').trim();

const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
if (!script) throw new Error('no <script> block found in web/index.html');

// ── Minimal DOM ────────────────────────────────────────────────────────────
const elements = new Map();
const missingIds = [];
let drawCalls = 0;

function makeContext() {
  return new Proxy({
    measureText: () => ({width: 10}),
    setTransform(){}, clearRect(){}, fillText(){},
  }, {
    get(target, prop) {
      if (prop in target) return target[prop];
      return () => { drawCalls++; };
    },
    set(){ return true; },
  });
}

function makeElement(id) {
  const el = {
    id, textContent: '', className: '', value: '', checked: false,
    clientWidth: 400, clientHeight: 300, width: 0, height: 0,
    style: {}, children: [],
    addEventListener(){}, setPointerCapture(){}, removeAttribute(){},
    getContext: () => makeElement._ctx ||= makeContext(),
    appendChild(child){ this.children.push(child); },
    querySelector(){ return makeElement('__query__'); },
    get innerHTML(){ return this._html || ''; },
    // The knob builder writes markup containing new ids, then looks them up by
    // id -- so registering them here is what makes that path work.
    set innerHTML(v){
      this._html = v;
      for (const m of v.matchAll(/id="([^"]+)"/g)) {
        if (!elements.has(m[1])) elements.set(m[1], makeElement(m[1]));
      }
    },
  };
  return el;
}

for (const m of html.matchAll(/id="([^"]+)"/g)) {
  elements.set(m[1], makeElement(m[1]));
}

const document = {
  getElementById(id) {
    if (!elements.has(id)) { missingIds.push(id); return makeElement(id); }
    return elements.get(id);
  },
  createElement: () => makeElement('__created__'),
};

// ── Stubbed browser globals ────────────────────────────────────────────────
let socket = null;
const frames = [];

class FakeWebSocket {
  static OPEN = 1;
  constructor(url){ this.url = url; this.readyState = 1; socket = this; }
  send(data){ frames.push(data); }
  close(){}
}

const rafQueue = [];
const sandbox = {
  document,
  WebSocket: FakeWebSocket,
  location: {hostname: '192.168.4.1'},
  window: {devicePixelRatio: 2},
  requestAnimationFrame: cb => rafQueue.push(cb),
  setTimeout: () => 0,
  console,
  Math, JSON, isFinite, Infinity,
};
sandbox.globalThis = sandbox;

// ── Run ────────────────────────────────────────────────────────────────────
const failures = [];
vm.createContext(sandbox);
try {
  vm.runInContext(script, sandbox, {filename: 'web/index.html'});
} catch (e) {
  failures.push('script threw while loading: ' + e.stack);
}

if (!socket) failures.push('the page never opened a WebSocket');
else {
  socket.onopen?.();
  // Two frames 50 ms apart, so the history has enough points to draw a series.
  socket.onmessage({data: frameJson});
  const second = JSON.parse(frameJson);
  second.t += 50;
  socket.onmessage({data: JSON.stringify(second)});
  // And a malformed one, which must be ignored rather than kill the page.
  socket.onmessage({data: '{"t":not json'});
}

for (let i = 0; i < 3 && rafQueue.length; i++) {
  const pending = rafQueue.splice(0, rafQueue.length);
  for (const cb of pending) {
    try { cb(); } catch (e) { failures.push('render threw: ' + e.stack); }
  }
}

// ── Assertions ─────────────────────────────────────────────────────────────
const text = id => elements.get(id)?.textContent;
const expect = (id, want) => {
  const got = text(id);
  if (got !== want) failures.push(`#${id} = ${JSON.stringify(got)}, expected ${JSON.stringify(want)}`);
};

if (missingIds.length) {
  failures.push('getElementById for ids not in the page: ' + [...new Set(missingIds)].join(', '));
}

// The canonical frame from test/test_telemetry has known values, so it gets
// exact assertions. A frame captured live from the board does not, so it is
// checked structurally instead -- every readout must still be populated.
const canonical = JSON.parse(frameJson).t === 1234567;

if (canonical) {
  expect('r-roll', '9.9°');
  expect('r-pitch', '-15.4°');
  expect('r-yaw', '114.5°');
  expect('r-rest', 'yes');

  // Position and velocity must carry their 3-sigma envelope.
  expect('r-pe', '12.50 ±4.50');
  expect('r-pu', '1.75 ±9.00');
  expect('r-ve', '0.50 ±0.75');
  expect('r-sat', 9);

  // Header chips reflect health.
  expect('c-status', 'normal');
  expect('c-gps', 'gps fix 9');
  expect('c-rate', '200 Hz');

  // Sliders seeded from the device's own params, not the page's defaults.
  expect('v-sigma_accel', '0.350 m/s²/√Hz');
  expect('v-tau_mag', '9.00 s');
  if (!elements.get('k-zupt')?.checked) failures.push('#k-zupt was not seeded from params');
} else {
  console.log('live frame -- checking structurally');
  const populated = ['r-roll','r-pitch','r-yaw','r-rest','r-pe','r-pn','r-pu',
                     'r-ve','r-vn','r-vu','r-sat','r-ba','r-gbias',
                     'c-status','c-gps','c-rate','v-sigma_accel','v-tau_mag'];
  for (const id of populated) {
    const got = text(id);
    if (got === '' || got === undefined || got === '—' || String(got).includes('NaN')) {
      failures.push(`#${id} was not populated from the live frame (got ${JSON.stringify(got)})`);
    }
  }
  console.log('  rpy =', text('r-roll'), text('r-pitch'), text('r-yaw'),
              '| pos E =', text('r-pe'), '| sats =', text('r-sat'),
              '| status =', text('c-status'));
}

if (drawCalls < 100) failures.push(`only ${drawCalls} canvas draw calls -- panels did not render`);

if (failures.length) {
  console.error('FAIL\n  ' + failures.join('\n  '));
  process.exit(1);
}
console.log(`dashboard OK -- ${elements.size} elements, ${drawCalls} draw calls`);
