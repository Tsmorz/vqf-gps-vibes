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
// Usage: node tools/check_dashboard.mjs <frame.json> [spectrum.json]
//
// The spectrum frame is a second message type on the same socket, so it comes
// from the same encoder and the same test, in its own file.

import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const html = readFileSync(new URL('../web/index.html', import.meta.url), 'utf8');
const frameJson = readFileSync(process.argv[2], 'utf8').trim();
const spectrumJson = process.argv[3] ? readFileSync(process.argv[3], 'utf8').trim() : null;

// The page now carries two <script> blocks: a tiny theme-init snippet at the
// top of <head> (so a stored light/dark choice never flashes the wrong
// palette) and the real application script at the bottom of <body>. The
// latter is the one this harness runs; it always opens with 'use strict'.
const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);
const script = scripts.find(s => s.trimStart().startsWith("'use strict'"));
if (!script) throw new Error('no application <script> block found in web/index.html');

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
    setAttribute(name, v){ this['_attr_' + name] = v; },
    getAttribute(name){ return this['_attr_' + name] ?? null; },
    dataset: {},
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
  documentElement: makeElement('html'),
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

// The dark theme's own custom properties, standing in for a real cascade --
// there is no CSS engine in this sandbox, so getComputedStyle can't resolve
// :root's declarations itself. Good enough for a smoke test: the page's
// theme code only needs *some* well-formed value back, never a themed one.
const rootVars = {
  '--x': '#d95926', '--y': '#199e70', '--z': '#3987e5',
  '--est': '#d95926', '--gps': '#3987e5', '--baro': '#199e70',
  '--well': '#0b0e14', '--grid': '#161d27', '--axis': '#2a3340',
  '--text': '#e8eef5', '--dim': '#94a1b1', '--faint': '#65707f',
};

const rafQueue = [];
const sandbox = {
  document,
  WebSocket: FakeWebSocket,
  location: {hostname: '192.168.4.1'},
  window: {devicePixelRatio: 2},
  requestAnimationFrame: cb => rafQueue.push(cb),
  setTimeout: () => 0,
  getComputedStyle: () => ({getPropertyValue: name => rootVars[name] ?? ''}),
  matchMedia: () => ({matches: false, addEventListener(){}, removeEventListener(){}}),
  localStorage: {
    _store: new Map(),
    getItem(k){ return this._store.has(k) ? this._store.get(k) : null; },
    setItem(k, v){ this._store.set(k, String(v)); },
    removeItem(k){ this._store.delete(k); },
  },
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
  // The spectrum rides the same socket as its own message type.
  if (spectrumJson) socket.onmessage({data: spectrumJson});
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

  // The trust the filter actually applied, which is the point of showing it:
  // it must track the frame, not the slider the frame's sigma knob carries.
  const gps = JSON.parse(frameJson).gps;
  expect('r-gsig', `±${gps.sig.toFixed(1)} m`);
  expect('r-grej', gps.rej);

  // Header chips reflect health.
  expect('c-status', 'normal');
  expect('c-gps', 'gps fix 9');
  expect('c-rate', '200 Hz');

  // Sliders seeded from the device's own params, not the page's defaults.
  // Derived from the frame rather than hardcoded: these are firmware defaults
  // that change whenever the filter is re-tuned, and a literal here turns every
  // retune into a spurious failure — which is exactly what it just did.
  const sent = JSON.parse(frameJson).params;
  expect('v-sigma_accel', `${sent.sigma_accel.toFixed(3)} m/s²/√Hz`);
  expect('v-tau_mag', `${sent.tau_mag.toFixed(2)} s`);
  if (!elements.get('k-zupt')?.checked) failures.push('#k-zupt was not seeded from params');

  // Sensor magnitudes and calibration state.
  expect('r-anorm', '9.80 m/s²');
  expect('r-mnorm', '48.9 µT');
  expect('r-magcal', 'calibrated');
  expect('btn-magcal', 'Calibrate magnetometer');

  // The aux rail defaults to on, so the button offers to cut it.
  expect('r-aux', 'on');
  expect('btn-aux', 'Power down GPS + baro');

  // The recorder is mid-stream in the frame, so the button offers to stop it.
  const rec = JSON.parse(frameJson).rec;
  expect('r-rec', rec.on ? 'recording' : 'idle');
  expect('r-rec-n', rec.n.toLocaleString());
  expect('r-rec-drop', rec.drop.toLocaleString());
  expect('btn-rec', rec.on ? 'Stop recording' : 'Start recording');

  // Range dropdowns seeded from the device rather than left on whichever
  // option the page happened to list first.
  const ranges = JSON.parse(frameJson).params;
  for (const [id, key] of [['r-accel_range_g', 'accel_range_g'],
                           ['r-gyro_range_dps', 'gyro_range_dps'],
                           ['r-mag_range_gauss', 'mag_range_gauss']]) {
    const got = elements.get(id)?.value;
    if (got !== String(ranges[key])) {
      failures.push(`#${id} = ${JSON.stringify(got)}, expected ${JSON.stringify(String(ranges[key]))}`);
    }
  }

  // A panel whose source is on must be on screen...
  for (const id of ['sec-pos3d', 'sec-pos', 'sec-vel', 'sec-baro']) {
    if (elements.get(id)?.hidden) failures.push(`#${id} was hidden while its source is enabled`);
  }

  // ...and must go when that source is switched off. Position and velocity are
  // still being propagated from dead reckoning at this point, so nothing but
  // this rule stops the page presenting them as measurements.
  const off = JSON.parse(frameJson);
  off.t += 100;
  off.params.gps_enabled = 0;
  off.params.baro_enabled = 0;
  socket.onmessage({data: JSON.stringify(off)});

  for (const id of ['sec-pos3d', 'sec-pos', 'sec-vel', 'sec-baro']) {
    if (!elements.get(id)?.hidden) failures.push(`#${id} was still shown with its source off`);
  }
  if (!elements.get('k-gvel')?.disabled) {
    failures.push('#k-gvel stayed enabled with GPS fusion off');
  }

  // ── Spectrum panel ───────────────────────────────────────────────────────
  // The analyser's state is echoed in the telemetry frame, so the panel and
  // its source dropdown must follow the device rather than the page defaults.
  const specState = JSON.parse(frameJson).spec;
  if (!specState) {
    failures.push('the telemetry frame carries no "spec" block');
  } else {
    if (elements.get('spec-body')?.hidden) {
      failures.push('#spec-body was hidden while the analyser is switched on');
    }
    if (!elements.get('k-spec')?.checked) {
      failures.push('#k-spec was not seeded from the frame');
    }
    const src = elements.get('sel-spec-src')?.value;
    if (src !== specState.src) {
      failures.push(`#sel-spec-src = ${JSON.stringify(src)}, expected ${JSON.stringify(specState.src)}`);
    }
  }

  if (spectrumJson) {
    // Derived from the frame, not hardcoded: the window length and the bin
    // values are firmware constants, and a literal here would turn any change
    // to them into a spurious failure.
    const s = JSON.parse(spectrumJson);
    const df = s.fs / s.n;
    // Whichever bin is loudest at or above the page's 5 Hz default floor.
    const first = Math.max(1, Math.ceil(5 / df));
    let peakBin = -1, peakValue = 0;
    for (const axis of s.bins) {
      for (let i = first; i < axis.length; i++) {
        if (axis[i] > peakValue) { peakValue = axis[i]; peakBin = i; }
      }
    }
    const unit = {accel: 'm/s²', gyro: 'rad/s'}[s.src];

    expect('r-spec-fs', s.fs.toFixed(1) + ' Hz');
    expect('r-spec-res', df.toFixed(2) + ' Hz');
    // The dominant readout comes off the peak hold, the live one off this
    // frame; with a single frame seeding the hold, both must agree.
    expect('r-spec-f', (peakBin * df).toFixed(1) + ' Hz');
    expect('r-spec-live', (peakBin * df).toFixed(1) + ' Hz');
    if (!String(text('r-spec-a')).endsWith(unit)) {
      failures.push(`#r-spec-a = ${JSON.stringify(text('r-spec-a'))}, expected it to end in ${unit}`);
    }
  }
} else {
  console.log('live frame -- checking structurally');
  const populated = ['r-roll','r-pitch','r-yaw','r-rest','r-pe','r-pn','r-pu',
                     'r-ve','r-vn','r-vu','r-sat','r-gsig','r-ba','r-gbias',
                     'r-anorm','r-mnorm','r-magcal','r-aux',
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
  console.log('  |accel| =', text('r-anorm'), '| |mag| =', text('r-mnorm'),
              '| mag cal =', text('r-magcal'));
}

if (drawCalls < 100) failures.push(`only ${drawCalls} canvas draw calls -- panels did not render`);

if (failures.length) {
  console.error('FAIL\n  ' + failures.join('\n  '));
  process.exit(1);
}
console.log(`dashboard OK -- ${elements.size} elements, ${drawCalls} draw calls`);
