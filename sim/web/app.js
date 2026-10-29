'use strict';

// ArduSailor simulator UI: draws the simulated boat and what the firmware thinks is going on, and sends
// commands to the simulator (see sim/native/main.cpp for the command list).

const DEG = Math.PI / 180;
const EARTH_RADIUS = 6371000;
const BOAT_LENGTH = 0.9;  // m, for drawing
const HISTORY_SECONDS = 60;

const state = {
  info: null,
  frame: null,
  origin: [0, 0],
  waypoints: [],        // [lat, lon, x, y]
  params: {},           // key -> {info, input, range}
  tunables: {},         // name -> {info, input}
  track: [],
  gpsFixes: [],
  history: [],
  view: { cx: 0, cy: 0, scale: 6, fitted: false },
  drag: null,
  pidDirty: false,
};

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------- helpers

let colors = {};
function readColors() {
  const s = getComputedStyle(document.documentElement);
  for (const name of ['bg', 'panel', 'text', 'muted', 'line', 'accent', 'water', 'water-line', 'crest', 'hull',
    'hull-edge', 'sail', 'sail-edge', 'boat-true', 'fw', 'req', 'rudder', 'wind', 'gps', 'target', 'warn']) {
    colors[name] = s.getPropertyValue('--' + name).trim();
  }
}
readColors();
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', readColors);

const wrap360 = (a) => ((a % 360) + 360) % 360;
const angleDiff = (from, to) => ((to - from + 540) % 360) - 180;
const finite = (v) => typeof v === 'number' && isFinite(v);
const fmt = (v, digits = 1, unit = '') => finite(v) ? v.toFixed(digits) + unit : '–';
const fmtDeg = (v, digits = 0) => finite(v) ? wrap360(v).toFixed(digits) + '°' : '–';

function localToLatLon(x, y) {
  const lat = state.origin[0] + y / EARTH_RADIUS / DEG;
  const lon = state.origin[1] + x / (EARTH_RADIUS * Math.cos(state.origin[0] * DEG)) / DEG;
  return [lat, lon];
}

function tunableValue(name, fallback) {
  const t = state.tunables[name];
  return t ? Number(t.input.value) : fallback;
}

function paramValue(key, fallback) {
  const p = state.params[key];
  return p ? Number(p.input.value) : fallback;
}

function send(line) {
  return fetch('/api/command', { method: 'POST', body: line }).catch(() => {});
}

function post(path) {
  return fetch(path, { method: 'POST' }).catch(() => {});
}

function setupCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(rect.width * dpr)), h = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w: rect.width, h: rect.height };
}

function arrow(ctx, x1, y1, x2, y2, head = 8) {
  const a = Math.atan2(y2 - y1, x2 - x1);
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - head * Math.cos(a - 0.4), y2 - head * Math.sin(a - 0.4));
  ctx.lineTo(x2 - head * Math.cos(a + 0.4), y2 - head * Math.sin(a + 0.4));
  ctx.closePath();
  ctx.fill();
}

// hull outline, bow towards -y, length L, in the current transform
function hullPath(ctx, L) {
  const B = 0.22 * L;
  ctx.beginPath();
  ctx.moveTo(0, -L / 2);
  ctx.bezierCurveTo(B * 0.55, -L * 0.33, B * 0.55, -L * 0.05, B * 0.5, L * 0.15);
  ctx.lineTo(B * 0.32, L / 2);
  ctx.lineTo(-B * 0.32, L / 2);
  ctx.lineTo(-B * 0.5, L * 0.15);
  ctx.bezierCurveTo(-B * 0.55, -L * 0.05, -B * 0.55, -L * 0.33, 0, -L / 2);
  ctx.closePath();
}

// ---------------------------------------------------------------- connection

function connect() {
  const es = new EventSource('/api/events');
  es.onopen = () => setStatus('connected', 'ok');
  es.onerror = () => setStatus('reconnecting…', 'bad');
  es.onmessage = (e) => {
    let msg;
    try { msg = JSON.parse(e.data); } catch (err) { return; }
    handle(msg);
  };
}

function setStatus(text, cls) {
  const s = $('status');
  s.textContent = text;
  s.className = 'status ' + cls;
}

function handle(msg) {
  switch (msg.type) {
    case 'info': onInfo(msg); break;
    case 'frame': onFrame(msg); break;
    case 'serial': consoleLine(msg.text); break;
    case 'event': consoleLine('» ' + msg.text, 'event'); break;
    case 'param': showParam(msg.key, msg.value); break;
    case 'tunable': showTunable(msg.name, msg.value); break;
    case 'waypoints': state.waypoints = msg.list; renderWaypoints(); break;
  }
}

function onInfo(info) {
  state.info = info;
  state.origin = info.origin;
  state.waypoints = info.waypoints;
  state.track = [];
  state.gpsFixes = [];
  state.history = [];
  buildParams(info.params);
  buildTunables(info.tunables);
  renderWaypoints();
  if (!state.view.fitted) fitView();
}

function onFrame(f) {
  const last = state.frame;
  state.frame = f;
  const b = f.boat;
  if (last && f.t < last.t) {  // the firmware restarted
    state.track = [];
    state.history = [];
  }
  const tail = state.track[state.track.length - 1];
  if (!tail || Math.hypot(b.x - tail[0], b.y - tail[1]) > 0.05) state.track.push([b.x, b.y]);
  if (state.track.length > 6000) state.track.splice(0, state.track.length - 6000);
  if (finite(f.fw.gps_x)) {
    const g = state.gpsFixes[state.gpsFixes.length - 1];
    if (!g || g[0] !== f.fw.gps_x || g[1] !== f.fw.gps_y) state.gpsFixes.push([f.fw.gps_x, f.fw.gps_y]);
    if (state.gpsFixes.length > 120) state.gpsFixes.shift();
  }
  state.history.push({ t: f.t, heading: b.heading, compass: f.fw.heading, requested: f.fw.requested, rudder: b.rudder });
  while (state.history.length && state.history[0].t < f.t - HISTORY_SECONDS) state.history.shift();
  updateHeader(f);
  updatePid(f.fw);
}

// ---------------------------------------------------------------- header, speed, buttons

function updateHeader(f) {
  const t = Math.floor(f.t);
  $('sim-time').textContent = `${Math.floor(t / 60)}:${String(t % 60).padStart(2, '0')}`;
  $('realtime').textContent = f.paused ? 'paused' : (finite(f.realtime) ? `${f.realtime.toFixed(1)}× real time` : '');
  for (const btn of document.querySelectorAll('#speed button')) {
    if (btn.hasAttribute('data-pause')) btn.classList.toggle('active', f.paused);
    else btn.classList.toggle('active', !f.paused && Number(btn.dataset.speed) === f.speed);
  }
}

for (const btn of document.querySelectorAll('#speed button')) {
  btn.addEventListener('click', () => {
    if (btn.hasAttribute('data-pause')) {
      send(`pause ${state.frame && state.frame.paused ? 0 : 1}`);
    } else {
      send(`speed ${btn.dataset.speed}\npause 0`);
    }
  });
}

$('reset-boat').addEventListener('click', () => { send('reset'); state.track = []; });
$('restart').addEventListener('click', () => post('/api/restart'));
$('defaults').addEventListener('click', () => {
  if (confirm('Forget every setting changed here and reboot the firmware?')) send('defaults');
});
$('erase').addEventListener('click', () => {
  if (confirm('Erase the EEPROM (stored PID gains and compass calibration) and reboot?')) post('/api/erase');
});
$('clear-track').addEventListener('click', () => { state.track = []; state.gpsFixes = []; });
$('fit-course').addEventListener('click', () => { $('follow').checked = false; fitView(); });
$('cal-tumble').addEventListener('click', () => send('serial mc\ntumble 48'));
$('cal-circle').addEventListener('click', () => send('serial mo'));

// ---------------------------------------------------------------- PID

for (const id of ['kp', 'ki', 'kd']) {
  $(id).addEventListener('input', () => { state.pidDirty = true; $(id).classList.add('changed'); });
}
$('pid-apply').addEventListener('click', () => {
  send(`pid ${Number($('kp').value)} ${Number($('ki').value)} ${Number($('kd').value)}`);
  state.pidDirty = false;
  for (const id of ['kp', 'ki', 'kd']) $(id).classList.remove('changed');
});
$('autotune').addEventListener('change', (e) => send(`autotune ${e.target.checked ? 1 : 0}`));

function updatePid(fw) {
  if (!state.pidDirty) {
    for (const k of ['kp', 'ki', 'kd']) {
      if (document.activeElement !== $(k)) $(k).value = +fw[k].toPrecision(4);
    }
  }
  if (document.activeElement !== $('autotune')) $('autotune').checked = fw.autotune;
}

// ---------------------------------------------------------------- simulator parameters and firmware settings

function debounce(fn, ms) {
  let timer;
  return (...args) => { clearTimeout(timer); timer = setTimeout(() => fn(...args), ms); };
}

function buildParams(params) {
  const root = $('param-groups');
  const open = new Set([...root.querySelectorAll('details[open] summary')].map((s) => s.textContent));
  root.innerHTML = '';
  state.params = {};
  const groups = {};
  for (const p of params) (groups[p.group] = groups[p.group] || []).push(p);
  for (const [group, list] of Object.entries(groups)) {
    const det = document.createElement('details');
    if (open.has(group) || (!open.size && (group === 'Wind' || group === 'Waves'))) det.open = true;
    det.innerHTML = `<summary>${group}</summary>`;
    for (const p of list) {
      const row = document.createElement('div');
      row.className = 'param';
      row.title = `${p.key}, default ${p.default}`;
      const label = document.createElement('label');
      label.innerHTML = `${p.label} <span class="unit">${p.unit}</span>`;
      const input = document.createElement('input');
      input.type = 'number';
      Object.assign(input, { min: p.min, max: p.max, step: p.step, value: p.value });
      const range = document.createElement('input');
      range.type = 'range';
      Object.assign(range, { min: p.min, max: p.max, step: p.step, value: p.value });
      const push = debounce((v) => send(`set ${p.key} ${v}`), 120);
      const changed = (v) => input.classList.toggle('changed', Math.abs(v - p.default) > 1e-9);
      range.addEventListener('input', () => { input.value = range.value; changed(Number(range.value)); push(range.value); keepWavesRealistic(); });
      input.addEventListener('change', () => { range.value = input.value; changed(Number(input.value)); push(input.value); keepWavesRealistic(); });
      row.append(label, input, range);
      det.append(row);
      state.params[p.key] = { info: p, input, range, push, changed };
      changed(p.value);
    }
    if (group === 'Waves') {
      const note = document.createElement('p');
      note.className = 'note';
      note.textContent = 'The period goes up with the height when it has to, so the waves are no steeper than 1 in 10 (real ones break at about 1 in 7).';
      det.append(note);
    }
    root.append(det);
  }
}

// Steeper than this (height to length) isn't a wave that could exist for long: they break at about 1 in 7.
const MAX_WAVE_STEEPNESS = 1 / 10;

// a higher wave needs a longer one, and in deep water length = g T^2 / 2 pi
function keepWavesRealistic() {
  const h = state.params.wave_height, t = state.params.wave_period;
  if (!h || !t) return;
  const step = Number(t.info.step) || 0.1;
  const shortest = Math.sqrt(2 * Math.PI * (Number(h.input.value) / MAX_WAVE_STEEPNESS) / 9.81);
  const period = +Math.min(t.info.max, Math.ceil(shortest / step - 1e-9) * step).toFixed(3);
  if (Number(t.input.value) >= period) return;
  t.input.value = period;
  t.range.value = period;
  t.changed(period);
  t.push(period);
}

function showParam(key, value) {
  const p = state.params[key];
  if (!p) return;
  if (document.activeElement !== p.input) p.input.value = +value.toPrecision(6);
  if (document.activeElement !== p.range) p.range.value = value;
  p.input.classList.toggle('changed', Math.abs(value - p.info.default) > 1e-9);
}

function buildTunables(tunables) {
  const root = $('tunables');
  root.innerHTML = '';
  state.tunables = {};
  let file = null;
  for (const t of [...tunables].sort((a, b) => a.file.localeCompare(b.file))) {
    if (t.file !== file) {
      file = t.file;
      const h = document.createElement('div');
      h.className = 'file';
      h.textContent = file;
      root.append(h);
    }
    const row = document.createElement('div');
    row.className = 'tunable';
    row.title = `default ${t.default}, ${t.min} to ${t.max}`;
    const name = document.createElement('span');
    name.className = 'name';
    name.innerHTML = t.name + (t.restart ? '<span class="restart" title="takes effect after a reboot">&#8635;</span>' : '');
    const input = document.createElement('input');
    input.type = 'number';
    Object.assign(input, { min: t.min, max: t.max, step: t.int ? 1 : 'any', value: t.value });
    input.classList.toggle('changed', t.value !== t.default);
    input.addEventListener('change', () => send(`tune ${t.name} ${input.value}`));
    const comment = document.createElement('span');
    comment.className = 'comment';
    comment.textContent = t.comment;
    row.append(name, input, comment);
    root.append(row);
    state.tunables[t.name] = { info: t, input };
  }
}

function showTunable(name, value) {
  const t = state.tunables[name];
  if (!t) return;
  if (document.activeElement !== t.input) t.input.value = value;
  t.input.classList.toggle('changed', value !== t.info.default);
}

function renderWaypoints() {
  const target = state.frame ? state.frame.fw.target_wp : -1;
  $('waypoints').innerHTML = state.waypoints.map((w, i) =>
    `<tr class="${i === target ? 'target' : ''}"><td>${i}</td><td>${w[0].toFixed(6)}</td><td>${w[1].toFixed(6)}</td>` +
    `<td>${w[2].toFixed(0)}, ${w[3].toFixed(0)} m</td></tr>`).join('');
}

// ---------------------------------------------------------------- serial console

const consoleEl = $('console');
const DATA_LINE = /^\d{4}\.\d{2}[NS], /;

function consoleLine(text, cls) {
  const isData = !cls && DATA_LINE.test(text);
  const div = document.createElement('div');
  div.textContent = text;
  if (cls) div.className = cls;
  else if (isData) div.className = 'data';
  if (isData && $('hide-data').checked) div.hidden = true;
  const atBottom = consoleEl.scrollHeight - consoleEl.scrollTop - consoleEl.clientHeight < 30;
  consoleEl.append(div);
  while (consoleEl.childElementCount > 600) consoleEl.firstElementChild.remove();
  if (atBottom) consoleEl.scrollTop = consoleEl.scrollHeight;
}

$('hide-data').addEventListener('change', (e) => {
  for (const div of consoleEl.querySelectorAll('.data')) div.hidden = e.target.checked;
});
$('clear-console').addEventListener('click', () => { consoleEl.innerHTML = ''; });

function typeSerial(text) {
  send('serial ' + text.replace(/\\/g, '\\\\').replace(/\n/g, '\\n'));
  consoleLine('« ' + text.replace(/\n/g, '\\n'), 'sent');
}

$('serial-form').addEventListener('submit', (e) => {
  e.preventDefault();
  const text = $('serial-input').value + ($('serial-newline').checked ? '\n' : '');
  if (text) typeSerial(text);
  $('serial-input').value = '';
});
for (const btn of document.querySelectorAll('.keys button')) {
  btn.addEventListener('click', () => typeSerial(btn.dataset.serial));
}

// ---------------------------------------------------------------- instruments

function steeringFor(fw) {
  const name = (t) => (t > 0 ? 'starboard' : 'port');
  if (fw.recovering) return `beam reach on ${name(fw.on_tack)} tack, out of irons`;
  if (fw.gybing_round) return `bearing away to gybe onto ${name(-fw.tack)}, can't tack`;
  if (fw.tack_wanted) return `coming up to tack onto ${name(-fw.tack)}` + (fw.failed_tacks ? ` (${fw.failed_tacks} failed)` : '');
  if (fw.tack !== fw.on_tack) return `${fw.beating ? 'tacking' : 'tacking or gybing'} onto ${name(fw.tack)}`;
  return (fw.beating ? `beating, ${name(fw.tack)} tack` : `${name(fw.tack)} tack`) + (fw.rebuilding ? ', building speed' : '');
}

function renderInstruments() {
  const f = state.frame;
  if (!f) return;
  const b = f.boat, fw = f.fw, env = f.env;
  const gpsError = finite(fw.gps_x) ? Math.hypot(fw.gps_x - b.x, fw.gps_y - b.y) : NaN;
  const worldWind = wrap360(fw.heading + fw.wind);
  const rows = [
    ['Heading', fmtDeg(b.heading, 1), fmtDeg(fw.heading, 1)],
    ['Steering for', '', fmtDeg(fw.requested, 1) + ' ' + steeringFor(fw)],
    ['Waypoint', '', `#${fw.target_wp}: ${fmtDeg(fw.wp_heading)} ${fmt(fw.wp_distance, 1, ' m')}, ` +
      `${fmt(Math.abs(fw.cross_track), 1, ' m')} ${fw.cross_track >= 0 ? 'right' : 'left'} of the line`],
    ['Speed', `${fmt(b.speed, 2, ' kn')} (over ground ${fmt(b.sog, 2)})`, `GPS ${fmt(fw.gps_speed, 1, ' kn')} ${fmtDeg(fw.gps_course)}`],
    ['Position', `leeway ${fmt(b.leeway, 1, '°')}`, `GPS ${fmt(gpsError, 1, ' m')} off, ${fw.gps_age_ms} ms old`],
    ['Apparent wind', `${fmtDeg(b.awa)} ${fmt(b.aws, 1, ' kn')}`, `vane ${fmtDeg(fw.wind)} (now ${fmtDeg(b.vane)})`],
    ['True wind', `${fmtDeg(env.twd)} ${fmt(env.tws, 1, ' kn')}`, `${fmtDeg(worldWind)} (heading + vane)`],
    ['Heel / pitch', `${fmt(b.heel, 1, '°')} / ${fmt(b.pitch, 1, '°')}`, `${fmt(fw.roll, 1, '°')} / ${fmt(fw.pitch, 1, '°')}`],
    ['Rudder', `${fmt(b.rudder, 1, '°')} (servo ${fmt(b.rudder_servo, 0)}${b.rudder_powered ? ', on' : ''})`, `servo ${fw.rudder_cmd}, PID ${fmt(fw.pid_output, 1)}`],
    ['Sheet', `boom ${fmt(b.boom, 0, '°')}, limit ${fmt(b.sheet_limit, 0, '°')} (servo ${fmt(b.winch_servo, 0)}${b.winch_powered ? ', on' : ''})`, `servo ${fw.winch_cmd}, heel adj ${fmt(fw.heel_adjust, 1)}`],
    ['Sail', `${fmt(b.alpha, 0, '°')} attack, drive ${fmt(b.drive, 2, ' N')}, side ${fmt(b.side, 1, ' N')}`, ''],
    ['Loop', '', `${fmt(f.loop_ms, 0, ' ms')}, cycle ${fw.cycle}, battery ${fmt(fw.voltage, 2, ' V')}`],
  ];
  const flags = [fw.recovering && 'in irons, falling off to recover', fw.manual && 'MANUAL OVERRIDE', fw.remote && 'remote control', fw.autotune && 'PID autotune',
    fw.logging && 'serial logging', b.tumbling && 'boat out of the water, tumbling',
    fw.cal_steps > 0 && fw.cal_steps < fw.cal_total && `compass calibration ${fw.cal_steps}/${fw.cal_total}`].filter(Boolean);
  $('instruments').innerHTML =
    '<thead><tr><th></th><th>Simulator (truth)</th><th>Firmware</th></tr></thead><tbody>' +
    rows.map((r) => `<tr><th>${r[0]}</th><td>${r[1]}</td><td class="fw">${r[2]}</td></tr>`).join('') +
    (flags.length ? `<tr class="flags"><td colspan="3">${flags.join(' · ')}</td></tr>` : '') + '</tbody>';
  renderWaypoints();
}
setInterval(renderInstruments, 150);

// ---------------------------------------------------------------- map

const mapCanvas = $('map');

function fitView() {
  const pts = state.waypoints.map((w) => [w[2], w[3]]);
  if (state.info) pts.push([state.info.start[0], state.info.start[1]]);
  if (!pts.length) return;
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  const rect = mapCanvas.getBoundingClientRect();
  const spanX = Math.max(20, Math.max(...xs) - Math.min(...xs)), spanY = Math.max(20, Math.max(...ys) - Math.min(...ys));
  state.view.cx = (Math.max(...xs) + Math.min(...xs)) / 2;
  state.view.cy = (Math.max(...ys) + Math.min(...ys)) / 2;
  state.view.scale = Math.max(0.5, Math.min(rect.width / (spanX * 1.4), rect.height / (spanY * 1.4)));
  state.view.fitted = rect.width > 0;
}

function toScreen(x, y, w, h) {
  const v = state.view;
  return [w / 2 + (x - v.cx) * v.scale, h / 2 - (y - v.cy) * v.scale];
}

function toWorld(sx, sy, w, h) {
  const v = state.view;
  return [v.cx + (sx - w / 2) / v.scale, v.cy - (sy - h / 2) / v.scale];
}

function niceSpacing(scale) {
  for (const s of [0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500]) if (s * scale >= 60) return s;
  return 1000;
}

function drawMap() {
  const { ctx, w, h } = setupCanvas(mapCanvas);
  const f = state.frame;
  if (!state.view.fitted) fitView();
  if (f && $('follow').checked && !state.drag) {
    state.view.cx = f.boat.x;
    state.view.cy = f.boat.y;
  }
  const scale = state.view.scale;

  ctx.fillStyle = colors.water;
  ctx.fillRect(0, 0, w, h);

  // grid
  const grid = niceSpacing(scale);
  const [x0, y1] = toWorld(0, 0, w, h), [x1, y0] = toWorld(w, h, w, h);
  ctx.strokeStyle = colors['water-line'];
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = Math.ceil(x0 / grid) * grid; x <= x1; x += grid) {
    const [sx] = toScreen(x, 0, w, h);
    ctx.moveTo(sx, 0); ctx.lineTo(sx, h);
  }
  for (let y = Math.ceil(y0 / grid) * grid; y <= y1; y += grid) {
    const [, sy] = toScreen(0, y, w, h);
    ctx.moveTo(0, sy); ctx.lineTo(w, sy);
  }
  ctx.stroke();

  if (f) drawWaves(ctx, w, h, f);
  drawRoute(ctx, w, h, f);

  // where the boat has been, and the GPS fixes the firmware got
  ctx.strokeStyle = colors['boat-true'];
  ctx.globalAlpha = 0.55;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  state.track.forEach((p, i) => {
    const [sx, sy] = toScreen(p[0], p[1], w, h);
    if (i) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
  });
  ctx.stroke();
  ctx.globalAlpha = 1;
  ctx.fillStyle = colors.gps;
  for (const g of state.gpsFixes) {
    const [sx, sy] = toScreen(g[0], g[1], w, h);
    ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
  }

  if (f) drawMapBoat(ctx, w, h, f);
  drawMapLegend(ctx, w, h, f, grid);
}

function drawWaves(ctx, w, h, f) {
  const env = f.env;
  if (!(env.wave_height > 0.005) || !(env.wave_length > 0)) return;
  const to = (env.wave_dir + 180) * DEG;
  const d = [Math.sin(to), Math.cos(to)];
  const k = 2 * Math.PI / env.wave_length;
  const spacing = env.wave_length * state.view.scale;
  if (spacing < 7) return;
  const sBoat = f.boat.x * d[0] + f.boat.y * d[1];
  const offset = (f.env.wave_phase - Math.PI / 2) / k;
  const corners = [[0, 0], [w, 0], [0, h], [w, h]].map(([sx, sy]) => {
    const p = toWorld(sx, sy, w, h);
    return p[0] * d[0] + p[1] * d[1];
  });
  const lo = Math.min(...corners), hi = Math.max(...corners);
  const n0 = Math.ceil((sBoat + offset - hi) / env.wave_length), n1 = Math.floor((sBoat + offset - lo) / env.wave_length);
  ctx.strokeStyle = colors.crest;
  ctx.globalAlpha = Math.min(0.9, 0.25 + env.wave_height * 4);
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  const reach = Math.hypot(w, h) / state.view.scale;
  for (let n = n0; n <= n1 && n - n0 < 400; n++) {
    const s = sBoat + offset - n * env.wave_length;
    const cx = d[0] * s, cy = d[1] * s;  // a point on the crest
    const [ax, ay] = toScreen(cx - d[1] * reach, cy + d[0] * reach, w, h);
    const [bx, by] = toScreen(cx + d[1] * reach, cy - d[0] * reach, w, h);
    ctx.moveTo(ax, ay);
    ctx.lineTo(bx, by);
  }
  ctx.stroke();
  ctx.globalAlpha = 1;
}

function drawRoute(ctx, w, h, f) {
  const wps = state.waypoints;
  if (!wps.length) return;
  const radius = tunableValue('GET_WITHIN', 5);
  const target = f ? f.fw.target_wp : -1;
  ctx.setLineDash([5, 5]);
  ctx.strokeStyle = colors.muted;
  ctx.lineWidth = 1;
  ctx.beginPath();
  wps.forEach((p, i) => {
    const [sx, sy] = toScreen(p[2], p[3], w, h);
    if (i) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
  });
  ctx.stroke();
  ctx.setLineDash([]);
  wps.forEach((p, i) => {
    const [sx, sy] = toScreen(p[2], p[3], w, h);
    const isTarget = i === target;
    ctx.strokeStyle = isTarget ? colors.target : colors.req;
    ctx.fillStyle = isTarget ? colors.target : colors.req;
    ctx.lineWidth = isTarget ? 2 : 1;
    ctx.beginPath();
    ctx.arc(sx, sy, Math.max(4, radius * state.view.scale), 0, 2 * Math.PI);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(sx, sy, 3, 0, 2 * Math.PI);
    ctx.fill();
    ctx.font = '600 12px system-ui, sans-serif';
    ctx.fillText(String(i), sx + 6, sy - 6);
  });
  if (f && wps[target] && finite(f.fw.leg_x)) drawCorridor(ctx, w, h, f, wps[target]);
  if (f && wps[target]) {
    const [bx, by] = toScreen(f.boat.x, f.boat.y, w, h);
    const [tx, ty] = toScreen(wps[target][2], wps[target][3], w, h);
    ctx.strokeStyle = colors.target;
    ctx.globalAlpha = 0.5;
    ctx.setLineDash([2, 4]);
    ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(tx, ty); ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  }
}

// the line the firmware is sailing this leg along, and the corridor it beats up, tacking at the edges
function drawCorridor(ctx, w, h, f, wp) {
  const ax = f.fw.leg_x, ay = f.fw.leg_y, bx = wp[2], by = wp[3];
  const len = Math.hypot(bx - ax, by - ay);
  if (len < 1) return;
  const nx = (by - ay) / len * f.fw.corridor, ny = -(bx - ax) / len * f.fw.corridor;  // to the right of the line
  const corners = [[ax + nx, ay + ny], [bx + nx, by + ny], [bx - nx, by - ny], [ax - nx, ay - ny]].map((p) => toScreen(p[0], p[1], w, h));
  ctx.fillStyle = colors.target;
  ctx.globalAlpha = f.fw.beating ? 0.08 : 0.04;
  ctx.beginPath();
  corners.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.closePath();
  ctx.fill();
  ctx.strokeStyle = colors.target;
  ctx.globalAlpha = f.fw.beating ? 0.45 : 0.2;
  ctx.lineWidth = 1;
  ctx.setLineDash([6, 4]);
  for (const [i, j] of [[0, 1], [3, 2]]) {
    ctx.beginPath(); ctx.moveTo(...corners[i]); ctx.lineTo(...corners[j]); ctx.stroke();
  }
  ctx.setLineDash([]);
  ctx.globalAlpha = 1;
}

function drawMapBoat(ctx, w, h, f) {
  const b = f.boat;
  const [sx, sy] = toScreen(b.x, b.y, w, h);
  const L = Math.max(BOAT_LENGTH * state.view.scale, 30);

  // where the firmware is steering (dashed) and where its compass says it's pointing (orange)
  const ray = (deg, len, color, dash) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.setLineDash(dash);
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(sx + len * Math.sin(deg * DEG), sy - len * Math.cos(deg * DEG));
    ctx.stroke();
    ctx.setLineDash([]);
  };
  ray(f.fw.requested, L * 2.6, colors.req, [6, 4]);
  ray(f.fw.heading, L * 1.9, colors.fw, []);

  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(b.heading * DEG);
  hullPath(ctx, L);
  ctx.fillStyle = colors.hull;
  ctx.fill();
  ctx.strokeStyle = colors['hull-edge'];
  ctx.lineWidth = 1.2;
  ctx.stroke();
  const mastY = -L / 2 + 0.36 * L;
  ctx.strokeStyle = colors['sail-edge'];
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, mastY);
  ctx.lineTo(0.42 * L * Math.sin(b.boom * DEG), mastY + 0.42 * L * Math.cos(b.boom * DEG));
  ctx.stroke();
  ctx.restore();
}

function drawMapLegend(ctx, w, h, f, grid) {
  ctx.font = '12px system-ui, sans-serif';
  ctx.fillStyle = colors.text;
  // scale bar
  const px = grid * state.view.scale;
  ctx.strokeStyle = colors.text;
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(12, h - 14); ctx.lineTo(12 + px, h - 14); ctx.stroke();
  ctx.fillText(`${grid} m`, 16 + px, h - 10);
  if (!f) return;
  // true wind, blowing from its direction
  const cx = 42, cy = 44, r = 26;
  const from = f.env.twd * DEG;
  ctx.strokeStyle = colors.wind;
  ctx.fillStyle = colors.wind;
  ctx.lineWidth = 2.5;
  arrow(ctx, cx + r * Math.sin(from), cy - r * Math.cos(from), cx - r * Math.sin(from), cy + r * Math.cos(from), 10);
  ctx.fillStyle = colors.text;
  ctx.fillText(`wind ${f.env.tws.toFixed(1)} kn from ${wrap360(f.env.twd).toFixed(0)}°`, cx + 34, cy - 4);
  if (f.env.wave_height > 0.005) {
    ctx.fillStyle = colors.muted;
    ctx.fillText(`waves ${(f.env.wave_height * 100).toFixed(0)} cm, ${f.env.wave_length.toFixed(1)} m apart`, cx + 34, cy + 12);
  }
  ctx.fillStyle = colors.text;
  ctx.textAlign = 'right';
  ctx.fillText('N ↑', w - 12, 18);
  ctx.textAlign = 'left';
}

// panning, zooming, dragging waypoints
mapCanvas.addEventListener('mousedown', (e) => {
  const rect = mapCanvas.getBoundingClientRect();
  const mx = e.clientX - rect.left, my = e.clientY - rect.top;
  let hit = -1;
  state.waypoints.forEach((p, i) => {
    const [sx, sy] = toScreen(p[2], p[3], rect.width, rect.height);
    if (Math.hypot(sx - mx, sy - my) < 10) hit = i;
  });
  state.drag = { mx, my, waypoint: hit, moved: false };
  if (hit < 0) {
    $('follow').checked = false;
    mapCanvas.classList.add('dragging');
  }
});
window.addEventListener('mousemove', (e) => {
  const d = state.drag;
  if (!d) return;
  const rect = mapCanvas.getBoundingClientRect();
  const mx = e.clientX - rect.left, my = e.clientY - rect.top;
  if (d.waypoint >= 0) {
    const [x, y] = toWorld(mx, my, rect.width, rect.height);
    const [lat, lon] = localToLatLon(x, y);
    state.waypoints[d.waypoint] = [lat, lon, x, y];
    d.moved = true;
  } else {
    state.view.cx -= (mx - d.mx) / state.view.scale;
    state.view.cy += (my - d.my) / state.view.scale;
    d.mx = mx;
    d.my = my;
  }
});
window.addEventListener('mouseup', () => {
  const d = state.drag;
  state.drag = null;
  mapCanvas.classList.remove('dragging');
  if (d && d.waypoint >= 0 && d.moved) {
    const p = state.waypoints[d.waypoint];
    send(`wp ${d.waypoint} ${p[0].toFixed(8)} ${p[1].toFixed(8)}`);
  }
});
mapCanvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const rect = mapCanvas.getBoundingClientRect();
  const mx = e.clientX - rect.left, my = e.clientY - rect.top;
  const before = toWorld(mx, my, rect.width, rect.height);
  state.view.scale = Math.max(0.2, Math.min(400, state.view.scale * Math.exp(-e.deltaY * 0.0015)));
  if (!$('follow').checked) {
    const after = toWorld(mx, my, rect.width, rect.height);
    state.view.cx += before[0] - after[0];
    state.view.cy += before[1] - after[1];
  }
}, { passive: false });

// ---------------------------------------------------------------- boat from above (bow up)

function drawBoat() {
  const { ctx, w, h } = setupCanvas($('boat'));
  ctx.fillStyle = colors.water;
  ctx.fillRect(0, 0, w, h);
  const f = state.frame;
  if (!f) return;
  const b = f.boat, fw = f.fw;
  const cx = w / 2, cy = h / 2;
  const ring = Math.min(w, h) / 2 - 16;
  const L = Math.min(ring * 1.45, h * 0.7);

  // compass ring, turned so the bow is up
  ctx.strokeStyle = colors['water-line'];
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, ring, 0, 2 * Math.PI); ctx.stroke();
  ctx.font = '11px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (let d = 0; d < 360; d += 30) {
    const a = (d - b.heading) * DEG;
    const inner = d % 90 === 0 ? ring - 8 : ring - 4;
    ctx.strokeStyle = colors.muted;
    ctx.beginPath();
    ctx.moveTo(cx + inner * Math.sin(a), cy - inner * Math.cos(a));
    ctx.lineTo(cx + ring * Math.sin(a), cy - ring * Math.cos(a));
    ctx.stroke();
    if (d % 90 === 0) {
      ctx.fillStyle = d === 0 ? colors.target : colors.muted;
      ctx.fillText(['N', 'E', 'S', 'W'][d / 90], cx + (ring - 16) * Math.sin(a), cy - (ring - 16) * Math.cos(a));
    }
  }

  // headings: requested and the compass's idea of the bow, against the true bow (straight up)
  const tick = (rel, color, len, width, dash) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash || []);
    ctx.beginPath();
    ctx.moveTo(cx + (ring - len) * Math.sin(rel * DEG), cy - (ring - len) * Math.cos(rel * DEG));
    ctx.lineTo(cx + (ring + 6) * Math.sin(rel * DEG), cy - (ring + 6) * Math.cos(rel * DEG));
    ctx.stroke();
    ctx.setLineDash([]);
  };
  tick(angleDiff(b.heading, fw.requested), colors.req, 30, 2, [4, 3]);
  tick(angleDiff(b.heading, fw.heading), colors.fw, 22, 3);
  tick(0, colors['boat-true'], 22, 3);

  // wind arrows: true (solid) and apparent (outline), pointing the way the air moves
  const windArrow = (rel, color, len, fill) => {
    const a = rel * DEG;
    const x1 = cx + (ring - 4) * Math.sin(a), y1 = cy - (ring - 4) * Math.cos(a);
    const x2 = cx + (ring - 4 - len) * Math.sin(a), y2 = cy - (ring - 4 - len) * Math.cos(a);
    ctx.strokeStyle = color;
    ctx.fillStyle = fill ? color : colors.water;
    ctx.lineWidth = 2;
    arrow(ctx, x1, y1, x2, y2, 9);
    if (!fill) ctx.stroke();
  };
  windArrow(angleDiff(b.heading, f.env.twd), colors.wind, 44, true);
  windArrow(b.awa, colors.wind, 30, false);

  ctx.save();
  ctx.translate(cx, cy);

  // sheet: how far out the boom may go
  const mastY = -L / 2 + 0.36 * L;
  const boomLen = 0.42 * L;
  ctx.strokeStyle = colors.muted;
  ctx.setLineDash([2, 4]);
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(0, mastY, boomLen * 1.06, Math.PI / 2 - b.sheet_limit * DEG, Math.PI / 2 + b.sheet_limit * DEG);
  ctx.stroke();
  ctx.setLineDash([]);

  // hull
  hullPath(ctx, L);
  ctx.fillStyle = colors.hull;
  ctx.fill();
  ctx.strokeStyle = colors['hull-edge'];
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.strokeStyle = colors.muted;
  ctx.setLineDash([3, 3]);
  ctx.beginPath(); ctx.moveTo(0, -0.05 * L); ctx.lineTo(0, 0.12 * L); ctx.stroke();  // keel, below
  ctx.setLineDash([]);

  // rudder: where it is (solid) and where the firmware has sent the servo (dashed)
  const rudderLen = 0.14 * L;
  const blade = (deg, color, width, dash) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash || []);
    ctx.beginPath();
    ctx.moveTo(0, L / 2 - 2);
    ctx.lineTo(rudderLen * Math.sin(deg * DEG), L / 2 - 2 + rudderLen * Math.cos(deg * DEG));
    ctx.stroke();
    ctx.setLineDash([]);
  };
  const commanded = (fw.rudder_cmd - 90) * paramValue('rudder_ratio', 1);
  if (Math.abs(commanded - b.rudder) > 1) blade(commanded, colors.fw, 2, [3, 3]);
  blade(b.rudder, colors.rudder, 4);

  // sail: from the mast to the end of the boom, full to leeward unless it's luffing
  const beta = b.boom * DEG;
  const ex = boomLen * Math.sin(beta), ey = mastY + boomLen * Math.cos(beta);
  const side = b.boom >= 0 ? 1 : -1;
  ctx.fillStyle = colors.sail;
  ctx.strokeStyle = colors['sail-edge'];
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(0, mastY);
  if (b.alpha < 4) {
    for (let i = 1; i <= 12; i++) {  // luffing
      const s = i / 12;
      const wob = Math.sin(s * Math.PI * 3 + f.t * 25) * 0.03 * L * Math.sin(s * Math.PI);
      ctx.lineTo(ex * s + wob * Math.cos(beta), mastY + (ey - mastY) * s - wob * Math.sin(beta));
    }
    ctx.stroke();
  } else {
    const camber = Math.min(0.16, 0.06 + Math.abs(b.lift) / 40) * boomLen;
    const mx = ex / 2 + side * camber * Math.cos(beta), my = (mastY + ey) / 2 - side * camber * Math.sin(beta);
    ctx.quadraticCurveTo(mx, my, ex, ey);
    ctx.lineTo(0, mastY);
    ctx.fill();
    ctx.stroke();
  }
  ctx.strokeStyle = colors['hull-edge'];
  ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(0, mastY); ctx.lineTo(ex, ey); ctx.stroke();  // boom
  ctx.fillStyle = colors['hull-edge'];
  ctx.beginPath(); ctx.arc(0, mastY, 3.5, 0, 2 * Math.PI); ctx.fill();  // mast

  // masthead vane, pointing into the wind it reads; the firmware's averaged reading in orange
  const vane = (deg, color, len) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, mastY);
    ctx.lineTo(len * Math.sin(deg * DEG), mastY - len * Math.cos(deg * DEG));
    ctx.stroke();
  };
  vane(fw.wind, colors.fw, 0.2 * L);
  vane(b.vane, colors.wind, 0.15 * L);
  ctx.restore();

  ctx.textAlign = 'left';
  ctx.textBaseline = 'alphabetic';
  ctx.font = '12px system-ui, sans-serif';
  ctx.fillStyle = colors.text;
  const lines = [
    `heading ${fmtDeg(b.heading)}  compass ${fmtDeg(fw.heading)}`,
    `rudder ${fmt(b.rudder, 0, '°')}  boom ${fmt(Math.abs(b.boom), 0, '°')} ${b.boom >= 0 ? 'stbd' : 'port'}`,
    `${fmt(b.speed, 2, ' kn')}  heel ${fmt(b.heel, 0, '°')}`,
  ];
  lines.forEach((t, i) => ctx.fillText(t, 8, 16 + i * 15));
  ctx.fillStyle = colors.muted;
  ctx.font = '11px system-ui, sans-serif';
  ctx.fillText('true wind ▶ solid, apparent ▷ outline', 8, h - 8);
}

// ---------------------------------------------------------------- from astern

function drawStern() {
  const { ctx, w, h } = setupCanvas($('stern'));
  const f = state.frame;
  ctx.fillStyle = colors.panel;
  ctx.fillRect(0, 0, w, h);
  if (!f) return;
  const b = f.boat, env = f.env;
  const s = Math.min(h / 2.3, w / 1.6);  // pixels per metre
  const waterY = h * 0.66;

  // sea surface across the boat: the waves' slope seen from astern
  const rel = (env.wave_dir + 180 - b.heading) * DEG;
  const across = Math.abs(Math.sin(rel));
  const k = 2 * Math.PI / Math.max(0.1, env.wave_length) * across;
  const amp = env.wave_height / 2 * s;
  const towards = Math.sin(rel) >= 0 ? 1 : -1;  // waves running to starboard move right
  const surface = (x) => waterY - amp * Math.sin(env.wave_phase - towards * k * (x - w / 2) / s);
  ctx.fillStyle = colors.water;
  ctx.beginPath();
  ctx.moveTo(0, h);
  for (let x = 0; x <= w; x += 4) ctx.lineTo(x, surface(x));
  ctx.lineTo(w, h);
  ctx.closePath();
  ctx.fill();

  ctx.save();
  ctx.translate(w / 2, waterY - b.heave * s);
  ctx.rotate(b.heel * DEG);

  // rig: mast, and the sail seen edge-on from the masthead to the end of the boom
  const mastTop = -1.45 * s, boomY = -0.16 * s;
  const boomX = 0.38 * s * Math.sin(b.boom * DEG);
  ctx.fillStyle = colors.sail;
  ctx.strokeStyle = colors['sail-edge'];
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, mastTop);
  ctx.lineTo(boomX, boomY);
  ctx.lineTo(0, boomY);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.strokeStyle = colors['hull-edge'];
  ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(0, -0.07 * s); ctx.lineTo(0, mastTop); ctx.stroke();
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(0, boomY); ctx.lineTo(boomX, boomY); ctx.stroke();

  // hull section, fin and bulb (drawn a little larger than scale, so the hull reads next to the mast)
  ctx.scale(1.5, 1.5);
  const half = 0.11 * s;
  ctx.fillStyle = colors.hull;
  ctx.strokeStyle = colors['hull-edge'];
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(-half, -0.07 * s);
  ctx.lineTo(half, -0.07 * s);
  ctx.quadraticCurveTo(half * 1.02, 0.05 * s, 0, 0.07 * s);
  ctx.quadraticCurveTo(-half * 1.02, 0.05 * s, -half, -0.07 * s);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = colors['hull-edge'];
  ctx.fillRect(-0.008 * s, 0.06 * s, 0.016 * s, 0.32 * s);
  ctx.beginPath();
  ctx.ellipse(0, 0.4 * s, 0.035 * s, 0.03 * s, 0, 0, 2 * Math.PI);
  ctx.fill();
  ctx.restore();

  // the vertical, for reference
  ctx.strokeStyle = colors.muted;
  ctx.setLineDash([3, 5]);
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(w / 2, waterY - 1.5 * s); ctx.lineTo(w / 2, waterY + 0.45 * s); ctx.stroke();
  ctx.setLineDash([]);

  ctx.font = '12px system-ui, sans-serif';
  ctx.fillStyle = colors.text;
  ctx.fillText(`heel ${fmt(Math.abs(b.heel), 1, '°')} ${b.heel >= 0 ? 'to starboard' : 'to port'}`, 8, 16);
  ctx.fillStyle = colors.fw;
  ctx.fillText(`firmware roll ${fmt(f.fw.roll, 1, '°')}${f.fw.heel_adjust > 0 ? `, easing ${fmt(f.fw.heel_adjust, 0)}` : ''}`, 8, 31);
  ctx.fillStyle = colors.muted;
  ctx.textAlign = 'left';
  ctx.fillText('port', 8, h - 8);
  ctx.textAlign = 'right';
  ctx.fillText('starboard', w - 8, h - 8);
  ctx.textAlign = 'left';
}

// ---------------------------------------------------------------- heading chart

function drawChart() {
  const { ctx, w, h } = setupCanvas($('chart'));
  ctx.fillStyle = colors.panel;
  ctx.fillRect(0, 0, w, h);
  const hist = state.history;
  if (hist.length < 2) return;
  const left = 34, right = w - 8, top = 8;
  const split = h * 0.68, bottom = h - 16;
  const tEnd = hist[hist.length - 1].t, tStart = tEnd - HISTORY_SECONDS;
  const X = (t) => left + (t - tStart) / HISTORY_SECONDS * (right - left);
  const Y = (deg) => top + (1 - wrap360(deg) / 360) * (split - top - 6);
  const RY = (deg) => split + 6 + (1 - (Math.max(-45, Math.min(45, deg)) + 45) / 90) * (bottom - split - 6);

  ctx.font = '10px system-ui, sans-serif';
  ctx.fillStyle = colors.muted;
  ctx.strokeStyle = colors.line;
  ctx.lineWidth = 1;
  ctx.textAlign = 'right';
  for (const d of [0, 90, 180, 270, 360]) {
    ctx.beginPath(); ctx.moveTo(left, Y(d === 360 ? 359.999 : d)); ctx.lineTo(right, Y(d === 360 ? 359.999 : d)); ctx.stroke();
    ctx.fillText(String(d), left - 4, Y(d === 360 ? 359.999 : d) + 3);
  }
  for (const d of [-45, 0, 45]) {
    ctx.beginPath(); ctx.moveTo(left, RY(d)); ctx.lineTo(right, RY(d)); ctx.stroke();
    ctx.fillText((d > 0 ? '+' : '') + d, left - 4, RY(d) + 3);
  }
  ctx.textAlign = 'left';
  for (let t = Math.max(0, Math.ceil(tStart / 10) * 10); t <= tEnd; t += 10) {
    ctx.fillText(`${Math.floor(t / 60)}:${String(Math.floor(t % 60)).padStart(2, '0')}`, X(t) - 10, h - 3);
  }

  const series = (key, color, width, dash, y) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash);
    ctx.beginPath();
    let prev = null;
    for (const p of hist) {
      const v = p[key];
      if (!finite(v)) { prev = null; continue; }
      const x = X(p.t), yy = y(v);
      const wrapped = y === Y && Math.abs(wrap360(v) - wrap360(prev)) > 180;  // don't draw across 0/360
      if (prev !== null && !wrapped) ctx.lineTo(x, yy);
      else ctx.moveTo(x, yy);
      prev = v;
    }
    ctx.stroke();
    ctx.setLineDash([]);
  };
  series('requested', colors.req, 1.5, [5, 3], Y);
  series('compass', colors.fw, 1.5, [], Y);
  series('heading', colors['boat-true'], 2, [], Y);
  series('rudder', colors.rudder, 1.5, [], RY);
}

// ---------------------------------------------------------------- go

function frameLoop() {
  try {
    drawMap();
    drawBoat();
    drawStern();
    drawChart();
  } catch (err) {
    console.error(err);
  }
  requestAnimationFrame(frameLoop);
}

connect();
requestAnimationFrame(frameLoop);
