// 载具镜头演示主程序：自由驾驶 + 脚本场景（加速/连续弯道/侧滑）。
// 所有按钮都驱动 src/ 下同一套 FollowCamera 实现，指标实时累计，不用固定数据。
import { FollowCamera } from '../src/followCamera.js';
import { Vehicle } from '../src/vehicle.js';
import { SCENARIOS, createRun, runScenario, vehicleState } from '../src/trajectories.js';
import { summarize } from '../src/metrics.js';
import { obstaclesAlongPath, scatterObstacles } from '../src/scene.js';
import { lookAt, perspective, mul4 } from '../src/math3d.js';
import { createRenderer } from './renderer.js';

const $ = (id) => document.getElementById(id);
const view = $('view'), hud = $('hud'), map = $('map');
const renderer = createRenderer(view);
const hctx = hud.getContext('2d');
const mctx = map.getContext('2d');
const DEG = 180 / Math.PI;

let mode = 'manual';
let vehicle, camera, run = null;
let rows = [], obstacles = [], refPath = null;
let simTime = 0, summarized = false, lastSummary = null, lastRow = null;

function camOpts() {
  return {
    aspect: 16 / 9,
    lookAheadDist: parseFloat($('look').value),
    deadZone: { x: parseFloat($('dzx').value), y: parseFloat($('dzy').value) },
  };
}

function reset(toMode) {
  mode = toMode;
  vehicle = new Vehicle();
  camera = new FollowCamera(camOpts());
  rows = [];
  simTime = 0;
  summarized = false;
  lastSummary = null;
  run = null;
  if (mode === 'manual') {
    obstacles = scatterObstacles();
    refPath = null;
    $('summaryBody').textContent = '自由驾驶中：指标随驾驶实时累计。';
  } else {
    const sc = SCENARIOS[mode];
    // 参考路径仅用于摆放路侧障碍与俯视地图；镜头指标完全来自随后的实时仿真
    const refRows = runScenario(new Vehicle(), new FollowCamera(camOpts()), sc, 1 / 60);
    refPath = refRows.map((r) => r.veh.pos);
    obstacles = obstaclesAlongPath(refPath, 7, 6);
    run = createRun(vehicle, camera, sc);
    $('summaryBody').textContent = `${sc.name}场景运行中……`;
  }
  for (const b of document.querySelectorAll('#btns button')) {
    b.classList.toggle('on', b.dataset.mode === mode);
  }
}

// ---------- 输入 ----------
const keys = {};
addEventListener('keydown', (e) => {
  const k = e.key.toLowerCase();
  keys[k] = true;
  if (k === 'r') reset('manual');
  if (['arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' '].includes(k)) e.preventDefault();
});
addEventListener('keyup', (e) => { keys[e.key.toLowerCase()] = false; });
function manualControls() {
  return {
    throttle: (keys.w || keys.arrowup ? 1 : 0) - (keys.s || keys.arrowdown ? 1 : 0),
    steer: (keys.a || keys.arrowleft ? 1 : 0) - (keys.d || keys.arrowright ? 1 : 0),
  };
}

// ---------- UI 绑定 ----------
for (const b of document.querySelectorAll('#btns button')) {
  b.onclick = () => reset(b.dataset.mode);
}
for (const id of ['look', 'dzx', 'dzy']) {
  $(id).oninput = () => {
    $('lookVal').textContent = parseFloat($('look').value).toFixed(1);
    $('dzxVal').textContent = parseFloat($('dzx').value).toFixed(2);
    $('dzyVal').textContent = parseFloat($('dzy').value).toFixed(2);
  };
}
$('exportBtn').onclick = () => {
  const data = {
    mode, exportedAt: new Date().toISOString(),
    params: {
      lookAheadDist: camera.lookAheadDist,
      deadZone: { ...camera.deadZone },
      anchor: { ...camera.anchor },
      fovYDeg: camera.fovYDeg,
    },
    summary: lastSummary,
    rows,
  };
  const blob = new Blob([JSON.stringify(data)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `followcam_${mode}_${Math.round(simTime)}s.json`;
  a.click();
  URL.revokeObjectURL(a.href);
};

// ---------- 汇总 ----------
function summaryHtml(m) {
  const d = (x) => (x * DEG).toFixed(1);
  const f = (x) => x.toFixed(3);
  return `<table>
    <tr><td class="sectionTitle" colspan="2">构图误差（投影空间）</td></tr>
    <tr><td>平均 |e| / 最大 |e|</td><td>${f(m.composition.meanErr)} / ${f(m.composition.maxErr)}</td></tr>
    <tr><td>平均 ex / ey</td><td>${f(m.composition.meanErrX)} / ${f(m.composition.meanErrY)}</td></tr>
    <tr><td class="sectionTitle" colspan="2">镜头运动成本</td></tr>
    <tr><td>偏航 / 俯仰总行程</td><td>${d(m.motion.yawTravel)}° / ${d(m.motion.pitchTravel)}°</td></tr>
    <tr><td>最大角速度 偏航 / 俯仰</td><td>${d(m.motion.maxYawRate)}°/s / ${d(m.motion.maxPitchRate)}°/s</td></tr>
    <tr><td class="sectionTitle" colspan="2">留白与死区</td></tr>
    <tr><td>最小上方 / 侧向留白</td><td>${f(m.leadRoom.minAhead)} / ${f(m.leadRoom.minSide)}</td></tr>
    <tr><td>死区内占比</td><td>${(m.deadZoneInside * 100).toFixed(0)}%</td></tr>
    <tr><td>样本数</td><td>${m.samples}</td></tr>
  </table>`;
}
function refreshSummary() {
  const m = summarize(rows, camera);
  if (!m) return;
  lastSummary = m;
  $('summaryBody').innerHTML = summaryHtml(m);
}

// ---------- 渲染 ----------
function render() {
  const fwd = camera.forward();
  const viewM = lookAt(camera.pos,
    [camera.pos[0] + fwd[0], camera.pos[1] + fwd[1], camera.pos[2] + fwd[2]], [0, 1, 0]);
  const projM = perspective(camera.fovYRad, camera.aspect, 0.1, 400);
  renderer.begin(mul4(projM, viewM));

  const g = [];
  for (let i = -100; i <= 100; i += 5) g.push(i, 0, -100, i, 0, 100, -100, 0, i, 100, 0, i);
  renderer.lines(g, [0.2, 0.23, 0.26]);

  if (refPath) {
    const p = [];
    for (let i = 1; i < refPath.length; i++) {
      p.push(refPath[i - 1][0], 0.02, refPath[i - 1][2], refPath[i][0], 0.02, refPath[i][2]);
    }
    renderer.lines(p, [0.32, 0.32, 0.34]);
  }
  for (const o of obstacles) {
    renderer.cube(o.center[0], o.center[1], o.center[2], o.yaw, o.size[0], o.size[1], o.size[2], [0.4, 0.45, 0.53]);
  }
  const vp = [], cp = [];
  for (let i = 1; i < rows.length; i++) {
    const a = rows[i - 1], b = rows[i];
    vp.push(a.veh.pos[0], 0.05, a.veh.pos[2], b.veh.pos[0], 0.05, b.veh.pos[2]);
    cp.push(...a.cam.pos, ...b.cam.pos);
  }
  renderer.lines(vp, [0.88, 0.88, 0.88]);
  renderer.lines(cp, [0.28, 0.72, 0.78]);

  renderer.cube(vehicle.pos[0], 0.8, vehicle.pos[2], vehicle.yaw, 2.2, 1.4, 4.2, [0.88, 0.53, 0.15]);
  const f2 = vehicle.forward();
  renderer.lines(
    [vehicle.pos[0], 1.6, vehicle.pos[2], vehicle.pos[0] + f2[0] * 3, 1.6, vehicle.pos[2] + f2[2] * 3],
    [1, 0.8, 0.3],
  );
  if (lastRow) {
    const L = lastRow.lead;
    renderer.cube(L[0], 0.4, L[2], 0, 0.5, 0.8, 0.5, [0.95, 0.28, 0.28], false);
    renderer.lines([vehicle.pos[0], 0.8, vehicle.pos[2], L[0], L[1] + 0.4, L[2]], [0.9, 0.82, 0.3]);
  }
}

function drawHud() {
  if (hud.width !== view.width) { hud.width = view.width; hud.height = view.height; }
  const w = hud.width, h = hud.height;
  hctx.clearRect(0, 0, w, h);
  if (!lastRow) return;
  const aspect = w / h;
  const toPx = (ax, ay) => [(ax / aspect * 0.5 + 0.5) * w, (0.5 - ay * 0.5) * h];
  const A = camera.anchor, dz = camera.deadZone;
  const inDz = lastRow.diag.inDeadZone;
  const [x0, y0] = toPx(A.x - dz.x, A.y + dz.y);
  const [x1, y1] = toPx(A.x + dz.x, A.y - dz.y);
  hctx.lineWidth = 2;
  hctx.strokeStyle = inDz ? 'rgba(125,220,135,0.85)' : 'rgba(240,110,100,0.85)';
  hctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
  const [axp, ayp] = toPx(A.x, A.y);
  hctx.strokeStyle = 'rgba(140,200,255,0.8)';
  hctx.beginPath();
  hctx.moveTo(axp - 9, ayp); hctx.lineTo(axp + 9, ayp);
  hctx.moveTo(axp, ayp - 9); hctx.lineTo(axp, ayp + 9);
  hctx.stroke();
  const [lx, ly] = toPx(lastRow.diag.ax, lastRow.diag.ay);
  hctx.fillStyle = inDz ? '#7fd18b' : '#ef7a6f';
  hctx.beginPath();
  hctx.arc(lx, ly, 6, 0, 7);
  hctx.fill();
}

function drawMap() {
  const W = map.width, H = map.height;
  mctx.fillStyle = '#16191e';
  mctx.fillRect(0, 0, W, H);
  if (!rows.length) return;
  let minX = 1e9, maxX = -1e9, minZ = 1e9, maxZ = -1e9;
  const eat = (x, z) => {
    minX = Math.min(minX, x); maxX = Math.max(maxX, x);
    minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
  };
  for (const r of rows) { eat(r.veh.pos[0], r.veh.pos[2]); eat(r.cam.pos[0], r.cam.pos[2]); }
  for (const o of obstacles) eat(o.center[0], o.center[2]);
  const pad = 12;
  minX -= pad; maxX += pad; minZ -= pad; maxZ += pad;
  const k = Math.min(W / (maxX - minX), H / (maxZ - minZ));
  const ox = (W - (maxX - minX) * k) / 2, oz = (H - (maxZ - minZ) * k) / 2;
  const px = (x) => (x - minX) * k + ox;
  const pz = (z) => H - ((z - minZ) * k + oz);
  mctx.fillStyle = '#3a4150';
  for (const o of obstacles) mctx.fillRect(px(o.center[0]) - 2, pz(o.center[2]) - 2, 4, 4);
  const poly = (get, color) => {
    mctx.strokeStyle = color;
    mctx.beginPath();
    for (let i = 0; i < rows.length; i++) {
      const [x, z] = get(rows[i]);
      if (i === 0) mctx.moveTo(px(x), pz(z)); else mctx.lineTo(px(x), pz(z));
    }
    mctx.stroke();
  };
  poly((r) => [r.veh.pos[0], r.veh.pos[2]], '#e8e8e8');
  poly((r) => [r.cam.pos[0], r.cam.pos[2]], '#4bc0c8');
  mctx.fillStyle = '#ef5050';
  for (let i = 0; i < rows.length; i += 30) {
    mctx.fillRect(px(rows[i].lead[0]) - 2, pz(rows[i].lead[2]) - 2, 4, 4);
  }
  const v = rows[rows.length - 1];
  mctx.fillStyle = '#e08f3c';
  mctx.beginPath();
  mctx.arc(px(v.veh.pos[0]), pz(v.veh.pos[2]), 4, 0, 7);
  mctx.fill();
}

function updateLive() {
  if (!lastRow) return;
  const d = lastRow.diag;
  const f = (x, n = 3) => x.toFixed(n);
  $('lvMode').textContent = mode === 'manual' ? '自由驾驶' : SCENARIOS[mode].name + (run && !run.done ? '（运行中）' : '（已完成）');
  $('lvSpeed').textContent = `${d.speed.toFixed(1)} m/s (${(d.speed * 3.6).toFixed(0)} km/h)`;
  $('lvEx').textContent = f(d.ax - camera.anchor.x);
  $('lvEy').textContent = f(d.ay - camera.anchor.y);
  $('lvE').textContent = f(Math.hypot(d.ax - camera.anchor.x, d.ay - camera.anchor.y));
  $('lvDz').textContent = d.inDeadZone ? '区内' : '区外(纠偏中)';
  $('lvDz').className = d.inDeadZone ? 'good' : 'bad';
  $('lvYawR').textContent = `${(Math.abs(d.dYaw) / d.dt * DEG).toFixed(1)} °/s`;
  $('lvPitchR').textContent = `${(Math.abs(d.dPitch) / d.dt * DEG).toFixed(1)} °/s`;
  $('lvAhead').textContent = f(d.roomAhead);
  $('lvSide').textContent = f(d.roomSide);
}

// ---------- 主循环 ----------
let last = performance.now();
let summaryTimer = 0;
function frame(now) {
  const aspect = renderer.resize();
  camera.aspect = aspect;
  // 滑杆实时生效（前视距离/死区）
  camera.lookAheadDist = parseFloat($('look').value);
  camera.deadZone.x = parseFloat($('dzx').value);
  camera.deadZone.y = parseFloat($('dzy').value);

  const dt = Math.min(0.05, Math.max(0.0005, (now - last) / 1000));
  last = now;

  if (run && !run.done) {
    lastRow = run.step(dt);
    rows.push(lastRow);
    simTime = run.t;
    if (run.done && !summarized) {
      summarized = true;
      refreshSummary();
    }
  } else {
    vehicle.update(dt, manualControls());
    const out = camera.update(dt, vehicleState(vehicle));
    simTime += dt;
    lastRow = {
      t: simTime, dt,
      veh: { pos: [...vehicle.pos], vel: [...vehicle.vel], yaw: vehicle.yaw, yawRate: vehicle.yawRate },
      cam: { pos: [...out.position], yaw: out.yaw, pitch: out.pitch },
      lead: [...out.leadPoint],
      diag: { ...out.diag },
    };
    rows.push(lastRow);
    if (rows.length > 6000) rows.shift();
    summaryTimer += dt;
    if (mode === 'manual' && summaryTimer > 0.5) {
      summaryTimer = 0;
      refreshSummary();
    }
  }

  render();
  drawHud();
  drawMap();
  updateLive();
  requestAnimationFrame(frame);
}

reset('manual');
requestAnimationFrame(frame);
