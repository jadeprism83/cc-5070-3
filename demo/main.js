// 载具镜头演示主程序：
//  - 自由驾驶（W/S/A/D 直接驾驶载具）与赛道自动驾驶；
//  - 驾驶镜头脚本（与入库样片同一控制器，实时跑完 24s 剧本）；
//  - 载入入库样片参数 / 在本场景回放样例；
//  - 所有画面与偏差都来自当场 FollowCamera 计算，不使用固定帧冒充。
import { FollowCamera } from '../src/followCamera.js';
import { Vehicle } from '../src/vehicle.js';
import { SCENARIOS, createRun, runScenario, vehicleState } from '../src/trajectories.js';
import { buildTrack, trackBarriers, FICTION_TRACK } from '../src/track.js';
import { createShotRun, shotControls, SHOT_PHASES, SHOT_DURATION } from '../src/drivingShot.js';
import { summarize } from '../src/metrics.js';
import { obstaclesAlongPath, scatterObstacles } from '../src/scene.js';
import { lookAt, perspective, mul4, wrapPi } from '../src/math3d.js';
import { createRenderer } from './renderer.js';

const $ = (id) => document.getElementById(id);
const view = $('view'), hud = $('hud'), map = $('map');
const renderer = createRenderer(view);
const hctx = hud.getContext('2d');
const mctx = map.getContext('2d');
const DEG = 180 / Math.PI;
const SAMPLE_URL = '../samples/driving-line/shot.json';

const track = buildTrack(FICTION_TRACK);
const trackObs = trackBarriers(track);
let mode = 'manual';
let vehicle, camera, run = null, runKind = null;
let rows = [], obstacles = [], refPath = null;
let simTime = 0, summarized = false, lastSummary = null, lastRow = null;
let manualAutoSteer = false;
let sample = null, sampleRaw = null, sampleReplay = null, sampleErrors = [];
let replayMismatch = null; // {pos, yawDeg, pitchDeg} 或 null

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
  lastRow = null;
  run = null;
  runKind = null;
  sampleReplay = null;
  replayMismatch = null;

  if (mode === 'manual' || mode === 'drive') {
    vehicle.pos = [0, 0, 0];
    vehicle.yaw = 0;
    // 自由驾驶/赛道驾驶都在虚构赛道起点，方便直接观察路缘与弯道
    if (mode === 'drive') vehicle.pos = [0, 0, 4];
    obstacles = mode === 'drive' ? trackObs : scatterObstacles();
    refPath = track.points.map((p) => [p.x, 0, p.z]);
    $('summaryBody').textContent = mode === 'drive'
      ? '赛道驾驶中：勾选“赛道自动驾驶”可自动沿赛道高速过弯；也可随时手动接管。'
      : '自由驾驶中：指标随驾驶实时累计。';
  } else if (mode === 'shot') {
    obstacles = trackObs;
    refPath = track.points.map((p) => [p.x, 0, p.z]);
    run = createShotRun(vehicle, camera, track);
    runKind = 'shot';
    $('summaryBody').textContent = '驾驶镜头脚本运行中……（加速→高速左弯→急停→倒车）';
  } else {
    const sc = SCENARIOS[mode];
    // 参考路径仅用于摆放路侧障碍与俯视地图；镜头指标完全来自随后的实时仿真
    const refRows = runScenario(new Vehicle(), new FollowCamera(camOpts()), sc, 1 / 60);
    refPath = refRows.map((r) => r.veh.pos);
    obstacles = obstaclesAlongPath(refPath, 7, 6);
    run = createRun(vehicle, camera, sc);
    runKind = 'scenario';
    $('summaryBody').textContent = `${sc.name}场景运行中……`;
  }
  for (const b of document.querySelectorAll('#btns button')) {
    b.classList.toggle('on', b.dataset.mode === mode);
  }
  $('modeHint').textContent = mode === 'drive'
    ? '赛道驾驶：W/S 油门·刹车，A/D 转向；倒车为 S 直至停稳后再按 S。'
    : mode === 'shot'
      ? '驾驶镜头脚本：自动沿虚构赛道执行加速/高速弯/急停/倒车，可拖动前视距离观察构图变化。'
      : '自由驾驶：W/S 油门·刹车，A/D 转向。';
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
// 自动驾驶意图：在起步与直道时使用油门阶段，弯中后段使用高速过弯阶段。
function autoDriveControls() {
  const s = track.project(vehicle.pos[0], vehicle.pos[2]).s;
  const sc = shotControls(vehicle, track, s < 175 ? 6.0 : 12.0);
  return { throttle: sc.throttle, steer: sc.steer };
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
$('autoDrive').onchange = () => { manualAutoSteer = $('autoDrive').checked; };
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

// ---------- 入库样例资源 ----------
async function loadSampleAsset() {
  try {
    const res = await fetch(SAMPLE_URL, { cache: 'no-store' });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    sample = await res.json();
  } catch (err) {
    $('assetBody').textContent = `样例载入失败：${err.message}（需通过 npm run demo 从仓库根目录访问）`;
    return;
  }
  const v = sample.verify;
  $('assetBody').innerHTML = `样例 <b>${sample.scenario}</b> · ${sample.track.name}<br>` +
    `帧数 ${sample.raw.frames} · 时长 ${sample.duration}s · 前视距离 ${sample.cameraParams.lookAheadDist}m<br>` +
    `原始记录核验：前瞻帧 ${v.leadFrames} / 主体帧 ${v.subjectFrames}，` +
    `倒行最大偏航 ${v.reverseMaxYawRateDeg.toFixed(1)}°/s，主体投影最大|e| ${v.subjectMaxErr.toFixed(3)}`;
  const thumbs = $('assetThumbs');
  thumbs.innerHTML = '';
  for (const f of sample.frames) {
    const fig = document.createElement('figure');
    const img = document.createElement('img');
    img.src = `../samples/driving-line/${f.file}`;
    img.alt = f.file;
    const cap = document.createElement('figcaption');
    cap.textContent = `t=${f.t.toFixed(1)}s ${f.targetKind === 'subject' ? '倒行·主体构图' : '前瞻构图'}`;
    fig.append(img, cap);
    thumbs.append(fig);
  }
  $('loadSampleBtn').disabled = false;
  $('replaySampleBtn').disabled = false;
  checkSampleRaw();
}

// 用入库 raw.json 的原始逐帧记录复核样例，而不是相信汇总字段。
async function checkSampleRaw() {
  sampleErrors = [];
  try {
    const res = await fetch('../samples/driving-line/raw.json', { cache: 'no-store' });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    sampleRaw = await res.json();
  } catch (err) {
    sampleErrors.push(`raw.json 不可读：${err.message}`);
    return;
  }
  const fr = sampleRaw.frames;
  const lead = fr.filter((x) => x.targetKind === 'lead');
  const subj = fr.filter((x) => x.targetKind === 'subject');
  if (!lead.length) sampleErrors.push('原始记录中没有前瞻帧');
  if (!subj.length) sampleErrors.push('原始记录中没有倒行主体帧');
  const chord = (x) => Math.hypot(x.lead[0] - x.veh.pos[0], x.lead[2] - x.veh.pos[2]);
  const low = lead.reduce((a, b) => (Math.abs(b.speed - 8) < Math.abs(a.speed - 8) ? b : a), lead[0]);
  const high = lead.reduce((a, b) => (Math.abs(b.speed - 26) < Math.abs(a.speed - 26) ? b : a), lead[0]);
  if (!(chord(high) > chord(low) + 3)) {
    sampleErrors.push(`前瞻距离未随速度伸展（${chord(low).toFixed(1)}m → ${chord(high).toFixed(1)}m）`);
  }
  if (subj.some((x) => x.subjectProj.behind)) sampleErrors.push('倒行时主体落在相机背后');
  const steady = subj.filter((x) => x.vLong <= -3);
  if (!steady.length || !steady.every((x) => x.leadBehindCamera)) {
    sampleErrors.push('缺少倒行前瞻落在相机背后的原始证据');
  }
  const subjErr = Math.max(...subj.map((x) => Math.hypot(x.subjectProj.ax, x.subjectProj.ay)));
  if (subjErr > 0.6) sampleErrors.push(`主体投影偏差过大（${subjErr.toFixed(2)}）`);
  if (Math.max(...fr.map((x) => x.vLong)) < 24) sampleErrors.push('样例未达到高速');
  if (Math.min(...fr.map((x) => x.vLong)) > -3) sampleErrors.push('样例未真实倒车');
  if (!sampleErrors.length) sampleErrors.push('原始记录与样片一致');
}

$('loadSampleBtn').onclick = () => {
  if (!sample) return;
  $('look').value = sample.cameraParams.lookAheadDist;
  $('lookVal').textContent = sample.cameraParams.lookAheadDist.toFixed(1);
  reset('shot');
  $('summaryBody').textContent = `已载入样例参数（前视距离 ${sample.cameraParams.lookAheadDist}m），驾驶镜头脚本运行中……`;
};

// 在本场景按样例原始记录回放，实时显示相机输出与原始记录构图数据的偏差。
$('replaySampleBtn').onclick = () => {
  if (!sampleRaw) return;
  reset('shot');
  run = null;
  runKind = 'replay';
  vehicle = new Vehicle();
  vehicle.pos = [...sampleRaw.frames[0].veh.pos];
  vehicle.yaw = sampleRaw.frames[0].veh.yaw;
  camera = new FollowCamera({
    ...camOpts(),
    lookAheadDist: sample?.cameraParams?.lookAheadDist ?? camOpts().lookAheadDist,
  });
  replayIdx = 0;
  $('summaryBody').textContent = '样例回放中：载具状态取自入库原始记录，相机由本场景实时计算。';
};
let replayIdx = 0;

// ---------- 汇总 ----------
function summaryHtml(m) {
  const d = (x) => (x * DEG).toFixed(1);
  const f = (x) => x.toFixed(3);
  return `<table>
    <tr><td class="sectionTitle" colspan="2">构图误差（投影空间）</td></tr>
    <tr><td>平均 |e| / 最大 |e|</td><td>${f(m.composition.meanErr)} / ${f(m.composition.maxErr)}</td></tr>
    <tr><td>平均 ex / ey</td><td>${f(m.composition.meanErrX)} / ${f(m.composition.meanErrY)}</td></tr>
    <tr><td class="sectionTitle" colspan="2">主体投影（构图核验）</td></tr>
    <tr><td>平均 / 最大 |e|</td><td>${f(m.subjectComposition.meanErr)} / ${f(m.subjectComposition.maxErr)}</td></tr>
    <tr><td class="sectionTitle" colspan="2">镜头运动成本</td></tr>
    <tr><td>偏航 / 俯仰总行程</td><td>${d(m.motion.yawTravel)}° / ${d(m.motion.pitchTravel)}°</td></tr>
    <tr><td>最大角速度 偏航 / 俯仰</td><td>${d(m.motion.maxYawRate)}°/s / ${d(m.motion.maxPitchRate)}°/s</td></tr>
    <tr><td class="sectionTitle" colspan="2">留白 / 死区 / 目标</td></tr>
    <tr><td>最小上方 / 侧向留白</td><td>${f(m.leadRoom.minAhead)} / ${f(m.leadRoom.minSide)}</td></tr>
    <tr><td>死区内占比</td><td>${(m.deadZoneInside * 100).toFixed(0)}%</td></tr>
    <tr><td>前瞻帧 / 主体帧</td><td>${m.target.leadFrames} / ${m.target.subjectFrames}</td></tr>
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
  for (let i = -200; i <= 200; i += 5) g.push(i, 0, -100, i, 0, 300, -100, 0, i, 200, 0, i);
  renderer.lines(g, [0.2, 0.23, 0.26]);

  if (refPath) {
    const p = [];
    for (let i = 1; i < refPath.length; i++) {
      p.push(refPath[i - 1][0], 0.02, refPath[i - 1][2], refPath[i][0], 0.02, refPath[i][2]);
    }
    renderer.lines(p, [0.32, 0.32, 0.34]);
  }
  // 赛道边界（左右各一条）
  if (mode === 'drive' || mode === 'shot') {
    const edges = track.edges(track.width / 2);
    for (const side of [edges.left, edges.right]) {
      const p = [];
      for (let i = 1; i < side.length; i++) {
        p.push(side[i - 1][0], 0.04, side[i - 1][1], side[i][0], 0.04, side[i][1]);
      }
      renderer.lines(p, [0.36, 0.42, 0.5]);
    }
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
    const L = lastRow.target ?? lastRow.lead;
    const isSubj = lastRow.diag.targetKind === 'subject';
    renderer.cube(L[0], 0.4, L[2], 0, 0.5, 0.8, 0.5, isSubj ? [0.78, 0.47, 1.0] : [0.95, 0.28, 0.28], false);
    renderer.lines([vehicle.pos[0], 0.8, vehicle.pos[2], L[0], L[1] + 0.4, L[2]], [0.9, 0.82, 0.3]);
    if (isSubj) {
      const S = lastRow.lead; // 倒行时同时画出被弃用的真实前瞻点（紫色小十字）
      renderer.cube(S[0], 0.3, S[2], 0, 0.3, 0.3, 0.3, [0.6, 0.35, 0.9], false);
    }
    if (lastRow.subject) {
      const S = lastRow.subject;
      renderer.cube(S[0], S[1], S[2], 0, 0.35, 0.35, 0.35, [0.35, 0.86, 0.5], false);
    }
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
  // 主体投影（绿圈）：倒行时这是构图目标
  if (lastRow.diag.subject && !lastRow.diag.subject.behind) {
    const [sx, sy] = toPx(lastRow.diag.subject.ax, lastRow.diag.subject.ay);
    hctx.strokeStyle = 'rgba(90,220,135,0.9)';
    hctx.beginPath();
    hctx.arc(sx, sy, 10, 0, 7);
    hctx.stroke();
  }
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
  if (mode === 'drive' || mode === 'shot') {
    mctx.strokeStyle = '#333a45';
    mctx.beginPath();
    for (let i = 0; i < track.points.length; i++) {
      const p = track.points[i];
      if (i === 0) mctx.moveTo(px(p.x), pz(p.z)); else mctx.lineTo(px(p.x), pz(p.z));
    }
    mctx.stroke();
  }
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
  for (let i = 0; i < rows.length; i += 30) {
    mctx.fillStyle = rows[i].diag.targetKind === 'subject' ? '#c878ff' : '#ef5050';
    const L = rows[i].target ?? rows[i].lead;
    mctx.fillRect(px(L[0]) - 2, pz(L[2]) - 2, 4, 4);
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
  const f = (x, n = 3) => (Number.isFinite(x) ? x.toFixed(n) : '–');
  const modeName = mode === 'manual' ? '自由驾驶'
    : mode === 'drive' ? (manualAutoSteer ? '赛道自动驾驶' : '赛道手动驾驶')
      : mode === 'shot' ? (runKind === 'replay' || runKind === 'replay-done'
        ? '样例回放' + (runKind === 'replay-done' ? '（已完成）' : '（运行中）')
        : '驾驶镜头脚本' + (run && !run.done ? '（运行中）' : '（已完成）'))
        : SCENARIOS[mode].name + (run && !run.done ? '（运行中）' : '（已完成）');
  $('lvMode').textContent = modeName;
  $('lvSpeed').textContent = `${d.speed.toFixed(1)} m/s (${(d.speed * 3.6).toFixed(0)} km/h)`;
  $('lvEx').textContent = f(d.ax - camera.anchor.x);
  $('lvEy').textContent = f(d.ay - camera.anchor.y);
  $('lvE').textContent = f(Math.hypot(d.ax - camera.anchor.x, d.ay - camera.anchor.y));
  $('lvDz').textContent = d.inDeadZone ? '区内' : '区外(纠偏中)';
  $('lvDz').className = d.inDeadZone ? 'good' : 'bad';
  const dtSafe = Math.max(d.dt, 1e-9);
  $('lvYawR').textContent = `${(Math.abs(d.dYaw) / dtSafe * DEG).toFixed(1)} °/s`;
  $('lvPitchR').textContent = `${(Math.abs(d.dPitch) / dtSafe * DEG).toFixed(1)} °/s`;
  $('lvAhead').textContent = f(d.roomAhead);
  $('lvSide').textContent = f(d.roomSide);
  $('lvTarget').textContent = d.targetKind === 'subject' ? '主体位置（倒行）' : '前瞻点';
  $('lvLook').textContent = `${f(d.lookAhead, 1)} m`;
  const sp = d.subject;
  $('lvSubject').textContent = sp ? (sp.behind ? '主体在相机背后' : `${f(Math.hypot(sp.ax - camera.anchor.x, sp.ay - camera.anchor.y))}`) : '–';
  $('lvPhase').textContent = mode === 'shot' && runKind === 'shot'
    ? (SHOT_PHASES.filter((p) => run.t >= p.t).at(-1)?.label ?? '–')
    : runKind === 'replay' || runKind === 'replay-done'
      ? `回放 ${(replayIdx / 60).toFixed(2)}s`
      : (lastRow.ctrl ? lastRow.ctrl.kind : '–');
}

// ---------- 运行状态与异常 ----------
function updateStatus() {
  const running = !!run && !run.done;
  $('stRun').textContent = lastRow
    ? `运行中 · t=${simTime.toFixed(2)}s · ${rows.length} 帧`
    : '尚未开始';
  $('stRun').className = lastRow ? 'good' : '';

  const alerts = [];
  if (lastRow) {
    const d = lastRow.diag;
    const inDz = d.inDeadZone;
    $('stFollow').textContent = inDz
      ? '正常 · 目标在死区内'
      : '纠偏中 · 目标在死区外';
    $('stFollow').className = inDz ? 'good' : 'warn';
    const e = Math.hypot(d.ax - camera.anchor.x, d.ay - camera.anchor.y);
    const overX = Math.abs(d.ax - camera.anchor.x) - camera.deadZone.x;
    const overY = Math.abs(d.ay - camera.anchor.y) - camera.deadZone.y;
    const compBad = overX > 0.05 || overY > 0.05;
    $('stComp').textContent = `${compBad ? '偏差超死区' : '偏差受控'} · |e|=${e.toFixed(3)}` +
      (compBad ? `（超出死区 x=${overX.toFixed(2)} y=${overY.toFixed(2)}）` : '');
    $('stComp').className = compBad ? 'bad' : 'good';
    if (d.subject?.behind) alerts.push({ kind: 'bad', text: '主体位置落在相机背后：倒行构图目标不可见' });
    const dtSafe = Math.max(d.dt, 1e-9);
    if (Math.abs(d.dYaw) / dtSafe > camera.maxYawRate * 1.05) {
      alerts.push({ kind: 'bad', text: `偏航角速度超出上限：${(Math.abs(d.dYaw) / dtSafe * DEG).toFixed(1)}°/s` });
    }
    if (compBad) {
      alerts.push({ kind: 'warn', text: `构图偏差超出死区：ex=${(d.ax - camera.anchor.x).toFixed(2)} ey=${(d.ay - camera.anchor.y).toFixed(2)}` });
    }
    if (!running && mode === 'shot' && runKind === 'shot') {
      alerts.push({ kind: 'ok', text: '驾驶镜头脚本已跑完，构图数据来自本次实时仿真' });
    }
  }
  if (sample) {
    $('stSample').textContent = sampleErrors.length && sampleErrors[0] !== '原始记录与样片一致'
      ? `异常 ${sampleErrors.length} 项`
      : '原始记录一致';
    $('stSample').className = sampleErrors.length && sampleErrors[0] !== '原始记录与样片一致' ? 'bad' : 'good';
    for (const err of sampleErrors) {
      if (err === '原始记录与样片一致') continue;
      alerts.push({ kind: 'bad', text: `样例核验：${err}` });
    }
  } else {
    $('stSample').textContent = '未载入';
  }

  if (replayMismatch) {
    const bad = replayMismatch.pos > 1e-6 || replayMismatch.yawDeg > 1e-4 || replayMismatch.pitchDeg > 1e-4;
    $('stReplay').textContent = `位置 ${replayMismatch.pos.toFixed(9)}m · 偏航 ${replayMismatch.yawDeg.toFixed(9)}° · 俯仰 ${replayMismatch.pitchDeg.toFixed(9)}°`;
    $('stReplay').className = bad ? 'bad' : 'good';
    if (bad) alerts.push({ kind: 'bad', text: '样例重建偏差超限：本场景相机输出与入库位姿不一致' });
  } else {
    $('stReplay').textContent = '未回放';
    $('stReplay').className = '';
  }

  const list = $('alerts');
  list.innerHTML = '';
  if (!alerts.length) {
    const li = document.createElement('li');
    li.className = 'ok';
    li.textContent = lastRow ? '未检测到异常' : '等待运行';
    list.append(li);
  } else {
    for (const a of alerts) {
      const li = document.createElement('li');
      li.className = a.kind;
      li.textContent = a.text;
      list.append(li);
    }
  }
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
  } else if (runKind === 'replay' && sampleRaw) {
    const fr = sampleRaw.frames[Math.min(replayIdx, sampleRaw.frames.length - 1)];
    const st = {
      position: fr.veh.pos, velocity: fr.veh.vel,
      yawRate: fr.veh.yawRate, forward: [Math.sin(fr.veh.yaw), 0, Math.cos(fr.veh.yaw)],
    };
    vehicle.pos = [...fr.veh.pos];
    vehicle.yaw = fr.veh.yaw;
    // 用原始记录的帧步长重放，保证与入库结果可逐帧对照
    const out = camera.update(fr.dt, st);
    const posErr = Math.hypot(
      out.position[0] - fr.cam.pos[0],
      out.position[1] - fr.cam.pos[1],
      out.position[2] - fr.cam.pos[2],
    );
    const yawErr = Math.abs(wrapPi(out.yaw - fr.cam.yaw));
    const pitchErr = Math.abs(out.pitch - fr.cam.pitch);
    replayMismatch = {
      pos: Math.max(replayMismatch?.pos ?? 0, posErr),
      yawDeg: Math.max(replayMismatch?.yawDeg ?? 0, yawErr * DEG),
      pitchDeg: Math.max(replayMismatch?.pitchDeg ?? 0, pitchErr * DEG),
    };
    lastRow = {
      t: fr.t, dt: fr.dt,
      veh: { pos: [...fr.veh.pos], vel: [...fr.veh.vel], yaw: fr.veh.yaw, yawRate: fr.veh.yawRate },
      cam: { pos: [...out.position], yaw: out.yaw, pitch: out.pitch },
      lead: [...out.leadPoint], subject: [...out.subjectPoint], target: [...out.targetPoint],
      diag: { ...out.diag },
    };
    rows.push(lastRow);
    simTime = fr.t;
    replayIdx++;
    if (replayIdx >= sampleRaw.frames.length) {
      runKind = 'replay-done';
      refreshSummary();
    }
  } else {
    let ctrl = manualControls();
    if (mode === 'drive' && manualAutoSteer) {
      ctrl = autoDriveControls();
    }
    vehicle.update(dt, ctrl);
    const out = camera.update(dt, vehicleState(vehicle));
    simTime += dt;
    lastRow = {
      t: simTime, dt,
      ctrl: { ...ctrl },
      veh: { pos: [...vehicle.pos], vel: [...vehicle.vel], yaw: vehicle.yaw, yawRate: vehicle.yawRate },
      cam: { pos: [...out.position], yaw: out.yaw, pitch: out.pitch },
      lead: [...out.leadPoint], subject: [...out.subjectPoint], target: [...out.targetPoint],
      diag: { ...out.diag },
    };
    rows.push(lastRow);
    if (rows.length > 6000) rows.shift();
    summaryTimer += dt;
    if ((mode === 'manual' || mode === 'drive') && summaryTimer > 0.5) {
      summaryTimer = 0;
      refreshSummary();
    }
  }

  render();
  drawHud();
  drawMap();
  updateLive();
  updateStatus();
  requestAnimationFrame(frame);
}

reset('manual');
loadSampleAsset();
requestAnimationFrame(frame);
