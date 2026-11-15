// 驾驶镜头样例构建：在净目录重跑驾驶镜头（虚构赛道 + 前瞻路径驾驶），
// 产出的样片、逐帧原始记录、镜头配置与重建脚本一并入库。
//
// 用法:
//   node tools/buildDrivingShot.js                 # 默认写入 samples/driving-line/
//   node tools/buildDrivingShot.js --out=<目录>     # 指定空/净目录
//
// 约定（与 README / tests 一致）：
//  - 样片 PNG 必须由本次运行的真实相机位姿渲染，拒绝复用历史文件；
//  - 前瞻距离与主体位置取自逐帧原始记录 raw.json，不从汇总反推；
//  - 重建脚本 replayDrivingShot.js 与样片一起写出，任何人可脱开本工具重放。
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { FollowCamera } from '../src/followCamera.js';
import { Vehicle } from '../src/vehicle.js';
import { buildTrack, trackBarriers, FICTION_TRACK } from '../src/track.js';
import { SHOT_DURATION, SHOT_DT, SHOT_PHASES, runShot } from '../src/drivingShot.js';
import { summarize, formatMetrics } from '../src/metrics.js';
import { lookAt, perspective } from '../src/math3d.js';
import {
  createImage, writePNG, makeProjector,
  drawLine3, drawBox3, drawGrid, drawCross3, drawLine,
} from '../src/softrender.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const outArg = args.find((a) => a.startsWith('--out='));
const outDir = outArg ? path.resolve(outArg.split('=')[1]) : path.join(root, 'samples', 'driving-line');
const CAM_PARAMS = {
  aspect: 16 / 9,
  lookAheadDist: 12,
  deadZone: { x: 0.22, y: 0.16 },
  anchor: { x: 0, y: 0.1 },
  fovYDeg: 60,
  speedLeadTime: 0.35,
};
const VIEW_FRACTIONS = [0.2, 0.45, 0.72, 0.93];

function assertPristine(dir) {
  if (!fs.existsSync(dir)) {
    fs.mkdirSync(dir, { recursive: true });
    return;
  }
  const entries = fs.readdirSync(dir).filter((n) => n !== '.gitkeep');
  if (entries.length) {
    throw new Error(
      `输出目录非净目录：${dir}\n  现有内容: ${entries.slice(0, 8).join(', ')}${entries.length > 8 ? ' …' : ''}\n` +
      '  样例必须在净目录生成；请换一个空目录或先清理后重跑。',
    );
  }
}

function sha256(buf) {
  return crypto.createHash('sha256').update(buf).digest('hex');
}

// ---------- 1. 仿真：全新载具 + 镜头，固定步长 ----------
function simulate() {
  const track = buildTrack(FICTION_TRACK);
  const vehicle = new Vehicle();
  const camera = new FollowCamera(CAM_PARAMS);
  const rows = runShot(vehicle, camera, track, SHOT_DT);
  return { track, vehicle, camera, rows };
}

// 前瞻距离与主体位置核验：直接从原始逐帧记录计算，不读 metrics。
export function verifyFromRaw(rows, camera) {
  const A = camera.anchor;
  const probe = (r) => {
    const dx = r.lead[0] - r.veh.pos[0];
    const dz = r.lead[2] - r.veh.pos[2];
    return Math.hypot(dx, dz);
  };
  const fs_ = rows.filter((r) => r.diag.targetKind === 'lead');
  const rs_ = rows.filter((r) => r.diag.targetKind === 'subject');
  // 前瞻距离应随速度伸展：比较低速与高速前瞻帧的弦长
  const bySpeed = (arr, s) => {
    let best = null, bd = 1e9;
    for (const r of arr) {
      const d = Math.abs(r.diag.speed - s);
      if (d < bd) { bd = d; best = r; }
    }
    return best;
  };
  const low = bySpeed(fs_, 8), high = bySpeed(fs_, 26);
  const subjectErr = (r) => {
    const sx = r.diag.subject.ax - A.x;
    const sy = r.diag.subject.ay - A.y;
    return Math.hypot(sx, sy);
  };
  return {
    samples: rows.length,
    leadFrames: fs_.length,
    subjectFrames: rs_.length,
    leadChordLow: low ? probe(low) : null,
    leadChordHigh: high ? probe(high) : null,
    leadChordGrows: low && high ? probe(high) > probe(low) + 3 : false,
    reverseMaxYawRateDeg: rs_.length
      ? Math.max(...rs_.map((r) => Math.abs(r.diag.dYaw) / Math.max(r.diag.dt, 1e-9))) * 180 / Math.PI
      : 0,
    // 触发条件证据：稳定倒车时原始前瞻点落在相机背后（逐帧点乘记录）
    steadyReverseFrames: rs_.filter((r) => r.diag.vLong <= -3).length,
    leadBehindCameraFrames: rows.filter((r) => r.diag.leadBehindCamera).length,
    steadyReverseLeadBehind: rs_.filter((r) => r.diag.vLong <= -3)
      .every((r) => r.diag.leadBehindCamera),
    subjectMaxErr: rs_.length ? Math.max(...rs_.map(subjectErr)) : null,
    subjectMaxErrX: rs_.length ? Math.max(...rs_.map((r) => Math.abs(r.diag.subject.ax - A.x))) : null,
    subjectMaxErrY: rs_.length ? Math.max(...rs_.map((r) => Math.abs(r.diag.subject.ay - A.y))) : null,
    allSubjectInFront: rs_.every((r) => !r.diag.subject.behind),
    maxLookAhead: Math.max(...rows.map((r) => r.diag.lookAhead)),
    vmax: Math.max(...rows.map((r) => r.diag.speed)),
    vminLong: Math.min(...rows.map((r) => r.diag.vLong)),
  };
}

// ---------- 2. 渲染样片（真实相机位姿） ----------
function renderFrames(track, rows, camera, dir) {
  const W = 960, H = 540;
  const obstacles = trackBarriers(track);
  const projM = perspective(camera.fovYRad, 16 / 9, 0.1, 400);
  const edges = track.edges(track.width / 2);
  const edgeL = edges.left, edgeR = edges.right;
  const frames = [];

  VIEW_FRACTIONS.forEach((frac, idx) => {
    const i = Math.min(rows.length - 1, Math.floor((rows.length - 1) * frac));
    const row = rows[i];
    const img = createImage(W, H);
    const fwd = [
      Math.cos(row.cam.pitch) * Math.sin(row.cam.yaw),
      Math.sin(row.cam.pitch),
      Math.cos(row.cam.pitch) * Math.cos(row.cam.yaw),
    ];
    const view = lookAt(row.cam.pos,
      [row.cam.pos[0] + fwd[0], row.cam.pos[1] + fwd[1], row.cam.pos[2] + fwd[2]], [0, 1, 0]);
    const proj = makeProjector(view, projM, W, H);

    drawGrid(img, proj);
    for (let k = 1; k < edgeL.length; k++) {
      drawLine3(img, proj, [edgeL[k - 1][0], 0.03, edgeL[k - 1][1]], [edgeL[k][0], 0.03, edgeL[k][1]], [70, 82, 96]);
      drawLine3(img, proj, [edgeR[k - 1][0], 0.03, edgeR[k - 1][1]], [edgeR[k][0], 0.03, edgeR[k][1]], [70, 82, 96]);
    }
    for (const o of obstacles) drawBox3(img, proj, o.center, o.size, o.yaw, [96, 110, 128]);
    for (let k = 1; k <= i; k++) {
      const a = rows[k - 1], b = rows[k];
      drawLine3(img, proj, [a.veh.pos[0], 0.06, a.veh.pos[2]], [b.veh.pos[0], 0.06, b.veh.pos[2]], [220, 220, 220]);
      drawLine3(img, proj, a.cam.pos, b.cam.pos, [70, 190, 200]);
    }
    drawBox3(img, proj, [row.veh.pos[0], 0.8, row.veh.pos[2]], [2.2, 1.4, 4.2], row.veh.yaw, [230, 140, 40]);
    // 构图目标：前瞻点（红/倒行时主体为紫）+ 主体位置十字（绿）；倒行另标真实前瞻点（暗紫小十字）
    const targetColor = row.diag.targetKind === 'subject' ? [200, 120, 255] : [240, 70, 70];
    drawCross3(img, proj, row.target, 0.8, targetColor);
    drawCross3(img, proj, row.subject, 0.5, [90, 220, 130]);
    if (row.diag.targetKind === 'subject') drawCross3(img, proj, row.lead, 0.45, [120, 70, 190]);
    drawLine3(img, proj, [row.veh.pos[0], 0.8, row.veh.pos[2]], row.target, [230, 210, 80]);

    const name = `shot_${String(idx + 1).padStart(2, '0')}_t${row.t.toFixed(2)}.png`;
    writePNG(path.join(dir, name), W, H, img.data);
    frames.push({ file: name, t: row.t, index: i, targetKind: row.diag.targetKind });
  });
  return frames;
}

function renderMap(track, rows, dir) {
  const S = 760;
  const img = createImage(S, S, [22, 25, 30]);
  const all = track.points;
  let minX = 1e9, maxX = -1e9, minZ = 1e9, maxZ = -1e9;
  for (const p of all) { minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x); minZ = Math.min(minZ, p.z); maxZ = Math.max(maxZ, p.z); }
  for (const r of rows) {
    minX = Math.min(minX, r.veh.pos[0], r.cam.pos[0]); maxX = Math.max(maxX, r.veh.pos[0], r.cam.pos[0]);
    minZ = Math.min(minZ, r.veh.pos[2], r.cam.pos[2]); maxZ = Math.max(maxZ, r.veh.pos[2], r.cam.pos[2]);
  }
  const pad = 18;
  minX -= pad; maxX += pad; minZ -= pad; maxZ += pad;
  const k = Math.min(S / (maxX - minX), S / (maxZ - minZ));
  const px = (x) => (x - minX) * k + (S - (maxX - minX) * k) / 2;
  const pz = (z) => S - ((z - minZ) * k + (S - (maxZ - minZ) * k) / 2);
  for (let i = 1; i < all.length; i++) {
    drawLine(img, px(all[i - 1].x), pz(all[i - 1].z), px(all[i].x), pz(all[i].z), [64, 70, 80]);
  }
  for (let i = 1; i < rows.length; i++) {
    const a = rows[i - 1], b = rows[i];
    drawLine(img, px(a.veh.pos[0]), pz(a.veh.pos[2]), px(b.veh.pos[0]), pz(b.veh.pos[2]), [230, 230, 230]);
    drawLine(img, px(a.cam.pos[0]), pz(a.cam.pos[2]), px(b.cam.pos[0]), pz(b.cam.pos[2]), [70, 190, 200]);
  }
  for (let i = 0; i < rows.length; i += 20) {
    const r = rows[i];
    const c = r.diag.targetKind === 'subject' ? [200, 120, 255] : [240, 80, 80];
    drawLine(img, px(r.target[0]) - 2, pz(r.target[2]), px(r.target[0]) + 2, pz(r.target[2]), c);
    drawLine(img, px(r.target[0]), pz(r.target[2]) - 2, px(r.target[0]), pz(r.target[2]) + 2, c);
  }
  const f = 'shot_map.png';
  writePNG(path.join(dir, f), S, S, img.data);
  return f;
}

// ---------- 3. 重建脚本 ----------
function replaySource(srcRel) {
  return `// 由 buildDrivingShot.js 自动生成：仅依赖入库的 shot.json + 仓库 src/，重放驾驶镜头。
// 用法: node samples/<样例目录>/replayDrivingShot.js
// 校验: 逐帧重算相机位姿并与原始记录比对，打印最大偏差与前瞻/主体核验值。
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { FollowCamera } from '${srcRel}';

const here = path.dirname(fileURLToPath(import.meta.url));
const shot = JSON.parse(fs.readFileSync(path.join(here, 'shot.json'), 'utf8'));
const raw = JSON.parse(fs.readFileSync(path.join(here, 'raw.json'), 'utf8'));

const cam = new FollowCamera(shot.cameraParams);
let maxPos = 0, maxYaw = 0, maxPitch = 0;
for (let i = 0; i < raw.frames.length; i++) {
  const fr = raw.frames[i];
  const out = cam.update(fr.dt, {
    position: fr.veh.pos, velocity: fr.veh.vel,
    yawRate: fr.veh.yawRate, forward: [Math.sin(fr.veh.yaw), 0, Math.cos(fr.veh.yaw)],
  });
  const d = Math.hypot(
    out.position[0] - fr.cam.pos[0],
    out.position[1] - fr.cam.pos[1],
    out.position[2] - fr.cam.pos[2],
  );
  const dy = Math.abs(Math.atan2(Math.sin(out.yaw - fr.cam.yaw), Math.cos(out.yaw - fr.cam.yaw)));
  const dp = Math.abs(out.pitch - fr.cam.pitch);
  maxPos = Math.max(maxPos, d); maxYaw = Math.max(maxYaw, dy); maxPitch = Math.max(maxPitch, dp);
}
const DEG = 180 / Math.PI;
console.log('样例:', path.basename(here), '| 帧数:', raw.frames.length, '| 时长:', shot.duration + 's');
console.log('重建最大偏差: 位置', maxPos.toFixed(9), 'm  偏航', (maxYaw * DEG).toFixed(9), '°  俯仰', (maxPitch * DEG).toFixed(9), '°');
console.log('原始记录核验: 前瞻帧', raw.verify.leadFrames, '主体帧', raw.verify.subjectFrames,
  '| 倒行最大偏航角速度', raw.verify.reverseMaxYawRateDeg.toFixed(3), '°/s',
  '| 主体投影最大|e|', raw.verify.subjectMaxErr.toFixed(3));
const ok = maxPos < 1e-9 && maxYaw < 1e-9 && maxPitch < 1e-9;
console.log(ok ? '重建一致' : '重建偏差超限');
process.exit(ok ? 0 : 1);
`;
}

// 重建脚本里指向仓库 src/ 的相对路径（样例可能生成在仓库内任意净目录）
function srcUrl(dir) {
  const rel = path.relative(dir, path.join(root, 'src', 'followCamera.js')).split(path.sep).join('/');
  return rel.startsWith('.') ? rel : `./${rel}`;
}

// ---------- 主流程 ----------
assertPristine(outDir);

const { track, camera, rows } = simulate();
const metrics = summarize(rows, camera);
const verify = verifyFromRaw(rows, camera);

console.log(`== 驾驶镜头（${FICTION_TRACK.name}）  净目录: ${path.relative(root, outDir) || outDir} ==`);
console.log(formatMetrics(metrics));
console.log(`  净目录核验          帧数=${rows.length}  前瞻帧=${verify.leadFrames}  主体帧=${verify.subjectFrames}` +
  `  前视距离低/高速弦长=${verify.leadChordLow.toFixed(2)}/${verify.leadChordHigh.toFixed(2)}m  随速度伸展=${verify.leadChordGrows}`);

// 强制约束：新鲜样片必须存在、原始记录必须自洽
const problems = [];
if (verify.leadFrames === 0) problems.push('没有前瞻帧');
if (verify.subjectFrames === 0) problems.push('没有主体帧（倒行段未触发）');
if (!verify.leadChordGrows) problems.push('前瞻距离未随速度伸展');
if (!verify.allSubjectInFront) problems.push('倒行时主体落在相机背后');
if (verify.subjectMaxErr === null || verify.subjectMaxErr > 0.6) problems.push(`主体投影偏差过大: ${verify.subjectMaxErr}`);
if (verify.reverseMaxYawRateDeg > 30) problems.push(`倒行偏航角速度过大: ${verify.reverseMaxYawRateDeg.toFixed(1)}°/s`);
if (!verify.steadyReverseLeadBehind) problems.push('缺少倒行前瞻落在相机背后的原始证据');
if (verify.vmax < 24) problems.push(`未达到高速: ${verify.vmax.toFixed(1)}m/s`);
if (verify.vminLong > -3) problems.push(`未真实倒车: ${verify.vminLong.toFixed(1)}m/s`);
if (problems.length) {
  console.error('\n样例核验失败，未写入任何样片：\n  - ' + problems.join('\n  - '));
  process.exit(1);
}

const frames = renderFrames(track, rows, camera, outDir);
const mapFile = renderMap(track, rows, outDir);

const rawFrames = rows.map((r) => ({
  t: +r.t.toFixed(6), dt: r.dt,
  ctrl: r.ctrl,
  veh: { pos: r.veh.pos, vel: r.veh.vel, yaw: r.veh.yaw, yawRate: r.veh.yawRate },
  cam: { pos: r.cam.pos, yaw: r.cam.yaw, pitch: r.cam.pitch },
  lead: r.lead, subject: r.subject, target: r.target,
  targetKind: r.diag.targetKind,
  vLong: r.diag.vLong, speed: r.diag.speed,
  lookAhead: r.diag.lookAhead, leadChord: r.diag.leadChord,
  leadBehindCamera: r.diag.leadBehindCamera,
  subjectProj: r.diag.subject,
  ax: r.diag.ax, ay: r.diag.ay,
  track: r.track,
}));

const raw = {
  generator: 'tools/buildDrivingShot.js',
  scenario: 'driving-line',
  track: { id: FICTION_TRACK.id, name: FICTION_TRACK.name, width: FICTION_TRACK.width, length: track.length },
  dt: SHOT_DT,
  phases: SHOT_PHASES,
  verify,
  frames: rawFrames,
};
const rawText = JSON.stringify(raw);
fs.writeFileSync(path.join(outDir, 'raw.json'), rawText);
const shot = {
  schema: 'driving-shot/v1',
  scenario: 'driving-line',
  track: raw.track,
  duration: SHOT_DURATION,
  dt: SHOT_DT,
  cameraParams: { ...CAM_PARAMS },
  phases: SHOT_PHASES,
  frames,
  map: mapFile,
  metrics,
  verify,
  raw: { file: 'raw.json', sha256: sha256(rawText), frames: rawFrames.length },
};
fs.writeFileSync(path.join(outDir, 'shot.json'), JSON.stringify(shot, null, 2));
fs.writeFileSync(path.join(outDir, 'replayDrivingShot.js'), replaySource(srcUrl(outDir)));

// 重建必须自洽：立即用刚写出的脚本核验
const { execFileSync } = await import('node:child_process');
try {
  const out = execFileSync(process.execPath, [path.join(outDir, 'replayDrivingShot.js')], { encoding: 'utf8' });
  console.log('\n' + out.trim());
} catch (err) {
  console.error('\n重建脚本自检失败:\n' + (err.stdout || '') + (err.stderr || ''));
  process.exit(1);
}

console.log(`\n已写入: ${fs.readdirSync(outDir).sort().join(', ')}`);
