// 辅助命令行：无头跑载具镜头场景，输出构图误差/运动成本指标，
// --save 时把轨迹记录(JSON)与真实相机视图(PNG)写入 results/。
// 图形入口见 README（demo/），此命令行仅作辅助。
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { FollowCamera } from '../src/followCamera.js';
import { Vehicle } from '../src/vehicle.js';
import { SCENARIOS, runScenario } from '../src/trajectories.js';
import { summarize, formatMetrics } from '../src/metrics.js';
import { obstaclesAlongPath } from '../src/scene.js';
import { lookAt, perspective } from '../src/math3d.js';
import {
  createImage, writePNG, makeProjector,
  drawLine3, drawBox3, drawGrid, drawCross3, drawLine,
} from '../src/softrender.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const which = args.find((a) => !a.startsWith('--')) ?? 'all';
const save = args.includes('--save');
const lookArg = args.find((a) => a.startsWith('--look='));
const lookAheadDist = lookArg ? Number(lookArg.split('=')[1]) : 12;
const dtArg = args.find((a) => a.startsWith('--dt='));
const dt = dtArg ? Number(dtArg.split('=')[1]) : 1 / 60;

const keys = which === 'all' ? Object.keys(SCENARIOS) : [which];
let failed = 0;

for (const key of keys) {
  const sc = SCENARIOS[key];
  if (!sc) {
    console.error(`未知场景: ${key}（可选 ${Object.keys(SCENARIOS).join('/')}）`);
    process.exit(2);
  }
  const vehicle = new Vehicle();
  const camera = new FollowCamera({ aspect: 16 / 9, lookAheadDist });
  const rows = runScenario(vehicle, camera, sc, dt);
  const m = summarize(rows, camera);
  console.log(`\n== ${sc.name} (${key})  dt=${dt.toFixed(4)}s  前视距离=${lookAheadDist}m  样本=${rows.length} ==`);
  console.log(formatMetrics(m));

  if (save) {
    const outDir = path.join(root, 'results');
    fs.mkdirSync(outDir, { recursive: true });
    fs.writeFileSync(
      path.join(outDir, `${key}.json`),
      JSON.stringify({ scenario: key, dt, lookAheadDist, params: { ...camera, pos: undefined }, metrics: m, rows }, null, 1),
    );
    renderViews(key, sc, rows, camera, outDir);
    renderMap(key, rows, outDir);
    console.log(`  已写入 results/${key}.json, results/${key}_view_t*.png, results/${key}_map.png`);
  }
}

function sceneObstacles(rows) {
  return obstaclesAlongPath(rows.map((r) => r.veh.pos), 7, 6);
}

function renderViews(key, sc, rows, camera, outDir) {
  const W = 960, H = 540;
  const obstacles = sceneObstacles(rows);
  const projM = perspective(camera.fovYRad, 16 / 9, 0.1, 400);
  for (const frac of [0.3, 0.6, 0.9]) {
    const idx = Math.min(rows.length - 1, Math.floor(rows.length * frac));
    // 用该时刻的相机位姿与截至当时的轨迹
    const row = rows[idx];
    const img = createImage(W, H);
    const fwd = [
      Math.cos(row.cam.pitch) * Math.sin(row.cam.yaw),
      Math.sin(row.cam.pitch),
      Math.cos(row.cam.pitch) * Math.cos(row.cam.yaw),
    ];
    const view = lookAt(row.cam.pos, [
      row.cam.pos[0] + fwd[0], row.cam.pos[1] + fwd[1], row.cam.pos[2] + fwd[2],
    ], [0, 1, 0]);
    const proj = makeProjector(view, projM, W, H);
    drawGrid(img, proj);
    for (const o of obstacles) drawBox3(img, proj, o.center, o.size, o.yaw, [96, 110, 128]);
    // 截至当时的载具轨迹（白）与相机轨迹（青）
    for (let i = 1; i <= idx; i++) {
      const a = rows[i - 1], b = rows[i];
      drawLine3(img, proj, [a.veh.pos[0], 0.05, a.veh.pos[2]], [b.veh.pos[0], 0.05, b.veh.pos[2]], [220, 220, 220]);
      drawLine3(img, proj, a.cam.pos, b.cam.pos, [70, 190, 200]);
    }
    // 载具本体（橙）与前瞻点（红十字）及连线（黄）
    drawBox3(img, proj, [row.veh.pos[0], 0.8, row.veh.pos[2]], [2.2, 1.4, 4.2], row.veh.yaw, [230, 140, 40]);
    drawCross3(img, proj, row.lead, 0.8, [240, 70, 70]);
    drawLine3(img, proj, [row.veh.pos[0], 0.8, row.veh.pos[2]], row.lead, [230, 210, 80]);
    writePNG(path.join(outDir, `${key}_view_t${row.t.toFixed(1)}.png`), W, H, img.data);
  }
}

function renderMap(key, rows, outDir) {
  const S = 720;
  const img = createImage(S, S, [22, 25, 30]);
  let minX = 1e9, maxX = -1e9, minZ = 1e9, maxZ = -1e9;
  for (const r of rows) {
    minX = Math.min(minX, r.veh.pos[0], r.cam.pos[0]);
    maxX = Math.max(maxX, r.veh.pos[0], r.cam.pos[0]);
    minZ = Math.min(minZ, r.veh.pos[2], r.cam.pos[2]);
    maxZ = Math.max(maxZ, r.veh.pos[2], r.cam.pos[2]);
  }
  const pad = 15;
  minX -= pad; maxX += pad; minZ -= pad; maxZ += pad;
  const k = Math.min(S / (maxX - minX), S / (maxZ - minZ));
  const px = (x) => (x - minX) * k + (S - (maxX - minX) * k) / 2;
  const pz = (z) => S - ((z - minZ) * k + (S - (maxZ - minZ) * k) / 2);
  for (let i = 1; i < rows.length; i++) {
    const a = rows[i - 1], b = rows[i];
    drawLine(img, px(a.veh.pos[0]), pz(a.veh.pos[2]), px(b.veh.pos[0]), pz(b.veh.pos[2]), [230, 230, 230]);
    drawLine(img, px(a.cam.pos[0]), pz(a.cam.pos[2]), px(b.cam.pos[0]), pz(b.cam.pos[2]), [70, 190, 200]);
  }
  // 前瞻点采样（红）
  for (let i = 0; i < rows.length; i += 30) {
    const r = rows[i];
    drawLine(img, px(r.lead[0]) - 2, pz(r.lead[2]), px(r.lead[0]) + 2, pz(r.lead[2]), [240, 80, 80]);
    drawLine(img, px(r.lead[0]), pz(r.lead[2]) - 2, px(r.lead[0]), pz(r.lead[2]) + 2, [240, 80, 80]);
  }
  writePNG(path.join(outDir, `${key}_map.png`), S, S, img.data);
}

process.exit(failed);
