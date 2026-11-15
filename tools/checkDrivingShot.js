// 入库样例核验：只读 samples/<样例>/ 的原始记录与样片，不重新仿真。
// 用途：CI / 复核“样片真实存在、前瞻距离与主体位置与原始记录一致”。
// 用法: node tools/checkDrivingShot.js [样例目录，默认 samples/driving-line]
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const dir = path.resolve(process.argv[2] ?? path.join(root, 'samples', 'driving-line'));
const fail = (msg) => { console.error(`  FAIL  ${msg}`); process.exitCode = 1; };
const pass = (msg, detail = '') => console.log(`  PASS  ${msg}${detail ? `  | ${detail}` : ''}`);

console.log(`== 样例核验: ${path.relative(root, dir) || dir} ==`);

const required = ['shot.json', 'raw.json', 'replayDrivingShot.js'];
for (const f of required) {
  if (!fs.existsSync(path.join(dir, f))) fail(`缺少入库文件 ${f}`);
}
if (process.exitCode) process.exit(process.exitCode);

const shotText = fs.readFileSync(path.join(dir, 'shot.json'), 'utf8');
const rawText = fs.readFileSync(path.join(dir, 'raw.json'), 'utf8');
const shot = JSON.parse(shotText);
const raw = JSON.parse(rawText);

// 1. 样片文件真实存在，且 shot.json 记录的帧数与 raw 一致
for (const fr of shot.frames) {
  const p = path.join(dir, fr.file);
  if (!fs.existsSync(p)) fail(`声明的样片不存在: ${fr.file}`);
  else pass(`样片存在 ${fr.file}`, `${fs.statSync(p).size} bytes`);
}
pass('地图样片存在', shot.map);
if (!fs.existsSync(path.join(dir, shot.map))) fail(`地图样片不存在: ${shot.map}`);

// 2. 原始记录哈希与 shot.json 一致（防止汇总与实际逐帧记录脱钩）
const hash = crypto.createHash('sha256').update(rawText).digest('hex');
if (hash !== shot.raw.sha256) fail(`raw.json 哈希不符（记录 ${shot.raw.sha256.slice(0, 12)}… / 实际 ${hash.slice(0, 12)}…）`);
else pass('原始记录哈希一致');
if (raw.frames.length !== shot.raw.frames) fail(`帧数不符: raw=${raw.frames.length} shot=${shot.raw.frames}`);
else pass('原始记录帧数一致', `${raw.frames.length} 帧`);

// 3. 前瞻距离核验：直接量原始记录里前瞻点与载具的距离，并核对随速度伸展
const lead = raw.frames.filter((f) => f.targetKind === 'lead');
const subj = raw.frames.filter((f) => f.targetKind === 'subject');
const chord = (f) => Math.hypot(f.lead[0] - f.veh.pos[0], f.lead[2] - f.veh.pos[2]);
const near = (s) => lead.reduce((a, b) => (Math.abs(b.speed - s) < Math.abs(a.speed - s) ? b : a));
const lo = chord(near(8)), hi = chord(near(26));
if (lead.length === 0) fail('原始记录没有前瞻帧');
else pass('前瞻帧存在', `${lead.length} 帧`);
if (!(hi > lo + 3)) fail(`前瞻距离未随速度伸展: ${lo.toFixed(2)}m → ${hi.toFixed(2)}m`);
else pass('前瞻距离随速度伸展', `${lo.toFixed(2)}m → ${hi.toFixed(2)}m`);
const storedLook = Math.max(...raw.frames.map((f) => f.lookAhead));
const vmaxPeak = Math.max(...raw.frames.map((f) => f.speed));
const expectedLook = shot.cameraParams.lookAheadDist + vmaxPeak * shot.cameraParams.speedLeadTime;
if (Math.abs(storedLook - expectedLook) > 1) {
  fail(`前视距离峰值与配置口径不符: 实测 ${storedLook.toFixed(2)}m，按 ${shot.cameraParams.lookAheadDist}m + 速度×${shot.cameraParams.speedLeadTime}s 应为 ${expectedLook.toFixed(2)}m`);
} else pass('前视距离峰值与配置口径一致', `${storedLook.toFixed(2)}m`);

// 4. 主体位置核验：倒行帧主体必须在相机前方，且投影位置与原始记录一致
if (!subj.length) fail('原始记录没有倒行主体帧');
else pass('倒行主体帧存在', `${subj.length} 帧`);
const behind = subj.filter((f) => f.subjectProj.behind);
if (behind.length) fail(`倒行时主体落在相机背后: ${behind.length} 帧`);
else pass('倒行主体始终在相机前方');
const maxSubjErr = Math.max(...subj.map((f) => Math.hypot(f.subjectProj.ax, f.subjectProj.ay)));
if (maxSubjErr > 0.6) fail(`主体投影偏差过大: ${maxSubjErr.toFixed(3)}`);
else pass('主体投影偏差受控', `max|e|=${maxSubjErr.toFixed(3)}`);
const steady = subj.filter((f) => f.vLong <= -3);
if (!steady.length || !steady.every((f) => f.leadBehindCamera)) {
  fail('缺少“稳定倒车时前瞻点落在相机背后”的原始证据');
} else pass('倒行前瞻落在相机背后的证据完整', `${steady.length} 帧`);
const revRate = Math.max(...subj.map((f) => Math.abs(f.cam.yaw)));
if (!Number.isFinite(revRate)) fail('倒行镜头数据异常');

// 5. 车辆确实高速与真实倒车
const vmax = Math.max(...raw.frames.map((f) => f.vLong));
const vmin = Math.min(...raw.frames.map((f) => f.vLong));
if (vmax < 24) fail(`未达到高速: ${vmax.toFixed(1)}m/s`); else pass('达到高速', `${vmax.toFixed(1)}m/s`);
if (vmin > -3) fail(`未真实倒车: ${vmin.toFixed(1)}m/s`); else pass('真实倒车', `${vmin.toFixed(1)}m/s`);

// 6. 重建脚本可执行且与原始记录一致
if (process.exitCode) {
  console.log('\n原始记录核验未通过，跳过重建（避免用不一致记录背书）。');
  process.exit(process.exitCode);
}
const { execFileSync } = await import('node:child_process');
try {
  const out = execFileSync(process.execPath, [path.join(dir, 'replayDrivingShot.js')], { encoding: 'utf8' });
  const line = out.split('\n').find((l) => l.includes('重建最大偏差')) ?? out.trim();
  pass('重建脚本可执行', line.replace('重建最大偏差: ', ''));
  if (!out.includes('重建一致')) fail('重建脚本报告不一致');
} catch (err) {
  fail(`重建脚本执行失败: ${(err.stdout || err.message).toString().split('\n')[0]}`);
}

console.log(process.exitCode ? '\n样例核验失败' : '\n样例核验通过');
