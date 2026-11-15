// 载具跟随镜头验证套件：加速、连续弯道、侧滑轨迹 + 帧率无关性 + 死区投影语义。
// 构图误差与镜头运动成本分别度量、分别断言。
import { FollowCamera, predictLeadArc, predictLeadNaive } from '../src/followCamera.js';
import { Vehicle } from '../src/vehicle.js';
import { SCENARIOS, runScenario } from '../src/trajectories.js';
import { buildTrack, FICTION_TRACK } from '../src/track.js';
import { runShot, SHOT_DT } from '../src/drivingShot.js';
import { summarize, formatMetrics } from '../src/metrics.js';
import { wrapPi } from '../src/math3d.js';
import fs from 'node:fs';

const DEG = 180 / Math.PI;
let failures = 0;
function check(name, cond, detail = '') {
  console.log(`  ${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? `  | ${detail}` : ''}`);
  if (!cond) failures++;
}
const f = (x, n = 3) => Number(x).toFixed(n);
const horiz = (v) => Math.hypot(v[0], v[2]);
const dist2 = (a, b) => Math.hypot(a[0] - b[0], a[2] - b[2]);

function freshCam(over = {}) {
  return new FollowCamera({ aspect: 16 / 9, ...over });
}

console.log('== 通用约束 ==');
{
  const cam = freshCam();
  check('视场角保持常规(不靠扩大FOV掩盖前瞻失效)', cam.fovYDeg <= 70, `fovY=${cam.fovYDeg}°`);
}

// ---------- 场景一：加速 ----------
console.log('\n== 场景: 加速 ==');
{
  const v = new Vehicle();
  const cam = freshCam();
  const rows = runScenario(v, cam, SCENARIOS.accel, 1 / 60);
  const m = summarize(rows, cam);
  console.log(formatMetrics(m));

  const vmax = Math.max(...rows.map((r) => horiz(r.veh.vel)));
  check('车辆达到高速', vmax > 24, `vmax=${f(vmax, 1)}m/s`);

  const leadDist = rows.map((r) => dist2(r.lead, r.veh.pos));
  const atSpeed = (s) => {
    let best = 0, bd = 1e9;
    rows.forEach((r, i) => {
      const d = Math.abs(horiz(r.veh.vel) - s);
      if (d < bd) { bd = d; best = i; }
    });
    return best;
  };
  const i10 = atSpeed(10), i25 = atSpeed(25);
  check('前视距离随速度伸展', leadDist[i25] > leadDist[i10] + 3,
    `v10→${f(leadDist[i10], 1)}m  v25→${f(leadDist[i25], 1)}m`);

  check('偏航角速度不超限', m.motion.maxYawRate <= cam.maxYawRate * 1.05,
    `max=${f(m.motion.maxYawRate * DEG, 1)}°/s 限=${f(cam.maxYawRate * DEG, 0)}°/s`);
  check('俯仰角速度不超限', m.motion.maxPitchRate <= cam.maxPitchRate * 1.05,
    `max=${f(m.motion.maxPitchRate * DEG, 1)}°/s`);
  check('前方留白充足', m.leadRoom.minAhead >= 0.2, `min=${f(m.leadRoom.minAhead)}`);
  check('构图误差收敛', m.composition.meanErr < 0.25, `mean|e|=${f(m.composition.meanErr)}`);

  const camDists = rows.map((r) => dist2(r.cam.pos, r.veh.pos));
  check('非固定距离尾随(距离随工况变化)', Math.max(...camDists) - Math.min(...camDists) > 1.0,
    `Δ=${f(Math.max(...camDists) - Math.min(...camDists), 2)}m`);
}

// ---------- 场景二：连续弯道 ----------
console.log('\n== 场景: 连续弯道 ==');
{
  const v = new Vehicle();
  const cam = freshCam({ lookAheadDist: 14 });
  const rows = runScenario(v, cam, SCENARIOS.curve, 1 / 60);
  const m = summarize(rows, cam);
  console.log(formatMetrics(m));

  // 圆周段(t∈[11,15])：弧线前瞻 vs 朴素直线外推，对照真实未来位置
  const arcErr = [], naiveErr = [];
  for (const r of rows) {
    if (r.t < 11 || r.t > 14.5) continue;
    const speed = horiz(r.veh.vel);
    if (speed < 12 || Math.abs(r.veh.yawRate) < 0.4) continue;
    const distEff = cam.lookAheadDist + speed * cam.speedLeadTime;
    const T = Math.min(distEff / speed, 2.5);
    const fut = rows.find((q) => q.t >= r.t + T);
    if (!fut) continue;
    const fwd = [Math.sin(r.veh.yaw), 0, Math.cos(r.veh.yaw)];
    const arc = predictLeadArc(r.veh.pos, r.veh.vel, r.veh.yawRate, distEff, cam.maxLateral, fwd);
    const naive = predictLeadNaive(r.veh.pos, r.veh.vel, distEff);
    arcErr.push(dist2(arc, fut.veh.pos));
    naiveErr.push(dist2(naive, fut.veh.pos));
  }
  const mean = (a) => a.reduce((s, x) => s + x, 0) / a.length;
  check('弧线前瞻贴合真实路径', mean(arcErr) < 2.0, `arc=${f(mean(arcErr), 2)}m`);
  check('直线外推被抛离路径(对照组)', mean(naiveErr) > 4.0, `naive=${f(mean(naiveErr), 2)}m`);
  check('前瞻显著优于直线外推', mean(arcErr) < mean(naiveErr) * 0.4,
    `${f(mean(arcErr), 2)}m vs ${f(mean(naiveErr), 2)}m`);

  check('偏航角速度不超限', m.motion.maxYawRate <= cam.maxYawRate * 1.05,
    `max=${f(m.motion.maxYawRate * DEG, 1)}°/s`);
  check('相机确实跟随转向(角行程)', m.motion.yawTravel > 0.6,
    `总偏航=${f(m.motion.yawTravel * DEG, 0)}°`);
  check('前方留白充足', m.leadRoom.minAhead >= 0.15, `min=${f(m.leadRoom.minAhead)}`);
  check('构图误差有界', m.composition.meanErr < 0.45, `mean|e|=${f(m.composition.meanErr)}`);
}

// ---------- 场景三：侧滑 ----------
console.log('\n== 场景: 侧滑 ==');
{
  const v = new Vehicle();
  const cam = freshCam();
  const rows = runScenario(v, cam, SCENARIOS.sideslip, 1 / 60);
  const m = summarize(rows, cam);
  console.log(formatMetrics(m));

  let maxSlip = 0, leadAngleMax = 0, signChanges = 0, prevSign = 0;
  for (const r of rows) {
    if (r.t < 3.55 || r.t > 6.2) continue;
    const speed = horiz(r.veh.vel);
    if (speed < 8) continue;
    const velAng = Math.atan2(r.veh.vel[0], r.veh.vel[2]);
    const slip = Math.abs(wrapPi(velAng - r.veh.yaw));
    if (slip > maxSlip) maxSlip = slip;
    const leadAng = Math.atan2(r.lead[0] - r.veh.pos[0], r.lead[2] - r.veh.pos[2]);
    const la = Math.abs(wrapPi(leadAng - velAng));
    if (la > leadAngleMax) leadAngleMax = la;
    const s = Math.sign(r.diag.dYaw);
    if (s !== 0) {
      if (prevSign !== 0 && s !== prevSign) signChanges++;
      prevSign = s;
    }
  }
  check('侧滑真实发生(航向≠速度方向)', maxSlip > 15 / DEG, `max=${f(maxSlip * DEG, 1)}°`);
  check('前瞻跟随速度方向(漂移感知)', leadAngleMax < 10 / DEG, `max偏差=${f(leadAngleMax * DEG, 1)}°`);
  check('镜头无振荡', signChanges <= 6, `变向${signChanges}次`);
  check('构图误差有界', m.composition.maxErr < 0.9, `max|e|=${f(m.composition.maxErr)}`);
  check('偏航角速度不超限', m.motion.maxYawRate <= cam.maxYawRate * 1.05,
    `max=${f(m.motion.maxYawRate * DEG, 1)}°/s`);
}

// ---------- 帧率无关性 ----------
console.log('\n== 帧率无关性(同一载具轨迹,不同相机更新步长) ==');
{
  const refV = new Vehicle();
  const refCam = freshCam({ lookAheadDist: 14 });
  const refRows = runScenario(refV, refCam, SCENARIOS.curve, 1 / 240);
  const states = refRows.map((r) => ({
    t: r.t,
    position: r.veh.pos, velocity: r.veh.vel, yawRate: r.veh.yawRate,
    forward: [Math.sin(r.veh.yaw), 0, Math.cos(r.veh.yaw)],
  }));
  const sampleAt = (t) => {
    const dt = 1 / 240;
    const i = Math.min(states.length - 2, Math.max(0, Math.floor(t / dt)));
    const a = states[i], b = states[i + 1];
    const k = Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t || 1)));
    const L = (x, y) => x + (y - x) * k;
    return {
      position: a.position.map((p, j) => L(p, b.position[j])),
      velocity: a.velocity.map((p, j) => L(p, b.velocity[j])),
      yawRate: L(a.yawRate, b.yawRate),
      forward: (() => {
        const fx = L(a.forward[0], b.forward[0]);
        const fz = L(a.forward[2], b.forward[2]);
        const l = Math.hypot(fx, fz) || 1;
        return [fx / l, 0, fz / l];
      })(),
    };
  };
  const replay = (dts) => {
    const cam = freshCam({ lookAheadDist: 14 });
    const out = [];
    let t = 0, i = 0;
    while (t <= 15.5) {
      const dt = typeof dts === 'function' ? dts(i) : dts;
      const o = cam.update(dt, sampleAt(t));
      out.push({ t, yaw: o.yaw, pitch: o.pitch, ax: o.diag.ax, ay: o.diag.ay });
      t += dt;
      i++;
    }
    return out;
  };
  const r30 = replay(1 / 30), r60 = replay(1 / 60), r120 = replay(1 / 120);
  let seed = 3;
  const rJitter = replay(() => {
    seed = (seed * 1664525 + 1013904223) >>> 0;
    return 1 / 90 + (seed / 4294967296) * (1 / 45 - 1 / 90);
  });

  const yawAt = (rec, t) => {
    let best = rec[0], bd = 1e9;
    for (const r of rec) {
      const d = Math.abs(r.t - t);
      if (d < bd) { bd = d; best = r; }
    }
    return best;
  };
  let maxDev = 0, maxCompDev = 0;
  for (let t = 1; t <= 15; t += 0.5) {
    const y60 = yawAt(r60, t);
    for (const rec of [r30, r120, rJitter]) {
      const y = yawAt(rec, t);
      maxDev = Math.max(maxDev, Math.abs(wrapPi(y.yaw - y60.yaw)));
      maxCompDev = Math.max(maxCompDev, Math.abs(y.ax - y60.ax), Math.abs(y.ay - y60.ay));
    }
  }
  check('偏航轨迹与步长无关', maxDev < 0.03, `最大偏差=${f(maxDev * DEG, 2)}°`);
  check('构图结果与步长无关', maxCompDev < 0.08, `最大偏差=${f(maxCompDev)}`);
}

// ---------- 死区投影语义 ----------
console.log('\n== 死区投影空间语义 ==');
{
  const s0 = { position: [0, 0, 0], velocity: [0, 0, 0], yawRate: 0, forward: [0, 0, 1] };
  const cam = freshCam();
  cam.update(0, s0);
  const dir = [0.15, 0.05, 1];
  const l = Math.hypot(...dir);
  const u = dir.map((x) => x / l);
  const p1 = [cam.pos[0] + u[0] * 10, cam.pos[1] + u[1] * 10, cam.pos[2] + u[2] * 10];
  const p2 = [cam.pos[0] + u[0] * 40, cam.pos[1] + u[1] * 40, cam.pos[2] + u[2] * 40];
  const a = cam.projectPoint(p1), b = cam.projectPoint(p2);
  check('同一视线方向不同距离 → 同一投影坐标',
    Math.abs(a.ax - b.ax) < 1e-9 && Math.abs(a.ay - b.ay) < 1e-9,
    `10m:(${f(a.ax)},${f(a.ay)}) 40m:(${f(b.ax)},${f(b.ay)})`);

  const cam2 = freshCam({ aspect: 1.0 });
  cam2.update(0, s0);
  cam2.pos = [...cam.pos];
  cam2.yaw = cam.yaw;
  cam2.pitch = cam.pitch;
  const c = cam2.projectPoint(p1);
  check('长宽比变化 → 修正坐标含义不变',
    Math.abs(a.ax - c.ax) < 1e-9 && Math.abs(a.ay - c.ay) < 1e-9,
    `16:9:(${f(a.ax)},${f(a.ay)}) 1:1:(${f(c.ax)},${f(c.ay)})`);

  const v = new Vehicle();
  const cam3 = freshCam();
  const rows = runScenario(v, cam3, {
    name: '匀速直行', duration: 9,
    controls: () => ({ throttle: 0.6, steer: 0 }),
  }, 1 / 60);
  const tail = rows.filter((r) => r.t > 7.5);
  const maxRate = Math.max(...tail.map((r) => Math.abs(r.diag.dYaw) / r.dt));
  const maxExcess = Math.max(...tail.map((r) => {
    const ex = Math.abs(r.diag.ax - cam3.anchor.x) - cam3.deadZone.x;
    const ey = Math.abs(r.diag.ay - cam3.anchor.y) - cam3.deadZone.y;
    return Math.max(ex, ey);
  }));
  check('目标进入死区后不再旋转', maxRate < 0.75 / DEG, `尾部最大角速度=${f(maxRate * DEG, 2)}°/s`);
  check('尾部目标贴在死区边缘(溢出<0.01)', maxExcess < 0.01, `最大溢出=${f(maxExcess, 4)}`);
}

// ---------- 驾驶镜头：虚构赛道 + 倒行主体构图 ----------
console.log('\n== 驾驶镜头: 虚构赛道（高速弯 + 急停 + 倒行） ==');
{
  const track = buildTrack(FICTION_TRACK);
  const v = new Vehicle();
  const cam = freshCam({ lookAheadDist: 12 });
  const rows = runShot(v, cam, track, SHOT_DT);
  const m = summarize(rows, cam);

  const speeds = rows.map((r) => r.diag.speed);
  const vmax = Math.max(...speeds);
  const vminLong = Math.min(...rows.map((r) => r.diag.vLong));
  check('驾驶镜头达到高速', vmax > 24, `vmax=${f(vmax, 1)}m/s`);
  check('驾驶镜头真实倒车', vminLong < -4, `最小纵向速度=${f(vminLong, 2)}m/s`);

  const rev = rows.filter((r) => r.diag.reverseMode);
  check('倒行段切入主体构图', rev.length > 30 && rev.every((r) => r.diag.targetKind === 'subject'),
    `倒行帧=${rev.length}`);
  // 触发条件核验：稳定倒车（纵向速度 ≤ -3 m/s）时，按速度方向推的原始前瞻点
  // 落到相机背后（leadBehindCamera，由镜头模块逐帧记录点乘结果）。
  const steadyRev = rev.filter((r) => r.diag.vLong <= -3);
  const behindCount = steadyRev.filter((r) => r.diag.leadBehindCamera).length;
  check('稳定倒车时前瞻点落在相机背后（说明必须改瞄准主体）',
    steadyRev.length > 0 && behindCount === steadyRev.length,
    `${behindCount}/${steadyRev.length} 帧`);
  check('前进段前瞻点始终在相机前方', rows.filter((r) => r.diag.targetKind === 'lead')
    .every((r) => !r.diag.leadBehindCamera), '逐帧检查');
  check('倒行主体始终在相机前方', rev.every((r) => !r.diag.subject.behind), `帧数=${rev.length}`);
  check('倒行无镜头翻转(偏航角速度受限)',
    Math.max(...rev.map((r) => Math.abs(r.diag.dYaw) / r.diag.dt)) <= cam.maxYawRate * 1.05,
    `max=${f(m.reverse.maxYawRate * DEG, 1)}°/s`);
  check('主体投影偏差受控', m.subjectComposition.maxErr < 0.6, `max|e|=${f(m.subjectComposition.maxErr)}`);
  check('构图误差整体有界', m.composition.maxErr < 0.5, `max|e|=${f(m.composition.maxErr)}`);
  check('后方倒行不引入镜头大回转', m.motion.yawTravel < Math.PI * 1.5,
    `总偏航=${f(m.motion.yawTravel * DEG, 0)}°`);

  // 前瞻距离核验：沿原始记录直接量弦长，高速应显著大于低速
  const leadRows = rows.filter((r) => r.diag.targetKind === 'lead');
  const chord = (r) => Math.hypot(r.lead[0] - r.veh.pos[0], r.lead[2] - r.veh.pos[2]);
  const near = (s) => leadRows.reduce((a, b) => (Math.abs(b.diag.speed - s) < Math.abs(a.diag.speed - s) ? b : a));
  const lo = chord(near(8)), hi = chord(near(26));
  check('前瞻距离随速度伸展（原始记录弦长）', hi > lo + 3, `${f(lo, 2)}m → ${f(hi, 2)}m`);
  check('前视距离与配置口径一致(带速度项)',
    Math.abs(hi - (12 + 27 * 0.35)) < 4, `实测=${f(hi, 2)}m`);

  // 急停段：纵向速度由高速穿过 0
  const brakeRows = rows.filter((r) => r.ctrl.kind === 'brake');
  const crossedZero = brakeRows.some((r) => Math.abs(r.diag.vLong) < 1);
  check('急停段确实把速度刹到近零', crossedZero && brakeRows.length > 20, `急停帧=${brakeRows.length}`);
}

// ---------- 旧样例回归：新模块必须让旧记录逐帧复现 ----------
console.log('\n== 旧样例回归（results/*.json 逐帧复现） ==');
for (const key of ['accel', 'curve', 'sideslip']) {
  const file = new URL(`../results/${key}.json`, import.meta.url);
  if (!fs.existsSync(file)) {
    check(`${key} 旧样例存在`, false, 'results 记录缺失');
    continue;
  }
  const data = JSON.parse(fs.readFileSync(file, 'utf8'));
  const cam = freshCam({ lookAheadDist: data.lookAheadDist });
  let maxPos = 0, maxYaw = 0, maxPitch = 0;
  let replayRows = 0;
  for (const r of data.rows) {
    const out = cam.update(r.dt, {
      position: r.veh.pos, velocity: r.veh.vel, yawRate: r.veh.yawRate,
      forward: [Math.sin(r.veh.yaw), 0, Math.cos(r.veh.yaw)],
    });
    maxPos = Math.max(maxPos, Math.hypot(
      out.position[0] - r.cam.pos[0], out.position[1] - r.cam.pos[1], out.position[2] - r.cam.pos[2]));
    maxYaw = Math.max(maxYaw, Math.abs(wrapPi(out.yaw - r.cam.yaw)));
    maxPitch = Math.max(maxPitch, Math.abs(out.pitch - r.cam.pitch));
    replayRows++;
  }
  check(`${key} 旧样例完整重放`, replayRows === data.rows.length && replayRows > 0, `帧数=${replayRows}`);
  check(`${key} 旧样例位姿逐帧一致`,
    maxPos < 1e-9 && maxYaw < 1e-9 && maxPitch < 1e-9,
    `位置=${maxPos.toExponential(2)}m 偏航=${f(maxYaw * DEG, 9)}° 俯仰=${f(maxPitch * DEG, 9)}°`);
}

console.log(`\n${failures === 0 ? '全部通过' : `失败 ${failures} 项`}`);
process.exit(failures === 0 ? 0 : 1);
