// 虚构赛道：由直线与圆弧段定义中心线，提供弧长采样、最近点投影与路侧障碍生成。
// 场景生成（tools/buildDrivingShot.js）、演示与测试共用同一份几何，避免三处各写一套。
import { clamp } from './math3d.js';

export const FICTION_TRACK = {
  id: 'fiction-ring-v1',
  name: '虚构环线（长直道 + 高速左弯）',
  width: 26,
  segments: [
    { type: 'straight', length: 190 },
    { type: 'arc', radius: 75, angleDeg: 90 },
    { type: 'straight', length: 160 },
  ],
};

// step: 采样间距（米）。返回的点带累积弧长 s 与航向 yaw。
export function buildTrack(def = FICTION_TRACK, step = 2) {
  const points = [{ x: 0, z: 0, s: 0, yaw: 0 }];
  let x = 0, z = 0, yaw = 0, s = 0;
  for (const sg of def.segments) {
    if (sg.type === 'straight') {
      const n = Math.max(1, Math.ceil(sg.length / step));
      const ds = sg.length / n;
      for (let i = 1; i <= n; i++) {
        x += Math.sin(yaw) * ds;
        z += Math.cos(yaw) * ds;
        s += ds;
        points.push({ x, z, s, yaw });
      }
      continue;
    }
    if (sg.type === 'arc') {
      const ang = ((sg.angleDeg ?? 90) * Math.PI) / 180;
      const sign = Math.sign(ang) || 1;
      const R = sg.radius;
      const arcLen = Math.abs(ang) * R;
      const n = Math.max(1, Math.ceil(arcLen / step));
      const x0 = x, z0 = z, yaw0 = yaw;
      // 圆心在转向侧：左转(正角)时在左侧，右转时在右侧
      const cx = x0 + Math.cos(yaw0) * R * sign;
      const cz = z0 - Math.sin(yaw0) * R * sign;
      const vx = x0 - cx, vz = z0 - cz;
      for (let i = 1; i <= n; i++) {
        const a = (ang * i) / n;
        const c = Math.cos(a), sn = Math.sin(a);
        x = cx + c * vx + sn * vz;
        z = cz - sn * vx + c * vz;
        s += arcLen / n;
        yaw = yaw0 + a;
        points.push({ x, z, s, yaw });
      }
      continue;
    }
    throw new Error(`未知赛道段类型: ${sg.type}`);
  }

  const length = s;
  const width = def.width ?? 26;

  function pointAt(dist) {
    const target = clamp(dist, 0, length);
    let lo = 0, hi = points.length - 1;
    while (lo + 1 < hi) {
      const mid = (lo + hi) >> 1;
      if (points[mid].s <= target) lo = mid; else hi = mid;
    }
    const a = points[lo], b = points[hi];
    const span = b.s - a.s;
    const k = span > 1e-9 ? (target - a.s) / span : 0;
    const dx = b.x - a.x, dz = b.z - a.z;
    const dl = Math.hypot(dx, dz) || 1;
    return {
      x: a.x + dx * k,
      z: a.z + dz * k,
      tx: dx / dl,
      tz: dz / dl,
      yaw: a.yaw + (b.yaw - a.yaw) * k,
      s: target,
    };
  }

  // 世界点 → 中心线最近点。lateral 为相对前进方向的左侧偏移（米）。
  function project(wx, wz) {
    let best = null;
    for (let i = 1; i < points.length; i++) {
      const a = points[i - 1], b = points[i];
      const dx = b.x - a.x, dz = b.z - a.z;
      const L2 = dx * dx + dz * dz;
      if (L2 < 1e-12) continue;
      const u = clamp(((wx - a.x) * dx + (wz - a.z) * dz) / L2, 0, 1);
      const px = a.x + dx * u, pz = a.z + dz * u;
      const d2 = (wx - px) ** 2 + (wz - pz) ** 2;
      if (!best || d2 < best.d2) {
        const L = Math.sqrt(L2);
        const tx = dx / L, tz = dz / L;
        best = {
          d2, x: px, z: pz, tx, tz,
          lateral: (wx - px) * tz - (wz - pz) * tx,
          s: a.s + L * u,
        };
      }
    }
    return best;
  }

  function edges(offset) {
    const left = [], right = [];
    for (const p of points) {
      const lx = Math.cos(p.yaw), lz = -Math.sin(p.yaw);
      left.push([p.x + lx * offset, p.z + lz * offset]);
      right.push([p.x - lx * offset, p.z - lz * offset]);
    }
    return { left, right };
  }

  return { def, width, points, length, pointAt, project, edges };
}

// 路侧隔离墩：沿两侧按间距摆放，返回与 scene.js 相同的障碍物结构。
export function trackBarriers(track, spacing = 14) {
  const obs = [];
  const off = track.width / 2 + 1.5;
  let k = 0;
  for (let s = 6; s < track.length - 4; s += spacing) {
    const p = track.pointAt(s);
    const lx = Math.cos(p.yaw), lz = -Math.sin(p.yaw);
    for (const side of [1, -1]) {
      const h = 1.1;
      obs.push({
        center: [p.x + lx * off * side, h / 2, p.z + lz * off * side],
        size: [0.7, h, 2.6],
        yaw: p.yaw,
      });
      k++;
    }
  }
  return obs;
}
