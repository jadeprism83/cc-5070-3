// 场景道具生成（演示与命令行渲染共用）
export function obstaclesAlongPath(pts, offset = 7, spacing = 6) {
  const obs = [];
  let acc = 0;
  let k = 0;
  for (let i = 1; i < pts.length; i++) {
    const dx = pts[i][0] - pts[i - 1][0];
    const dz = pts[i][2] - pts[i - 1][2];
    const d = Math.hypot(dx, dz);
    if (d < 1e-6) continue;
    acc += d;
    if (acc < spacing) continue;
    acc = 0;
    const fx = dx / d, fz = dz / d;
    const left = [fz, -fx];
    for (const side of [1, -1]) {
      const h = 1.8 + ((k * 2654435761) % 100) / 100 * 1.6;
      obs.push({
        center: [pts[i][0] + left[0] * offset * side, h / 2, pts[i][2] + left[1] * offset * side],
        size: [1.8, h, 1.8],
        yaw: ((k * 37) % 90) * Math.PI / 180,
      });
      k++;
    }
  }
  return obs;
}

export function scatterObstacles(area = 90, count = 70, seed = 7, avoidRadius = 14) {
  let s = seed >>> 0;
  const rnd = () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
  const obs = [];
  for (let i = 0; i < count; i++) {
    const x = (rnd() * 2 - 1) * area;
    const z = (rnd() * 2 - 1) * area + 30;
    if (Math.hypot(x, z) < avoidRadius) continue;
    const h = 1.5 + rnd() * 2.5;
    obs.push({ center: [x, h / 2, z], size: [1.6 + rnd() * 1.6, h, 1.6 + rnd() * 1.6], yaw: rnd() * Math.PI });
  }
  return obs;
}
