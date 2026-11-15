// 运行指标汇总：构图误差与镜头运动成本分别度量，互不相混。
// 构图误差：前瞻目标在投影空间（长宽比修正坐标）相对期望锚点的偏差。
// 运动成本：相机角行程与角速度（偏航/俯仰分开）。
// 前方留白：前瞻点投影到画面后，距画面边缘的剩余空间（NDC 单位）。
export function summarize(rows, camera) {
  const A = camera.anchor;
  const rs = rows.filter((r) => r.t >= 0.6 && r.diag);
  if (!rs.length) return null;
  let sumE = 0, maxE = 0, sumEx = 0, sumEy = 0, maxEx = 0, maxEy = 0;
  let yawTravel = 0, pitchTravel = 0, maxYawRate = 0, maxPitchRate = 0;
  let minAhead = Infinity, minSide = Infinity, inside = 0;
  for (const r of rs) {
    const ex = r.diag.ax - A.x;
    const ey = r.diag.ay - A.y;
    const e = Math.hypot(ex, ey);
    sumE += e;
    if (e > maxE) maxE = e;
    sumEx += Math.abs(ex);
    sumEy += Math.abs(ey);
    if (Math.abs(ex) > maxEx) maxEx = Math.abs(ex);
    if (Math.abs(ey) > maxEy) maxEy = Math.abs(ey);
    if (r.diag.dt > 0) {
      yawTravel += Math.abs(r.diag.dYaw);
      pitchTravel += Math.abs(r.diag.dPitch);
      const yr = Math.abs(r.diag.dYaw) / r.diag.dt;
      const pr = Math.abs(r.diag.dPitch) / r.diag.dt;
      if (yr > maxYawRate) maxYawRate = yr;
      if (pr > maxPitchRate) maxPitchRate = pr;
    }
    if (r.diag.roomAhead < minAhead) minAhead = r.diag.roomAhead;
    if (r.diag.roomSide < minSide) minSide = r.diag.roomSide;
    if (r.diag.inDeadZone) inside++;
  }
  const n = rs.length;
  return {
    composition: {
      meanErr: sumE / n, maxErr: maxE,
      meanErrX: sumEx / n, meanErrY: sumEy / n,
      maxErrX: maxEx, maxErrY: maxEy,
    },
    motion: { yawTravel, pitchTravel, maxYawRate, maxPitchRate },
    leadRoom: { minAhead, minSide },
    deadZoneInside: inside / n,
    samples: n,
  };
}

export function formatMetrics(m) {
  const d = (x) => (x * 180 / Math.PI).toFixed(1);
  const f = (x) => x.toFixed(3);
  return [
    `  构图误差(投影单位)  平均|e|=${f(m.composition.meanErr)}  最大|e|=${f(m.composition.maxErr)}` +
    `  平均ex=${f(m.composition.meanErrX)}  平均ey=${f(m.composition.meanErrY)}`,
    `  镜头运动成本        偏航行程=${d(m.motion.yawTravel)}°  俯仰行程=${d(m.motion.pitchTravel)}°` +
    `  最大角速度: 偏航=${d(m.motion.maxYawRate)}°/s  俯仰=${d(m.motion.maxPitchRate)}°/s`,
    `  前方留白            最小上方留白=${f(m.leadRoom.minAhead)}  最小侧向留白=${f(m.leadRoom.minSide)}` +
    `  死区内占比=${(m.deadZoneInside * 100).toFixed(0)}%`,
  ].join('\n');
}
