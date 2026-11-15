// 虚构赛道驾驶场景：沿中心线的路径驾驶控制器。
// 场景中每个时刻的油门/转向由“当前速度 + 赛道最近点 + 弧长前瞻”计算，
// 不是预录帧；演示、样例生成与测试都用这同一份控制器逐帧驱动。
import { clamp, wrapPi } from './math3d.js';
import { vehicleState } from './trajectories.js';

// 阶段：按时间切换驾驶意图。目标速度单位为 m/s（负值即倒挡）。
export const SHOT_PHASES = [
  { t: 0.0, kind: 'accel', target: 30, label: '全油门沿长直道加速' },
  { t: 8.6, kind: 'coast', target: 21, label: '松油门进入左弯前' },
  { t: 10.6, kind: 'hold', target: 21, label: '高速过左弯' },
  { t: 16.2, kind: 'accel', target: 27, label: '出弯加速' },
  { t: 17.4, kind: 'brake', target: 0, label: '急刹' },
  { t: 19.2, kind: 'reverse', target: -5, label: '倒车' },
  { t: 23.4, kind: 'stop', target: 0, label: '停稳' },
];

export const SHOT_DURATION = 24.0;
export const SHOT_DT = 1 / 60;

export function phaseAt(t) {
  let p = SHOT_PHASES[0];
  for (const q of SHOT_PHASES) if (t >= q.t) p = q;
  return p;
}

// 沿赛道前进的路径驾驶：以弧长前瞻点为目标航向，转向增益与横向误差修正。
export function driveAlongTrack(vehicle, track, opts = {}) {
  const lookDist = opts.lookDist ?? 10;
  const lookPerSpeed = opts.lookPerSpeed ?? 0.45;
  const steerGain = opts.steerGain ?? 3.0;
  const lateralGain = opts.lateralGain ?? 0.04;
  const fwd = vehicle.forward();
  const vLong = vehicle.vel[0] * fwd[0] + vehicle.vel[2] * fwd[2];
  const pr = track.project(vehicle.pos[0], vehicle.pos[2]);
  const look = lookDist + Math.abs(vLong) * lookPerSpeed;
  const tp = track.pointAt(pr.s + look);
  const des = Math.atan2(tp.x - vehicle.pos[0], tp.z - vehicle.pos[2]);
  return clamp(wrapPi(des - vehicle.yaw) * steerGain + pr.lateral * lateralGain, -1, 1);
}

// 每个阶段输出真实控制量（演示与样例共用）。返回 { throttle, steer, label }。
export function shotControls(vehicle, track, t, opts = {}) {
  const ph = phaseAt(t);
  const fwd = vehicle.forward();
  const vLong = vehicle.vel[0] * fwd[0] + vehicle.vel[2] * fwd[2];
  let throttle = 0;
  switch (ph.kind) {
    case 'accel':
      throttle = clamp((ph.target - vLong) * 0.3, 0, 1);
      break;
    case 'coast':
      throttle = vLong < ph.target - 1 ? clamp((ph.target - vLong) * 0.3, 0, 1) : 0;
      break;
    case 'hold':
      throttle = clamp((ph.target - vLong) * 0.3, -1, 1);
      break;
    case 'brake':
      throttle = vLong > 0.15 ? -1 : 0;
      break;
    case 'reverse':
      throttle = vLong > ph.target ? -1 : 0;
      break;
    case 'stop':
      throttle = clamp(-vLong * 0.3, -1, 1);
      break;
    default:
      throttle = 0;
  }
  // 倒挡与低速停车阶段不路径转向，避免打横摆
  const steer = ph.kind === 'reverse' || (ph.kind === 'stop' && Math.abs(vLong) < 2)
    ? 0
    : driveAlongTrack(vehicle, track, opts);
  return { throttle, steer, kind: ph.kind, target: ph.target, label: ph.label };
}

// 驾驶镜头场景的增量执行器。演示按真实帧 dt 调用，样例生成与测试按固定 dt 批跑；
// 每一行都记录当帧的控制量、载具状态、镜头位姿与前瞻/主体诊断，不做任何事后修补。
export function createShotRun(vehicle, camera, track, opts = {}) {
  const duration = opts.duration ?? SHOT_DURATION;
  let t = 0;
  const rows = [];
  const run = {
    rows,
    get t() { return t; },
    get done() { return t >= duration - 1e-9; },
    step(dt) {
      const ctrl = shotControls(vehicle, track, t, opts);
      vehicle.update(dt, ctrl);
      const out = camera.update(dt, vehicleState(vehicle));
      const trk = track.project(vehicle.pos[0], vehicle.pos[2]);
      const row = {
        t, dt,
        ctrl: { throttle: ctrl.throttle, steer: ctrl.steer, kind: ctrl.kind },
        veh: {
          pos: [...vehicle.pos], vel: [...vehicle.vel],
          yaw: vehicle.yaw, yawRate: vehicle.yawRate,
        },
        cam: { pos: [...out.position], yaw: out.yaw, pitch: out.pitch },
        lead: [...out.leadPoint],
        subject: [...out.subjectPoint],
        target: [...out.targetPoint],
        diag: { ...out.diag },
        track: { s: trk.s, lateral: trk.lateral },
      };
      rows.push(row);
      t += dt;
      return row;
    },
  };
  return run;
}

export function runShot(vehicle, camera, track, dt, opts = {}) {
  const run = createShotRun(vehicle, camera, track, opts);
  while (!run.done) run.step(dt);
  return run.rows;
}
