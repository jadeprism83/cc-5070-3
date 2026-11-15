// 高速载具跟随构图镜头核心模块。
// 设计要点：
//  - 死区定义在投影空间（按垂直半视场归一化的坐标），与画面长宽比、目标距离无关；
//  - 前瞻点沿“速度方向 + 横摆角速度”积分出的弧线预测，而不是 速度*常数 的直线外推，
//    转弯时视线沿路径走，不会被抛到弯道外侧障碍区；
//  - 旋转受限：偏航/俯仰角速度有硬上限，且只对超出死区的部分纠偏（回到死区边缘而非中心）；
//  - 所有积分均为显式时间策略（1-exp(-k*dt) 精确阻尼 + 角速度*dt 限幅），与帧率无关。
import { sub, dot, norm, clamp, wrapPi, cross } from './math3d.js';

const DEG = Math.PI / 180;

// 弧线前瞻：假设当前速率与横摆角速度在预测窗内近似不变，对旋转中的速度方向积分。
// ω→0 时退化为直线。横向偏移相对载具航向被 maxLateral 限制，避免视角被甩出过远。
export function predictLeadArc(pos, vel, yawRate, dist, maxLateral, forward) {
  const speed = Math.hypot(vel[0], vel[2]);
  const f = norm([forward[0], 0, forward[2]]);
  const dir = speed > 0.5 ? [vel[0] / speed, 0, vel[2] / speed] : f;
  const T = Math.min(dist / Math.max(speed, 0.5), 2.5);
  let off;
  const w = yawRate || 0;
  if (Math.abs(w) > 1e-3 && speed > 0.5) {
    // ∫0^T rotY(ωt)·v dt = sin(ωT)/ω·v + (1-cos(ωT))/ω·(Jv)，Jv = rotY(90°)v
    const a = w * T;
    const jv = [dir[2], 0, -dir[0]];
    off = [
      dir[0] * (speed * Math.sin(a) / w) + jv[0] * (speed * (1 - Math.cos(a)) / w),
      0,
      dir[2] * (speed * Math.sin(a) / w) + jv[2] * (speed * (1 - Math.cos(a)) / w),
    ];
  } else {
    off = [dir[0] * speed * T, 0, dir[2] * speed * T];
  }
  // 最低前瞻长度，避免蠕行时构图塌陷
  const ol = Math.hypot(off[0], off[2]);
  if (ol < 3) off = [dir[0] * 3, 0, dir[2] * 3];
  // 相对载具航向的横向限幅
  const along = off[0] * f[0] + off[2] * f[2];
  let lat = [off[0] - f[0] * along, 0, off[2] - f[2] * along];
  const latL = Math.hypot(lat[0], lat[2]);
  if (latL > maxLateral) {
    const k = maxLateral / latL;
    off = [f[0] * along + lat[0] * k, 0, f[2] * along + lat[2] * k];
  }
  return [pos[0] + off[0], pos[1], pos[2] + off[2]];
}

// 对照用：朴素直线外推（速度*时间），用于测试证明弧线方案的必要性
export function predictLeadNaive(pos, vel, dist) {
  const speed = Math.hypot(vel[0], vel[2]);
  if (speed < 1e-6) return [pos[0], pos[1], pos[2]];
  return [pos[0] + (vel[0] / speed) * dist, pos[1], pos[2] + (vel[2] / speed) * dist];
}

export class FollowCamera {
  constructor(opts = {}) {
    this.fovYDeg = opts.fovYDeg ?? 60;              // 不依赖扩大视场掩盖前瞻失效
    this.aspect = opts.aspect ?? 16 / 9;
    // 死区与锚点：投影空间坐标，单位为“垂直半视场”，x 已乘长宽比修正
    this.deadZone = { x: opts.deadZone?.x ?? 0.22, y: opts.deadZone?.y ?? 0.16 };
    this.anchor = { x: opts.anchor?.x ?? 0, y: opts.anchor?.y ?? 0.1 };
    this.lookAheadDist = opts.lookAheadDist ?? 12;  // 前视距离（演示中可调）
    this.speedLeadTime = opts.speedLeadTime ?? 0.35; // 随速度附加的前视量（秒）
    this.maxLateral = opts.maxLateral ?? 10;
    this.followDist = opts.followDist ?? 7;
    this.followDistPerSpeed = opts.followDistPerSpeed ?? 0.1;
    this.followHeight = opts.followHeight ?? 3.0;
    this.posStiffness = opts.posStiffness ?? 5;      // 1/s
    this.yawStiffness = opts.yawStiffness ?? 4.5;    // 1/s
    this.pitchStiffness = opts.pitchStiffness ?? 4.5;
    this.maxYawRate = (opts.maxYawRateDeg ?? 150) * DEG;
    this.maxPitchRate = (opts.maxPitchRateDeg ?? 90) * DEG;
    this.pitchMin = (opts.pitchMinDeg ?? -32) * DEG;
    this.pitchMax = (opts.pitchMaxDeg ?? 22) * DEG;
    // 倒行跟随：沿速度方向的前瞻点会落到相机背后，必须改用“主体位置”作为构图目标，
    // 否则相机会原地掉头 180°。用纵向速度双阈值滞回，停车时保持当前模式。
    this.reverseEnterLong = opts.reverseEnterLong ?? -0.25; // m/s，低于此值进入倒行
    this.reverseExitLong = opts.reverseExitLong ?? 0.5;     // m/s，高于此值退出
    this.subjectAimHeight = opts.subjectAimHeight ?? 1.0;  // 主体瞄准点离地高度
    this.reverseMode = false;
    this.initialized = false;
    this.pos = [0, 0, 0];
    this.yaw = 0;
    this.pitch = 0;
  }

  get fovYRad() { return this.fovYDeg * DEG; }

  forward() {
    const cp = Math.cos(this.pitch);
    return [cp * Math.sin(this.yaw), Math.sin(this.pitch), cp * Math.cos(this.yaw)];
  }

  // 每步更新。state: { position, velocity, yawRate, forward }，dt 单位秒。
  update(dt, state) {
    const fwdFlat = norm([state.forward[0], 0, state.forward[2]]);
    const vLong = state.velocity[0] * fwdFlat[0] + state.velocity[2] * fwdFlat[2];
    // 倒行滞回：只有明确倒车才切换，停车/换挡抖动不改变构图目标
    if (this.reverseMode) {
      if (vLong > this.reverseExitLong) this.reverseMode = false;
    } else if (vLong < this.reverseEnterLong) {
      this.reverseMode = true;
    }

    const speed = Math.hypot(state.velocity[0], state.velocity[2]);
    const distEff = this.lookAheadDist + speed * this.speedLeadTime;
    const lead = predictLeadArc(
      state.position, state.velocity, state.yawRate ?? 0,
      distEff, this.maxLateral, state.forward,
    );
    // 倒行时速度方向的前瞻点会落到相机背后（相机始终在车头反方向），
    // 继续追前瞻点会让镜头原地掉头 180°；改为瞄准主体位置。
    const subject = [
      state.position[0],
      state.position[1] + this.subjectAimHeight,
      state.position[2],
    ];
    const target = this.reverseMode ? subject : lead;

    // 相机位置：弹性跟随锚点（非刚性固定距离）
    const followDist = this.followDist + speed * this.followDistPerSpeed;
    const anchorPos = [
      state.position[0] - state.forward[0] * followDist,
      state.position[1] + this.followHeight,
      state.position[2] - state.forward[2] * followDist,
    ];
    if (!this.initialized) {
      this.pos = [...anchorPos];
      const d = norm(sub(target, this.pos));
      this.yaw = Math.atan2(d[0], d[2]);
      this.pitch = clamp(Math.asin(clamp(d[1], -1, 1)), this.pitchMin, this.pitchMax);
      this.initialized = true;
      dt = 0;
    }
    if (dt > 0) {
      const kp = 1 - Math.exp(-this.posStiffness * dt);
      this.pos[0] += (anchorPos[0] - this.pos[0]) * kp;
      this.pos[1] += (anchorPos[1] - this.pos[1]) * kp;
      this.pos[2] += (anchorPos[2] - this.pos[2]) * kp;
    }

    let f = this.forward();
    let r = norm([-f[2], 0, f[0]]);
    let u = cross(r, f);

    let dYaw = 0, dPitch = 0;
    if (dt > 0) {
      const rel = sub(target, this.pos);
      const tanH = Math.tan(this.fovYRad / 2);
      let yawDes, pitchDes;
      const vz0 = dot(rel, f);
      if (vz0 > 0.25) {
        // 迭代求解期望姿态：在候选姿态下重算投影残差，只纠超出死区的部分，
        // 使目标精确回到死区边缘（一阶近似会有稳态残差，迭代消除）。
        let yawC = this.yaw, pitchC = this.pitch;
        for (let it = 0; it < 3; it++) {
          const cp = Math.cos(pitchC);
          const fc = [cp * Math.sin(yawC), Math.sin(pitchC), cp * Math.cos(yawC)];
          const rc = norm([-fc[2], 0, fc[0]]);
          const uc = cross(rc, fc);
          const vz = dot(rel, fc);
          if (vz <= 1e-3) break;
          const ax = dot(rel, rc) / vz / tanH;
          const ay = dot(rel, uc) / vz / tanH;
          const ex = ax - this.anchor.x;
          const ey = ay - this.anchor.y;
          const needX = ex - clamp(ex, -this.deadZone.x, this.deadZone.x);
          const needY = ey - clamp(ey, -this.deadZone.y, this.deadZone.y);
          if (needX === 0 && needY === 0) break;
          yawC -= Math.atan(needX * tanH);
          pitchC = clamp(pitchC + Math.atan(needY * tanH), this.pitchMin, this.pitchMax);
        }
        yawDes = yawC;
        pitchDes = clamp(pitchC, this.pitchMin, this.pitchMax);
      } else {
        // 目标在相机背后/近平面：直接朝它转（限幅仍生效）
        const dir = norm(rel);
        yawDes = Math.atan2(dir[0], dir[2]);
        pitchDes = clamp(Math.asin(clamp(dir[1], -1, 1)), this.pitchMin, this.pitchMax);
      }
      // 显式时间积分：精确阻尼 + 角速度限幅，任意 dt 稳定
      const ky = 1 - Math.exp(-this.yawStiffness * dt);
      const kp2 = 1 - Math.exp(-this.pitchStiffness * dt);
      dYaw = clamp(wrapPi(yawDes - this.yaw) * ky, -this.maxYawRate * dt, this.maxYawRate * dt);
      dPitch = clamp((pitchDes - this.pitch) * kp2, -this.maxPitchRate * dt, this.maxPitchRate * dt);
      this.yaw = wrapPi(this.yaw + dYaw);
      this.pitch = clamp(this.pitch + dPitch, this.pitchMin, this.pitchMax);
      f = this.forward();
      r = norm([-f[2], 0, f[0]]);
      u = cross(r, f);
    }

    const pr = this.projectPoint(target);
    const sp = this.projectPoint(subject);
    // 原始前瞻点核验：倒行时它沿速度方向落到车后，正是不能用它构图的原因。
    // 保留逐帧证据（相机前向点乘），供测试与样例核验直接读取。
    const arcLeadBehind = dot(sub(lead, this.pos), f) <= 0;
    // 死区判定含亚像素级容差：稳态跟踪残差(~0.002 投影单位)视为贴在边缘
    const inDeadZone = !pr.behind &&
      Math.abs(pr.ax - this.anchor.x) <= this.deadZone.x + 0.01 &&
      Math.abs(pr.ay - this.anchor.y) <= this.deadZone.y + 0.01;
    return {
      position: [...this.pos],
      yaw: this.yaw,
      pitch: this.pitch,
      forward: f,
      right: r,
      up: u,
      leadPoint: lead,
      subjectPoint: subject,
      targetPoint: target,
      diag: {
        nx: pr.nx, ny: pr.ny, ax: pr.ax, ay: pr.ay, vz: pr.vz,
        inDeadZone, dYaw, dPitch, dt,
        roomAhead: 1 - pr.ny,
        roomSide: 1 - Math.abs(pr.nx),
        speed, vLong,
        reverseMode: this.reverseMode,
        targetKind: this.reverseMode ? 'subject' : 'lead',
        lookAhead: distEff,
        leadChord: Math.hypot(lead[0] - state.position[0], lead[2] - state.position[2]),
        leadBehindCamera: arcLeadBehind,
        targetDist: Math.hypot(target[0] - state.position[0], target[2] - state.position[2]),
        subject: {
          nx: sp.nx, ny: sp.ny, ax: sp.ax, ay: sp.ay, vz: sp.vz, behind: sp.behind,
        },
      },
    };
  }

  // 世界点 → 投影坐标。nx/ny 为标准 NDC；ax/ay 为长宽比修正后的归一化坐标（死区所在空间）。
  projectPoint(p) {
    const f = this.forward();
    const r = norm([-f[2], 0, f[0]]);
    const u = cross(r, f);
    const rel = sub(p, this.pos);
    const vz = dot(rel, f);
    if (vz <= 1e-3) return { nx: 0, ny: 0, ax: 0, ay: 0, vz, behind: true };
    const tanH = Math.tan(this.fovYRad / 2);
    const nx = dot(rel, r) / vz / (tanH * this.aspect);
    const ny = dot(rel, u) / vz / tanH;
    return { nx, ny, ax: nx * this.aspect, ay: ny, vz, behind: false };
  }
}
