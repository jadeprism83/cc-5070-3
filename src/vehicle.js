// 简化的载具运动模型（演示/测试共用）：纵向油门刹车 + 速度相关转向 + 横向抓地衰减（可侧滑）。
import { clamp } from './math3d.js';

export class Vehicle {
  constructor(opts = {}) {
    this.accel = opts.accel ?? 9;          // m/s^2
    this.brake = opts.brake ?? 14;
    this.drag = opts.drag ?? 0.28;         // 线性阻力 1/s
    this.maxSpeed = opts.maxSpeed ?? 36;
    this.maxReverse = opts.maxReverse ?? 6;
    this.maxYawRate = opts.maxYawRate ?? 1.7;  // rad/s（低速）
    this.speedRef = opts.speedRef ?? 18;   // 转向随速度衰减的参考速度
    this.grip = opts.grip ?? 6;            // 横向速度衰减率 1/s，越小越滑
    this.pos = [0, 0, 0];
    this.yaw = 0;
    this.vel = [0, 0, 0];
    this.yawRate = 0;
  }

  forward() { return [Math.sin(this.yaw), 0, Math.cos(this.yaw)]; }

  // controls: { throttle ∈ [-1,1], steer ∈ [-1,1]（正=左转） }
  update(dt, controls) {
    const fwd = this.forward();
    let vLong = this.vel[0] * fwd[0] + this.vel[2] * fwd[2];
    const t = clamp(controls.throttle ?? 0, -1, 1);
    const a = (t >= 0 ? t * this.accel : t * this.brake) - this.drag * vLong;
    vLong = clamp(vLong + a * dt, -this.maxReverse, this.maxSpeed);

    const speedFactor = 1 / (1 + (vLong / this.speedRef) ** 2);
    const motionSign = clamp(vLong / 2, -1, 1); // 静止时转向不打横摆
    this.yawRate = clamp(controls.steer ?? 0, -1, 1) * this.maxYawRate * speedFactor * motionSign;
    this.yaw += this.yawRate * dt;

    const fwd2 = this.forward();
    const vL2 = this.vel[0] * fwd2[0] + this.vel[2] * fwd2[2];
    // 横向分量按抓地力指数衰减；抓地低时侧滑可持续
    let lat = [this.vel[0] - fwd2[0] * vL2, 0, this.vel[2] - fwd2[2] * vL2];
    const decay = Math.exp(-this.grip * dt);
    lat = [lat[0] * decay, 0, lat[2] * decay];
    this.vel = [fwd2[0] * vLong + lat[0], 0, fwd2[2] * vLong + lat[2]];
    this.pos = [
      this.pos[0] + this.vel[0] * dt,
      this.pos[1],
      this.pos[2] + this.vel[2] * dt,
    ];
  }

  applyLateralImpulse(mag) {
    const fwd = this.forward();
    const left = [fwd[2], 0, -fwd[0]];
    this.vel = [this.vel[0] + left[0] * mag, 0, this.vel[2] + left[2] * mag];
  }
}
