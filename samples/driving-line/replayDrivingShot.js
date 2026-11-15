// 由 buildDrivingShot.js 自动生成：仅依赖入库的 shot.json + 仓库 src/，重放驾驶镜头。
// 用法: node samples/<样例目录>/replayDrivingShot.js
// 校验: 逐帧重算相机位姿并与原始记录比对，打印最大偏差与前瞻/主体核验值。
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { FollowCamera } from '../../src/followCamera.js';

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
