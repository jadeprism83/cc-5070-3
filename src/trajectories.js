// 测试/演示共用的轨迹场景与仿真驱动。演示界面的按钮与命令行、单元测试
// 走的都是这里的同一份实现，不允许用固定数据冒充。
export function vehicleState(v) {
  return {
    position: [...v.pos],
    velocity: [...v.vel],
    yawRate: v.yawRate,
    forward: v.forward(),
  };
}

export const SCENARIOS = {
  accel: {
    key: 'accel',
    name: '加速',
    duration: 9,
    controls: (t) => ({ throttle: t < 6.5 ? 1 : 0.8, steer: 0 }),
  },
  curve: {
    key: 'curve',
    name: '连续弯道',
    duration: 16,
    controls: (t) => ({
      throttle: t < 2.5 ? 0.8 : 0.5,
      steer:
        t < 3 ? 0 :
        t < 9 ? 0.85 * Math.sin(((t - 3) / 6) * 2 * Math.PI) :
        t < 10 ? 0.85 * (t - 9) :
        0.85,
    }),
  },
  sideslip: {
    key: 'sideslip',
    name: '侧滑',
    duration: 8,
    controls: (t) => ({ throttle: t < 3 ? 1 : 0.62, steer: 0 }),
    events: [
      {
        t: 3.5,
        apply: (v) => { v.grip = 1.1; v.applyLateralImpulse(7.5); },
      },
      { t: 6.5, apply: (v) => { v.grip = 6; } },
    ],
  },
};

// 增量式场景执行器：演示端按真实帧 dt 逐步调用，测试端按固定 dt 批跑。
export function createRun(vehicle, camera, scenario) {
  let t = 0;
  const fired = new Set();
  const run = {
    rows: [],
    get t() { return t; },
    get done() { return t >= scenario.duration; },
    step(dt) {
      const events = scenario.events ?? [];
      for (let i = 0; i < events.length; i++) {
        if (!fired.has(i) && t >= events[i].t - 1e-9) {
          events[i].apply(vehicle);
          fired.add(i);
        }
      }
      vehicle.update(dt, scenario.controls(t));
      const out = camera.update(dt, vehicleState(vehicle));
      const row = {
        t, dt,
        veh: {
          pos: [...vehicle.pos], vel: [...vehicle.vel],
          yaw: vehicle.yaw, yawRate: vehicle.yawRate, grip: vehicle.grip,
        },
        cam: { pos: [...out.position], yaw: out.yaw, pitch: out.pitch },
        lead: [...out.leadPoint],
        diag: { ...out.diag },
      };
      run.rows.push(row);
      t += dt;
      return row;
    },
  };
  return run;
}

export function runScenario(vehicle, camera, scenario, dt) {
  const run = createRun(vehicle, camera, scenario);
  while (!run.done) run.step(dt);
  return run.rows;
}
