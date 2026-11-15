// 极简软件光栅化器：把跟随相机的真实视图渲染成 PNG（无头环境交付"实际视图"用）。
// 与演示端共用同一 FollowCamera 的位姿输出，投影数学一致。
import zlib from 'node:zlib';
import fs from 'node:fs';
import { xformP, mul4 } from './math3d.js';

// ---- PNG 编码 ----
const crcTable = (() => {
  const t = new Int32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    t[n] = c;
  }
  return t;
})();
function crc32(buf) {
  let c = -1;
  for (let i = 0; i < buf.length; i++) c = crcTable[(c ^ buf[i]) & 0xff] ^ (c >>> 8);
  return (c ^ -1) >>> 0;
}
function chunk(type, data) {
  const out = Buffer.alloc(8 + data.length + 4);
  out.writeUInt32BE(data.length, 0);
  out.write(type, 4);
  data.copy(out, 8);
  out.writeUInt32BE(crc32(out.subarray(4, 8 + data.length)), 8 + data.length);
  return out;
}
export function writePNG(path, w, h, rgb) {
  const stride = w * 3;
  const raw = Buffer.alloc((stride + 1) * h);
  for (let y = 0; y < h; y++) {
    raw[y * (stride + 1)] = 0;
    Buffer.from(rgb.buffer, rgb.byteOffset + y * stride, stride).copy(raw, y * (stride + 1) + 1);
  }
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(w, 0);
  ihdr.writeUInt32BE(h, 4);
  ihdr[8] = 8; ihdr[9] = 2; // 8bit RGB
  const png = Buffer.concat([
    Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]),
    chunk('IHDR', ihdr),
    chunk('IDAT', zlib.deflateSync(raw)),
    chunk('IEND', Buffer.alloc(0)),
  ]);
  fs.writeFileSync(path, png);
}

// ---- 位图与画线 ----
export function createImage(w, h, bg = [16, 19, 24]) {
  const data = new Uint8Array(w * h * 3);
  for (let i = 0; i < w * h; i++) {
    data[i * 3] = bg[0]; data[i * 3 + 1] = bg[1]; data[i * 3 + 2] = bg[2];
  }
  return { w, h, data };
}
export function setPx(img, x, y, c) {
  x |= 0; y |= 0;
  if (x < 0 || y < 0 || x >= img.w || y >= img.h) return;
  const i = (y * img.w + x) * 3;
  img.data[i] = c[0]; img.data[i + 1] = c[1]; img.data[i + 2] = c[2];
}
export function drawLine(img, x0, y0, x1, y1, c) {
  x0 |= 0; y0 |= 0; x1 |= 0; y1 |= 0;
  const dx = Math.abs(x1 - x0), dy = -Math.abs(y1 - y0);
  const sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  let err = dx + dy;
  for (;;) {
    setPx(img, x0, y0, c);
    if (x0 === x1 && y0 === y1) break;
    const e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

// ---- 三维投影 ----
export function makeProjector(view, proj, w, h) {
  const vp = mul4(proj, view);
  return (p) => {
    const [x, y, z, ww] = xformP(vp, p);
    if (ww < 0.1) return null; // 近平面/背后
    return [((x / ww) * 0.5 + 0.5) * w, (1 - (y / ww) * 0.5 - 0.5) * h];
  };
}
export function drawLine3(img, proj, a, b, c) {
  const pa = proj(a), pb = proj(b);
  if (!pa || !pb) return;
  drawLine(img, pa[0], pa[1], pb[0], pb[1], c);
}
export function drawBox3(img, proj, center, size, yaw, c) {
  const cs = Math.cos(yaw), sn = Math.sin(yaw);
  const [sx, sy, sz] = [size[0] / 2, size[1] / 2, size[2] / 2];
  const corner = (ix, iy, iz) => {
    const lx = ix * sx, ly = iy * sy, lz = iz * sz;
    return [
      center[0] + cs * lx + sn * lz,
      center[1] + ly,
      center[2] - sn * lx + cs * lz,
    ];
  };
  const V = [];
  for (const iz of [-1, 1]) for (const iy of [-1, 1]) for (const ix of [-1, 1]) V.push(corner(ix, iy, iz));
  const E = [
    [0, 1], [1, 3], [3, 2], [2, 0],
    [4, 5], [5, 7], [7, 6], [6, 4],
    [0, 4], [1, 5], [2, 6], [3, 7],
  ];
  for (const [a, b] of E) drawLine3(img, proj, V[a], V[b], c);
}
export function drawGrid(img, proj, half = 80, step = 5, c = [52, 58, 66]) {
  for (let i = -half; i <= half; i += step) {
    drawLine3(img, proj, [i, 0, -half], [i, 0, half], c);
    drawLine3(img, proj, [-half, 0, i], [half, 0, i], c);
  }
}
export function drawCross3(img, proj, p, r, c) {
  drawLine3(img, proj, [p[0] - r, p[1], p[2]], [p[0] + r, p[1], p[2]], c);
  drawLine3(img, proj, [p[0], p[1], p[2] - r], [p[0], p[1], p[2] + r], c);
  drawLine3(img, proj, [p[0], p[1] - r, p[2]], [p[0], p[1] + r, p[2]], c);
}
