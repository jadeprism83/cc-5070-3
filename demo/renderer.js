// 极简 WebGL 渲染器：盒子（带简单明暗）+ 线条。相机矩阵来自 FollowCamera 的真实位姿。
import { modelTRS } from '../src/math3d.js';

const VS = `
attribute vec3 aPos; attribute vec3 aNormal;
uniform mat4 uVP; uniform mat4 uModel;
varying vec3 vN;
void main(){ gl_Position = uVP * uModel * vec4(aPos,1.0); vN = aNormal; }`;
const FS = `
precision mediump float;
uniform vec3 uColor; uniform float uLit;
varying vec3 vN;
void main(){
  float l = uLit > 0.5 ? (0.55 + 0.45*max(dot(normalize(vN), normalize(vec3(0.4,0.8,0.3))), 0.0)) : 1.0;
  gl_FragColor = vec4(uColor*l, 1.0);
}`;

function compile(gl, type, src) {
  const s = gl.createShader(type);
  gl.shaderSource(s, src);
  gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
  return s;
}

// 单位立方体（中心原点，边长1）：36顶点 + 法线
function cubeData() {
  const faces = [
    [[1, 0, 0], [[1, -1, -1], [1, 1, -1], [1, 1, 1], [1, -1, 1]]],
    [[-1, 0, 0], [[-1, -1, 1], [-1, 1, 1], [-1, 1, -1], [-1, -1, -1]]],
    [[0, 1, 0], [[-1, 1, -1], [-1, 1, 1], [1, 1, 1], [1, 1, -1]]],
    [[0, -1, 0], [[-1, -1, 1], [-1, -1, -1], [1, -1, -1], [1, -1, 1]]],
    [[0, 0, 1], [[-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]]],
    [[0, 0, -1], [[1, -1, -1], [-1, -1, -1], [-1, 1, -1], [1, 1, -1]]],
  ];
  const pos = [], nor = [];
  for (const [n, q] of faces) {
    const idx = [0, 1, 2, 0, 2, 3];
    for (const i of idx) {
      pos.push(q[i][0] / 2, q[i][1] / 2, q[i][2] / 2);
      nor.push(...n);
    }
  }
  return { pos: new Float32Array(pos), nor: new Float32Array(nor) };
}

export function createRenderer(canvas) {
  const gl = canvas.getContext('webgl', { antialias: true });
  if (!gl) throw new Error('WebGL 不可用');
  const prog = gl.createProgram();
  gl.attachShader(prog, compile(gl, gl.VERTEX_SHADER, VS));
  gl.attachShader(prog, compile(gl, gl.FRAGMENT_SHADER, FS));
  gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
  gl.useProgram(prog);
  const loc = {
    aPos: gl.getAttribLocation(prog, 'aPos'),
    aNormal: gl.getAttribLocation(prog, 'aNormal'),
    uVP: gl.getUniformLocation(prog, 'uVP'),
    uModel: gl.getUniformLocation(prog, 'uModel'),
    uColor: gl.getUniformLocation(prog, 'uColor'),
    uLit: gl.getUniformLocation(prog, 'uLit'),
  };
  const cube = cubeData();
  const cubePos = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, cubePos);
  gl.bufferData(gl.ARRAY_BUFFER, cube.pos, gl.STATIC_DRAW);
  const cubeNor = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, cubeNor);
  gl.bufferData(gl.ARRAY_BUFFER, cube.nor, gl.STATIC_DRAW);
  const lineBuf = gl.createBuffer();

  gl.enable(gl.DEPTH_TEST);
  gl.clearColor(0.07, 0.08, 0.1, 1);

  const ident = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);

  return {
    gl,
    resize() {
      const dpr = Math.min(2, window.devicePixelRatio || 1);
      const w = Math.floor(canvas.clientWidth * dpr);
      const h = Math.floor(canvas.clientHeight * dpr);
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w; canvas.height = h;
      }
      gl.viewport(0, 0, w, h);
      return w / h;
    },
    begin(vp) {
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.uniformMatrix4fv(loc.uVP, false, new Float32Array(vp));
    },
    cube(x, y, z, yaw, sx, sy, sz, color, lit = true) {
      gl.uniformMatrix4fv(loc.uModel, false, new Float32Array(modelTRS(x, y, z, yaw, sx, sy, sz)));
      gl.uniform3fv(loc.uColor, color);
      gl.uniform1f(loc.uLit, lit ? 1 : 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, cubePos);
      gl.enableVertexAttribArray(loc.aPos);
      gl.vertexAttribPointer(loc.aPos, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, cubeNor);
      gl.enableVertexAttribArray(loc.aNormal);
      gl.vertexAttribPointer(loc.aNormal, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.TRIANGLES, 0, 36);
    },
    lines(flat, color) {
      if (flat.length < 6) return;
      gl.uniformMatrix4fv(loc.uModel, false, ident);
      gl.uniform3fv(loc.uColor, color);
      gl.uniform1f(loc.uLit, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, lineBuf);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(flat), gl.DYNAMIC_DRAW);
      gl.enableVertexAttribArray(loc.aPos);
      gl.vertexAttribPointer(loc.aPos, 3, gl.FLOAT, false, 0, 0);
      gl.disableVertexAttribArray(loc.aNormal);
      gl.vertexAttrib3f(loc.aNormal, 0, 1, 0);
      gl.drawArrays(gl.LINES, 0, flat.length / 3);
    },
  };
}
