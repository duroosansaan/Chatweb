import { SimulationState } from './types';

export function generateStandaloneHtml(state: SimulationState, title = 'Interactive Physics Simulation'): string {
  const safeJson = JSON.stringify(state).replace(/</g, '\\u003c');
  return `<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0" />
<title>${title.replace(/</g, '&lt;')}</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: system-ui, -apple-system, sans-serif; }
  body { background: #ece9d8; display: flex; flex-direction: column; height: 100vh; overflow: hidden; }
  header { background: #f3f4f6; border-bottom: 1px solid #9ca3af; padding: 6px 12px; display: flex; align-items: center; gap: 10px; font-size: 13px; }
  button { padding: 4px 12px; border: 1px solid #6b7280; background: #fff; cursor: pointer; border-radius: 3px; font-weight: 600; }
  button:hover { background: #e5e7eb; }
  #timeLabel { margin-left: auto; font-family: monospace; font-size: 13px; }
  canvas { flex: 1; width: 100%; height: 100%; background: #fff; display: block; touch-action: none; }
</style>
</head>
<body>
<header>
  <strong>${title.replace(/</g, '&lt;')}</strong>
  <button id="runBtn">Run</button>
  <button id="resetBtn">Reset</button>
  <span id="timeLabel">t = 0.00 s</span>
</header>
<canvas id="c"></canvas>
<script>
const INITIAL = ${safeJson};
let state = JSON.parse(JSON.stringify(INITIAL));
let running = false;
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
const runBtn = document.getElementById('runBtn');
const resetBtn = document.getElementById('resetBtn');
const timeLabel = document.getElementById('timeLabel');
const ppm = 50;

function resize() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * dpr;
  canvas.height = canvas.clientHeight * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resize);
resize();

function toScreen(wx, wy) {
  return { x: canvas.clientWidth / 2 + wx * ppm, y: canvas.clientHeight / 2 - wy * ppm };
}

runBtn.onclick = () => {
  running = !running;
  runBtn.textContent = running ? 'Stop' : 'Run';
};
resetBtn.onclick = () => {
  running = false;
  runBtn.textContent = 'Run';
  state = JSON.parse(JSON.stringify(INITIAL));
  timeLabel.textContent = 't = 0.00 s';
};

function step() {
  const dt = (state.world.dt || 1/60) / 4;
  const map = new Map(state.bodies.map(b => [b.id, b]));
  for (let s = 0; s < 4; s++) {
    for (const b of state.bodies) {
      if (b.isStatic) continue;
      b.fx = b.mass * (state.world.gx || 0);
      b.fy = b.mass * (state.world.gy ?? -9.81);
    }
    for (const c of state.constraints) {
      if (!c.enabled) continue;
      const bA = c.bodyAId ? map.get(c.bodyAId) : null;
      const bB = c.bodyBId ? map.get(c.bodyBId) : null;
      if (c.kind === 'spring' && bA && bB) {
        const dx = bB.x - bA.x, dy = bB.y - bA.y;
        const len = Math.hypot(dx, dy) || 1e-6;
        const f = (c.stiffness || 50) * (len - (c.restLength || 1.5));
        const fx = (f * dx) / len, fy = (f * dy) / len;
        if (!bA.isStatic) { bA.fx += fx; bA.fy += fy; }
        if (!bB.isStatic) { bB.fx -= fx; bB.fy -= fy; }
      }
    }
    for (const b of state.bodies) {
      if (b.isStatic) continue;
      b.vx += (b.fx / b.mass) * dt;
      b.vy += (b.fy / b.mass) * dt;
      b.x += b.vx * dt;
      b.y += b.vy * dt;
      b.angle += b.omega * dt;
    }
  }
  state.time += (state.world.dt || 1/60);
  timeLabel.textContent = 't = ' + state.time.toFixed(2) + ' s';
}

function draw() {
  if (running) step();
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  for (const b of state.bodies) {
    const p = toScreen(b.x, b.y);
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-b.angle);
    ctx.fillStyle = b.fillColor || '#93c5fd';
    ctx.strokeStyle = b.strokeColor || '#1e293b';
    ctx.lineWidth = 1.5;
    if (b.shape === 'circle' || b.shape === 'gear' || b.shape === 'pulley') {
      ctx.beginPath();
      ctx.arc(0, 0, b.radius * ppm, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    } else {
      ctx.fillRect(-b.width * ppm / 2, -b.height * ppm / 2, b.width * ppm, b.height * ppm);
      ctx.strokeRect(-b.width * ppm / 2, -b.height * ppm / 2, b.width * ppm, b.height * ppm);
    }
    ctx.restore();
  }
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);
</script>
</body>
</html>`;
}
