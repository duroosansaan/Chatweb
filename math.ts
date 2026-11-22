import { RigidBody, Vec2 } from './types';

export function vAdd(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function vSub(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function vScale(a: Vec2, s: number): Vec2 {
  return { x: a.x * s, y: a.y * s };
}

export function vDot(a: Vec2, b: Vec2): number {
  return a.x * b.x + a.y * b.y;
}

export function vCross(a: Vec2, b: Vec2): number {
  return a.x * b.y - a.y * b.x;
}

export function vCrossSV(s: number, v: Vec2): Vec2 {
  return { x: -s * v.y, y: s * v.x };
}

export function vLenSq(a: Vec2): number {
  return a.x * a.x + a.y * a.y;
}

export function vLen(a: Vec2): number {
  return Math.hypot(a.x, a.y);
}

export function vNorm(a: Vec2): Vec2 {
  const len = Math.hypot(a.x, a.y);
  if (len < 1e-9) return { x: 1, y: 0 };
  return { x: a.x / len, y: a.y / len };
}

export function vPerp(a: Vec2): Vec2 {
  return { x: -a.y, y: a.x };
}

export function vRotate(v: Vec2, angle: number): Vec2 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return { x: v.x * c - v.y * s, y: v.x * s + v.y * c };
}

export function localToWorld(body: RigidBody, local: Vec2): Vec2 {
  const r = vRotate(local, body.angle);
  return { x: body.x + r.x, y: body.y + r.y };
}

export function worldToLocal(body: RigidBody, world: Vec2): Vec2 {
  const dx = world.x - body.x;
  const dy = world.y - body.y;
  return vRotate({ x: dx, y: dy }, -body.angle);
}

export function getRectVertices(w: number, h: number): Vec2[] {
  const hw = w / 2;
  const hh = h / 2;
  return [
    { x: -hw, y: -hh },
    { x: hw, y: -hh },
    { x: hw, y: hh },
    { x: -hw, y: hh },
  ];
}

export function getWorldVertices(body: RigidBody): Vec2[] {
  const verts = body.shape === 'rect' ? getRectVertices(body.width, body.height) : body.vertices;
  const c = Math.cos(body.angle);
  const s = Math.sin(body.angle);
  return verts.map((v) => ({
    x: body.x + v.x * c - v.y * s,
    y: body.y + v.x * s + v.y * c,
  }));
}

/**
 * Ensures vertices are CCW, computes area, centroid, and shifts vertices so centroid = (0,0).
 */
export function computePolygonProperties(rawVerts: Vec2[], density: number): {
  vertices: Vec2[];
  centroid: Vec2;
  area: number;
  mass: number;
  inertia: number;
} {
  if (rawVerts.length < 3) {
    return {
      vertices: rawVerts,
      centroid: { x: 0, y: 0 },
      area: 0.1,
      mass: 0.1 * density,
      inertia: 0.01 * density,
    };
  }

  let signedArea = 0;
  let cx = 0;
  let cy = 0;
  const n = rawVerts.length;

  for (let i = 0; i < n; i++) {
    const p0 = rawVerts[i];
    const p1 = rawVerts[(i + 1) % n];
    const cross = p0.x * p1.y - p1.x * p0.y;
    signedArea += cross;
    cx += (p0.x + p1.x) * cross;
    cy += (p0.y + p1.y) * cross;
  }

  signedArea *= 0.5;
  let verts = [...rawVerts];
  if (signedArea < 0) {
    verts.reverse();
    signedArea = -signedArea;
    cx = -cx;
    cy = -cy;
  }

  const area = Math.max(signedArea, 1e-4);
  const centroid = {
    x: cx / (6 * area),
    y: cy / (6 * area),
  };

  const shifted = verts.map((v) => ({
    x: v.x - centroid.x,
    y: v.y - centroid.y,
  }));

  let inertiaNumer = 0;
  let inertiaDenom = 0;
  for (let i = 0; i < shifted.length; i++) {
    const p0 = shifted[i];
    const p1 = shifted[(i + 1) % shifted.length];
    const cross = Math.abs(vCross(p0, p1));
    inertiaNumer += cross * (vDot(p0, p0) + vDot(p0, p1) + vDot(p1, p1));
    inertiaDenom += cross;
  }

  const mass = area * density;
  const inertia = inertiaDenom > 1e-8 ? (mass / 6) * (inertiaNumer / inertiaDenom) : mass * 0.1;

  return {
    vertices: shifted,
    centroid,
    area,
    mass: Math.max(mass, 0.01),
    inertia: Math.max(inertia, 0.001),
  };
}

export function recalcBodyMassProperties(body: RigidBody): void {
  if (body.isStatic) {
    body.invMass = 0;
    body.invInertia = 0;
    return;
  }

  const m = Math.max(body.mass, 0.001);
  body.mass = m;
  body.invMass = 1 / m;

  if (body.shape === 'circle' || body.shape === 'gear' || body.shape === 'pulley') {
    body.inertia = 0.5 * m * body.radius * body.radius;
  } else if (body.shape === 'rect') {
    body.inertia = (m * (body.width * body.width + body.height * body.height)) / 12;
  } else if (body.shape === 'point') {
    body.inertia = 0.01 * m;
  } else {
    const props = computePolygonProperties(body.vertices, body.density || 1);
    const scale = props.mass > 0 ? m / props.mass : 1;
    body.inertia = Math.max(props.inertia * scale, 0.001);
  }
  body.invInertia = body.inertia > 1e-8 ? 1 / body.inertia : 0;
}

export function pointInBody(body: RigidBody, worldPt: Vec2, tolerance = 0): boolean {
  if (body.shape === 'circle' || body.shape === 'gear' || body.shape === 'pulley' || body.shape === 'point') {
    const r = (body.shape === 'point' ? 0.18 : body.radius) + tolerance;
    const dx = worldPt.x - body.x;
    const dy = worldPt.y - body.y;
    return dx * dx + dy * dy <= r * r;
  }

  const local = worldToLocal(body, worldPt);
  if (body.shape === 'rect') {
    const hw = body.width / 2 + tolerance;
    const hh = body.height / 2 + tolerance;
    return Math.abs(local.x) <= hw && Math.abs(local.y) <= hh;
  }

  // Polygon ray-casting
  const verts = body.vertices;
  let inside = false;
  for (let i = 0, j = verts.length - 1; i < verts.length; j = i++) {
    const xi = verts[i].x, yi = verts[i].y;
    const xj = verts[j].x, yj = verts[j].y;
    const intersect =
      yi > local.y !== yj > local.y &&
      local.x < ((xj - xi) * (local.y - yi)) / (yj - yi + 1e-12) + xi;
    if (intersect) inside = !inside;
  }
  if (inside) return true;
  if (tolerance > 0) {
    for (let i = 0; i < verts.length; i++) {
      const a = verts[i];
      const b = verts[(i + 1) % verts.length];
      if (distToSegment(local, a, b) <= tolerance) return true;
    }
  }
  return false;
}

export function distToSegment(p: Vec2, a: Vec2, b: Vec2): number {
  const ab = vSub(b, a);
  const lenSq = vLenSq(ab);
  if (lenSq < 1e-12) return vLen(vSub(p, a));
  const t = Math.max(0, Math.min(1, vDot(vSub(p, a), ab) / lenSq));
  const proj = { x: a.x + t * ab.x, y: a.y + t * ab.y };
  return vLen(vSub(p, proj));
}

export function clamp(v: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, v));
}
