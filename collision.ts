import {
  getWorldVertices,
  vAdd,
  vDot,
  vLen,
  vLenSq,
  vNorm,
  vPerp,
  vScale,
  vSub,
} from './math';
import { RigidBody, Vec2 } from './types';

export interface ContactPoint {
  position: Vec2;
  penetration: number;
  normalImpulse: number;
  tangentImpulse: number;
}

export interface ContactManifold {
  bodyA: RigidBody;
  bodyB: RigidBody;
  normal: Vec2; // points from A -> B
  contacts: ContactPoint[];
}

function isCircleLike(b: RigidBody): boolean {
  return b.shape === 'circle' || b.shape === 'gear' || b.shape === 'pulley';
}

function canCollide(a: RigidBody, b: RigidBody): boolean {
  if (!a.collidable || !b.collidable) return false;
  if (a.isStatic && b.isStatic) return false;
  if (a.shape === 'point' || b.shape === 'point') return false;
  if (a.collisionGroup !== 0 && b.collisionGroup !== 0 && a.collisionGroup !== b.collisionGroup) {
    return false;
  }
  return true;
}

export function detectCollision(a: RigidBody, b: RigidBody): ContactManifold | null {
  if (!canCollide(a, b)) return null;

  const aCirc = isCircleLike(a);
  const bCirc = isCircleLike(b);

  if (aCirc && bCirc) {
    return collideCircles(a, b);
  }
  if (aCirc && !bCirc) {
    return collideCirclePolygon(a, b);
  }
  if (!aCirc && bCirc) {
    const m = collideCirclePolygon(b, a);
    if (!m) return null;
    return {
      bodyA: a,
      bodyB: b,
      normal: { x: -m.normal.x, y: -m.normal.y },
      contacts: m.contacts,
    };
  }
  return collidePolygons(a, b);
}

function collideCircles(a: RigidBody, b: RigidBody): ContactManifold | null {
  const diff = { x: b.x - a.x, y: b.y - a.y };
  const rSum = a.radius + b.radius;
  const distSq = vLenSq(diff);
  if (distSq >= rSum * rSum) return null;

  const dist = Math.sqrt(distSq);
  const normal = dist > 1e-6 ? { x: diff.x / dist, y: diff.y / dist } : { x: 0, y: 1 };
  const penetration = rSum - dist;
  const contactPos = {
    x: a.x + normal.x * (a.radius - penetration * 0.5),
    y: a.y + normal.y * (a.radius - penetration * 0.5),
  };

  return {
    bodyA: a,
    bodyB: b,
    normal,
    contacts: [{ position: contactPos, penetration, normalImpulse: 0, tangentImpulse: 0 }],
  };
}

function collideCirclePolygon(circle: RigidBody, poly: RigidBody): ContactManifold | null {
  const verts = getWorldVertices(poly);
  if (verts.length < 3) return null;

  const cPos = { x: circle.x, y: circle.y };
  let minSep = -Infinity;
  let bestEdgeIdx = 0;

  for (let i = 0; i < verts.length; i++) {
    const v1 = verts[i];
    const v2 = verts[(i + 1) % verts.length];
    const edge = vSub(v2, v1);
    // Outward normal for CCW polygon: (edge.y, -edge.x)
    const normal = vNorm({ x: edge.y, y: -edge.x });
    const sep = vDot(normal, vSub(cPos, v1));
    if (sep > circle.radius) return null;
    if (sep > minSep) {
      minSep = sep;
      bestEdgeIdx = i;
    }
  }

  const v1 = verts[bestEdgeIdx];
  const v2 = verts[(bestEdgeIdx + 1) % verts.length];

  // If circle center is inside polygon
  if (minSep < 1e-6) {
    const edge = vSub(v2, v1);
    const outNormal = vNorm({ x: edge.y, y: -edge.x });
    // normal points from circle -> poly
    const normal = { x: -outNormal.x, y: -outNormal.y };
    const penetration = circle.radius - minSep;
    return {
      bodyA: circle,
      bodyB: poly,
      normal,
      contacts: [
        {
          position: vAdd(cPos, vScale(normal, circle.radius * 0.5)),
          penetration,
          normalImpulse: 0,
          tangentImpulse: 0,
        },
      ],
    };
  }

  // Closest point on best edge
  const edge = vSub(v2, v1);
  const edgeLenSq = vLenSq(edge);
  const t = edgeLenSq > 1e-9 ? Math.max(0, Math.min(1, vDot(vSub(cPos, v1), edge) / edgeLenSq)) : 0;
  const closest = { x: v1.x + t * edge.x, y: v1.y + t * edge.y };
  const toClosest = vSub(closest, cPos);
  const dist = vLen(toClosest);
  if (dist > circle.radius) return null;

  const normal = dist > 1e-6 ? vScale(toClosest, 1 / dist) : vNorm({ x: -edge.y, y: edge.x });
  const penetration = circle.radius - dist;

  return {
    bodyA: circle,
    bodyB: poly,
    normal,
    contacts: [
      {
        position: closest,
        penetration,
        normalImpulse: 0,
        tangentImpulse: 0,
      },
    ],
  };
}

function getEdgeNormals(verts: Vec2[]): Vec2[] {
  const normals: Vec2[] = [];
  for (let i = 0; i < verts.length; i++) {
    const p1 = verts[i];
    const p2 = verts[(i + 1) % verts.length];
    const edge = vSub(p2, p1);
    normals.push(vNorm({ x: edge.y, y: -edge.x }));
  }
  return normals;
}

function findMaxSeparation(vertsA: Vec2[], normalsA: Vec2[], vertsB: Vec2[]): {
  sep: number;
  edgeIndex: number;
} {
  let bestSep = -Infinity;
  let bestIndex = 0;

  for (let i = 0; i < vertsA.length; i++) {
    const n = normalsA[i];
    const v1 = vertsA[i];
    // Find support point on B along -n
    let minProj = Infinity;
    for (let j = 0; j < vertsB.length; j++) {
      const proj = vDot(n, vSub(vertsB[j], v1));
      if (proj < minProj) {
        minProj = proj;
      }
    }
    if (minProj > bestSep) {
      bestSep = minProj;
      bestIndex = i;
    }
  }
  return { sep: bestSep, edgeIndex: bestIndex };
}

function clipSegmentToLine(
  vIn: [Vec2, Vec2],
  normal: Vec2,
  offset: number
): Vec2[] {
  const vOut: Vec2[] = [];
  const d0 = vDot(normal, vIn[0]) - offset;
  const d1 = vDot(normal, vIn[1]) - offset;

  if (d0 >= 0) vOut.push(vIn[0]);
  if (d1 >= 0) vOut.push(vIn[1]);

  if (d0 * d1 < 0) {
    const interp = d0 / (d0 - d1);
    vOut.push({
      x: vIn[0].x + interp * (vIn[1].x - vIn[0].x),
      y: vIn[0].y + interp * (vIn[1].y - vIn[0].y),
    });
  }
  return vOut;
}

function findIncidentEdge(
  refNormal: Vec2,
  vertsInc: Vec2[],
  normalsInc: Vec2[]
): [Vec2, Vec2] {
  let minDot = Infinity;
  let incIdx = 0;
  for (let i = 0; i < normalsInc.length; i++) {
    const d = vDot(refNormal, normalsInc[i]);
    if (d < minDot) {
      minDot = d;
      incIdx = i;
    }
  }
  const p1 = vertsInc[incIdx];
  const p2 = vertsInc[(incIdx + 1) % vertsInc.length];
  return [p1, p2];
}

function collidePolygons(a: RigidBody, b: RigidBody): ContactManifold | null {
  const vertsA = getWorldVertices(a);
  const vertsB = getWorldVertices(b);
  if (vertsA.length < 3 || vertsB.length < 3) return null;

  const normalsA = getEdgeNormals(vertsA);
  const normalsB = getEdgeNormals(vertsB);

  const resA = findMaxSeparation(vertsA, normalsA, vertsB);
  if (resA.sep > 0) return null;

  const resB = findMaxSeparation(vertsB, normalsB, vertsA);
  if (resB.sep > 0) return null;

  let refVerts: Vec2[];
  let refNormals: Vec2[];
  let incVerts: Vec2[];
  let incNormals: Vec2[];
  let refIdx: number;
  let flip: boolean;

  // Bias slightly toward A to avoid flickering reference edges
  if (resB.sep > resA.sep + 0.002) {
    refVerts = vertsB;
    refNormals = normalsB;
    incVerts = vertsA;
    incNormals = normalsA;
    refIdx = resB.edgeIndex;
    flip = true;
  } else {
    refVerts = vertsA;
    refNormals = normalsA;
    incVerts = vertsB;
    incNormals = normalsB;
    refIdx = resA.edgeIndex;
    flip = false;
  }

  const refV1 = refVerts[refIdx];
  const refV2 = refVerts[(refIdx + 1) % refVerts.length];
  const refNormal = refNormals[refIdx];
  const tangent = vPerp(refNormal); // points along refV1 -> refV2 or opposite
  const edgeDir = vNorm(vSub(refV2, refV1));

  let incEdge = findIncidentEdge(refNormal, incVerts, incNormals);

  // Clip against side planes of reference edge
  const sideOffset1 = vDot(edgeDir, refV1);
  const clip1 = clipSegmentToLine(incEdge, edgeDir, sideOffset1);
  if (clip1.length < 2) return null;

  const negEdgeDir = { x: -edgeDir.x, y: -edgeDir.y };
  const sideOffset2 = vDot(negEdgeDir, refV2);
  const clip2 = clipSegmentToLine([clip1[0], clip1[1]], negEdgeDir, sideOffset2);
  if (clip2.length < 2) return null;

  // Keep points behind reference face
  const refOffset = vDot(refNormal, refV1);
  const contacts: ContactPoint[] = [];
  for (const pt of clip2) {
    const sep = vDot(refNormal, pt) - refOffset;
    if (sep <= 0.001) {
      contacts.push({
        position: pt,
        penetration: Math.max(0, -sep),
        normalImpulse: 0,
        tangentImpulse: 0,
      });
    }
  }

  if (contacts.length === 0) return null;

  // Normal must always point from A -> B
  const normal = flip ? { x: -refNormal.x, y: -refNormal.y } : refNormal;
  void tangent;

  return {
    bodyA: a,
    bodyB: b,
    normal,
    contacts,
  };
}
