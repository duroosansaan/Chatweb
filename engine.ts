import { ContactManifold, detectCollision } from './collision';
import {
  clamp,
  localToWorld,
  recalcBodyMassProperties,
  vAdd,
  vCross,
  vCrossSV,
  vDot,
  vLen,
  vNorm,
  vPerp,
  vRotate,
  vScale,
  vSub,
} from './math';
import {
  MeterWidget,
  PhysicsConstraint,
  RigidBody,
  ScriptRule,
  SimulationState,
  TapeFrame,
  Vec2,
  WorldSettings,
} from './types';

export interface ActiveDragSpring {
  bodyId: string;
  localPoint: Vec2;
  targetWorld: Vec2;
}

export function createDefaultWorldSettings(): WorldSettings {
  return {
    gravityMode: 'vertical',
    gx: 0,
    gy: -9.81,
    gravityG: 1.0,
    airResistance: 0.01,
    electrostaticsEnabled: false,
    electrostaticK: 100,
    dt: 1 / 60,
    substeps: 6,
    solverIterations: 10,
    autoStopTime: null,
    showGrid: true,
    gridSpacing: 1,
    snapToGrid: false,
    showRulers: true,
    showAxes: true,
    showCOM: true,
    showCoordinates: true,
    showTrackPoints: true,
    showVectors: {
      velocity: false,
      acceleration: false,
      force: false,
      gravity: false,
      contact: false,
    },
    vectorScale: 0.25,
    unitSystem: 'SI',
    precision: 2,
  };
}

/**
 * Safe mathematical expression evaluator (no eval/Function).
 * Supports + - * / ^, parentheses, constants pi, e, and functions sin, cos, tan, abs, sqrt, min, max, sign, pow.
 */
export function evaluateMathExpression(expr: string, vars: Record<string, number>): number | null {
  try {
    const tokens: string[] = [];
    let i = 0;
    const s = expr.trim();
    while (i < s.length) {
      const ch = s[i];
      if (/\s/.test(ch)) {
        i++;
        continue;
      }
      if (/[0-9.]/.test(ch)) {
        let num = '';
        while (i < s.length && /[0-9.eE]/.test(s[i])) {
          if ((s[i] === 'e' || s[i] === 'E') && (s[i + 1] === '+' || s[i + 1] === '-')) {
            num += s[i] + s[i + 1];
            i += 2;
          } else {
            num += s[i];
            i++;
          }
        }
        tokens.push(num);
        continue;
      }
      if (/[a-zA-Z_]/.test(ch)) {
        let id = '';
        while (i < s.length && /[a-zA-Z0-9_]/.test(s[i])) {
          id += s[i++];
        }
        tokens.push(id);
        continue;
      }
      if ('+-*/^(),'.includes(ch)) {
        tokens.push(ch);
        i++;
        continue;
      }
      return null;
    }

    let pos = 0;
    const peek = () => tokens[pos];
    const consume = () => tokens[pos++];

    function parseExpr(): number {
      let left = parseTerm();
      while (peek() === '+' || peek() === '-') {
        const op = consume();
        const right = parseTerm();
        left = op === '+' ? left + right : left - right;
      }
      return left;
    }

    function parseTerm(): number {
      let left = parsePower();
      while (peek() === '*' || peek() === '/') {
        const op = consume();
        const right = parsePower();
        left = op === '*' ? left * right : right !== 0 ? left / right : 0;
      }
      return left;
    }

    function parsePower(): number {
      let base = parseUnary();
      if (peek() === '^') {
        consume();
        const exp = parsePower();
        base = Math.pow(base, exp);
      }
      return base;
    }

    function parseUnary(): number {
      if (peek() === '-') {
        consume();
        return -parseUnary();
      }
      if (peek() === '+') {
        consume();
        return parseUnary();
      }
      return parsePrimary();
    }

    function parsePrimary(): number {
      const tok = peek();
      if (tok === undefined) return 0;
      if (tok === '(') {
        consume();
        const val = parseExpr();
        if (peek() === ')') consume();
        return val;
      }
      if (/^[0-9.]/.test(tok)) {
        consume();
        return parseFloat(tok);
      }
      if (/^[a-zA-Z_]/.test(tok)) {
        consume();
        if (peek() === '(') {
          consume();
          const args: number[] = [];
          if (peek() !== ')') {
            args.push(parseExpr());
            while (peek() === ',') {
              consume();
              args.push(parseExpr());
            }
          }
          if (peek() === ')') consume();
          const fn = tok.toLowerCase();
          const a = args[0] ?? 0;
          const b = args[1] ?? 0;
          switch (fn) {
            case 'sin': return Math.sin(a);
            case 'cos': return Math.cos(a);
            case 'tan': return Math.tan(a);
            case 'asin': return Math.asin(clamp(a, -1, 1));
            case 'acos': return Math.acos(clamp(a, -1, 1));
            case 'atan': return Math.atan(a);
            case 'abs': return Math.abs(a);
            case 'sqrt': return Math.sqrt(Math.max(0, a));
            case 'sign': return Math.sign(a);
            case 'min': return Math.min(a, b);
            case 'max': return Math.max(a, b);
            case 'pow': return Math.pow(a, b);
            case 'exp': return Math.exp(clamp(a, -20, 20));
            default: return 0;
          }
        }
        if (tok === 'pi' || tok === 'PI') return Math.PI;
        if (tok === 'e') return Math.E;
        return vars[tok] ?? 0;
      }
      consume();
      return 0;
    }

    const result = parseExpr();
    return Number.isFinite(result) ? result : null;
  } catch {
    return null;
  }
}

export function stepSimulation(
  state: SimulationState,
  scripts: ScriptRule[],
  dragSpring: ActiveDragSpring | null
): ContactManifold[] {
  const { bodies, constraints, world } = state;
  const substeps = Math.max(1, world.substeps);
  const fullDt = world.dt;
  const dt = fullDt / substeps;
  const invFullDt = 1 / fullDt;

  const bodyMap = new Map<string, RigidBody>();
  for (const b of bodies) {
    recalcBodyMassProperties(b);
    bodyMap.set(b.id, b);
  }

  // Record pre-step velocities for acceleration measurement
  const preVx = new Map<string, number>();
  const preVy = new Map<string, number>();
  const preOmega = new Map<string, number>();
  for (const b of bodies) {
    preVx.set(b.id, b.vx);
    preVy.set(b.id, b.vy);
    preOmega.set(b.id, b.omega);
    b.contactFx = 0;
    b.contactFy = 0;
  }

  // Apply active script rules before stepping
  for (const rule of scripts) {
    if (!rule.enabled) continue;
    if (rule.targetType === 'body' && rule.targetId) {
      const b = bodyMap.get(rule.targetId);
      if (b) {
        const val = evaluateMathExpression(rule.expression, {
          t: state.time,
          x: b.x,
          y: b.y,
          vx: b.vx,
          vy: b.vy,
          angle: b.angle,
          omega: b.omega,
          mass: b.mass,
        });
        if (val !== null) {
          if (rule.property === 'vx') b.vx = val;
          else if (rule.property === 'vy') b.vy = val;
          else if (rule.property === 'omega') b.omega = val;
          else if (rule.property === 'x') b.x = val;
          else if (rule.property === 'y') b.y = val;
          else if (rule.property === 'angle') b.angle = val;
        }
      }
    } else if (rule.targetType === 'constraint' && rule.targetId) {
      const c = constraints.find((item) => item.id === rule.targetId);
      if (c) {
        const val = evaluateMathExpression(rule.expression, { t: state.time });
        if (val !== null) {
          if (rule.property === 'motorTargetOmega') c.motorTargetOmega = val;
          else if (rule.property === 'forceX') c.forceX = val;
          else if (rule.property === 'forceY') c.forceY = val;
          else if (rule.property === 'stiffness') c.stiffness = Math.max(0, val);
          else if (rule.property === 'restLength') c.restLength = Math.max(0.05, val);
        }
      }
    }
  }

  let lastManifolds: ContactManifold[] = [];

  for (let sub = 0; sub < substeps; sub++) {
    const subTime = state.time + sub * dt;

    // 1. Reset force accumulators
    for (const b of bodies) {
      b.fx = 0;
      b.fy = 0;
      b.torque = 0;
    }

    // 2. External Forces: Gravity, Air Resistance, Electrostatics, Thrusters
    for (let i = 0; i < bodies.length; i++) {
      const b = bodies[i];
      if (b.isStatic) continue;

      if (world.gravityMode === 'vertical') {
        b.fx += b.mass * world.gx;
        b.fy += b.mass * world.gy;
      }

      if (world.airResistance > 0) {
        const speed = Math.hypot(b.vx, b.vy);
        const drag = world.airResistance * (1 + 0.08 * speed);
        b.fx -= drag * b.vx;
        b.fy -= drag * b.vy;
        b.torque -= world.airResistance * 0.25 * b.omega;
      }

      if (b.thruster && b.thruster.enabled) {
        const worldForce = vRotate({ x: b.thruster.fx, y: b.thruster.fy }, b.angle);
        const r = vRotate({ x: b.thruster.localX, y: b.thruster.localY }, b.angle);
        b.fx += worldForce.x;
        b.fy += worldForce.y;
        b.torque += vCross(r, worldForce);
      }

      // Pairwise Planetary Gravity & Electrostatics
      if (world.gravityMode === 'planetary' || world.electrostaticsEnabled) {
        for (let j = i + 1; j < bodies.length; j++) {
          const other = bodies[j];
          const diff = { x: other.x - b.x, y: other.y - b.y };
          const distSq = Math.max(diff.x * diff.x + diff.y * diff.y, 0.04);
          const dist = Math.sqrt(distSq);
          const dir = { x: diff.x / dist, y: diff.y / dist };

          if (world.gravityMode === 'planetary') {
            const fMag = (world.gravityG * b.mass * other.mass) / distSq;
            const fx = dir.x * fMag;
            const fy = dir.y * fMag;
            b.fx += fx;
            b.fy += fy;
            if (!other.isStatic) {
              other.fx -= fx;
              other.fy -= fy;
            }
          }

          if (world.electrostaticsEnabled && b.charge !== 0 && other.charge !== 0) {
            // Like charges repel
            const fMag = (world.electrostaticK * b.charge * other.charge) / distSq;
            const fx = -dir.x * fMag;
            const fy = -dir.y * fMag;
            b.fx += fx;
            b.fy += fy;
            if (!other.isStatic) {
              other.fx -= fx;
              other.fy -= fy;
            }
          }
        }
      }
    }

    // 3. Force-based Constraints (Springs, Dampers, Applied Forces, Torques)
    for (const c of constraints) {
      if (!c.enabled) continue;
      const bA = c.bodyAId ? bodyMap.get(c.bodyAId) ?? null : null;
      const bB = c.bodyBId ? bodyMap.get(c.bodyBId) ?? null : null;

      if (c.kind === 'force' && bA && !bA.isStatic) {
        const pA = localToWorld(bA, c.localA);
        const rA = vSub(pA, { x: bA.x, y: bA.y });
        const rawF = { x: c.forceX ?? 0, y: c.forceY ?? 0 };
        const f = c.forceFollowBody ? vRotate(rawF, bA.angle) : rawF;
        bA.fx += f.x;
        bA.fy += f.y;
        bA.torque += vCross(rA, f);
        continue;
      }

      if (c.kind === 'torque' && bA && !bA.isStatic) {
        bA.torque += c.torqueValue ?? 0;
        continue;
      }

      if (c.kind === 'spring' || c.kind === 'damper') {
        const pA = bA ? localToWorld(bA, c.localA) : c.localA;
        const pB = bB ? localToWorld(bB, c.localB) : c.localB;
        const delta = vSub(pB, pA);
        const len = vLen(delta);
        const dir = len > 1e-6 ? vScale(delta, 1 / len) : { x: 0, y: 1 };

        const rA = bA ? vSub(pA, { x: bA.x, y: bA.y }) : { x: 0, y: 0 };
        const rB = bB ? vSub(pB, { x: bB.x, y: bB.y }) : { x: 0, y: 0 };

        const vA = bA ? vAdd({ x: bA.vx, y: bA.vy }, vCrossSV(bA.omega, rA)) : { x: 0, y: 0 };
        const vB = bB ? vAdd({ x: bB.vx, y: bB.vy }, vCrossSV(bB.omega, rB)) : { x: 0, y: 0 };
        const relV = vSub(vB, vA);
        const vAlong = vDot(relV, dir);

        let fMag = 0; // force pulling A toward B
        if (c.kind === 'spring') {
          const k = c.stiffness ?? 50;
          const rest = c.restLength ?? 1.5;
          const damp = c.damping ?? 0.5;
          fMag = k * (len - rest) + damp * vAlong;
        } else {
          const damp = c.damping ?? 8;
          fMag = damp * vAlong;
        }

        const forceOnA = vScale(dir, fMag);
        if (bA && !bA.isStatic) {
          bA.fx += forceOnA.x;
          bA.fy += forceOnA.y;
          bA.torque += vCross(rA, forceOnA);
        }
        if (bB && !bB.isStatic) {
          bB.fx -= forceOnA.x;
          bB.fy -= forceOnA.y;
          bB.torque -= vCross(rB, forceOnA);
        }
      }
    }

    // 4. Interactive Pointer Drag Spring while Running
    if (dragSpring) {
      const b = bodyMap.get(dragSpring.bodyId);
      if (b && !b.isStatic) {
        const worldPt = localToWorld(b, dragSpring.localPoint);
        const r = vSub(worldPt, { x: b.x, y: b.y });
        const velAtPt = vAdd({ x: b.vx, y: b.vy }, vCrossSV(b.omega, r));
        const diff = vSub(dragSpring.targetWorld, worldPt);
        const kGrab = Math.max(80, b.mass * 120);
        const cGrab = Math.max(8, b.mass * 16);
        const f = {
          x: kGrab * diff.x - cGrab * velAtPt.x,
          y: kGrab * diff.y - cGrab * velAtPt.y,
        };
        b.fx += f.x;
        b.fy += f.y;
        b.torque += vCross(r, f);
      }
    }

    // 5. Integrate Velocities (Symplectic Euler)
    for (const b of bodies) {
      if (b.isStatic) {
        b.vx = 0;
        b.vy = 0;
        b.omega = 0;
        continue;
      }
      b.vx += b.fx * b.invMass * dt;
      b.vy += b.fy * b.invMass * dt;
      b.omega += b.torque * b.invInertia * dt;
    }

    // 6. Detect Collisions
    const manifolds: ContactManifold[] = [];
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        const m = detectCollision(bodies[i], bodies[j]);
        if (m) manifolds.push(m);
      }
    }
    lastManifolds = manifolds;

    // Precompute joint data & solve Sequential Impulses
    const iterations = Math.max(4, world.solverIterations);
    const invDt = 1 / dt;

    for (let iter = 0; iter < iterations; iter++) {
      // A. Solve Constraints (Joints, Ropes, Rods, Slots, Actuators, Motors, Gears)
      for (const c of constraints) {
        if (!c.enabled) continue;
        const bA = c.bodyAId ? bodyMap.get(c.bodyAId) ?? null : null;
        const bB = c.bodyBId ? bodyMap.get(c.bodyBId) ?? null : null;

        if (c.kind === 'pin' || c.kind === 'weld') {
          solvePinConstraint(bA, bB, c.localA, c.localB, invDt);
          if (c.kind === 'weld') {
            solveAngularLock(bA, bB, c.referenceAngle ?? 0, invDt);
          }
        } else if (c.kind === 'slot') {
          solveSlotConstraint(bA, bB, c.localA, c.localB, c.slotAngle ?? 0, Boolean(c.slotKeyed), c.referenceAngle ?? 0, invDt);
        } else if (c.kind === 'rod') {
          solveDistanceConstraint(bA, bB, c.localA, c.localB, c.restLength ?? 1, false, invDt);
        } else if (c.kind === 'rope') {
          solveRopeConstraint(bA, bB, c.localA, c.localB, c.maxLength ?? c.restLength ?? 2, c.pulleyBodyId ? bodyMap.get(c.pulleyBodyId) ?? null : null, invDt);
        } else if (c.kind === 'actuator') {
          const baseL = c.restLength ?? 1.5;
          let targetL = baseL;
          if (c.actuatorMode === 'velocity') {
            targetL = Math.max(0.2, baseL + (c.actuatorVelocity ?? 0.5) * subTime);
          } else {
            const amp = c.actuatorAmplitude ?? 0.5;
            const freq = c.actuatorFrequency ?? 3.0;
            targetL = Math.max(0.2, baseL + amp * Math.sin(freq * subTime));
          }
          solveDistanceConstraint(bA, bB, c.localA, c.localB, targetL, false, invDt);
        } else if (c.kind === 'motor') {
          solveMotorConstraint(bA, bB, c.motorTargetOmega ?? 2.0, c.motorMaxTorque ?? 50, dt);
        } else if (c.kind === 'gearMesh' && bA && bB) {
          solveGearMeshConstraint(bA, bB);
        }
      }

      // B. Solve Contact Manifolds
      for (const m of manifolds) {
        const a = m.bodyA;
        const b = m.bodyB;
        const e = Math.min(a.restitution, b.restitution);
        const muS = Math.sqrt(a.staticFriction * b.staticFriction);
        const muK = Math.sqrt(a.kineticFriction * b.kineticFriction);
        const normal = m.normal;
        const tangent = vPerp(normal);
        const contactCount = m.contacts.length;

        for (const cp of m.contacts) {
          const rA = vSub(cp.position, { x: a.x, y: a.y });
          const rB = vSub(cp.position, { x: b.x, y: b.y });

          const vA = vAdd({ x: a.vx, y: a.vy }, vCrossSV(a.omega, rA));
          const vB = vAdd({ x: b.vx, y: b.vy }, vCrossSV(b.omega, rB));
          const relV = vSub(vB, vA);

          const vn = vDot(relV, normal);
          const rACrossN = vCross(rA, normal);
          const rBCrossN = vCross(rB, normal);
          const kNormal =
            a.invMass +
            b.invMass +
            rACrossN * rACrossN * a.invInertia +
            rBCrossN * rBCrossN * b.invInertia;
          const effMassN = kNormal > 1e-9 ? 1 / kNormal : 0;

          // Baumgarte bias with slop so stacked resting boxes don't jitter
          const slop = 0.004;
          const baumgarte = 0.22;
          const penBias = baumgarte * invDt * Math.max(0, cp.penetration - slop);
          const bounce = vn < -0.6 ? e * vn : 0;

          let jn = (-(vn + bounce) + penBias) * effMassN;
          jn /= contactCount;
          const oldJn = cp.normalImpulse;
          cp.normalImpulse = Math.max(0, oldJn + jn);
          const dJn = cp.normalImpulse - oldJn;

          const impN = vScale(normal, dJn);
          applyImpulse(a, b, impN, rA, rB);

          if (iter === iterations - 1) {
            const fN = vScale(impN, invDt);
            a.contactFx -= fN.x;
            a.contactFy -= fN.y;
            b.contactFx += fN.x;
            b.contactFy += fN.y;
          }

          // Friction impulse
          const vA2 = vAdd({ x: a.vx, y: a.vy }, vCrossSV(a.omega, rA));
          const vB2 = vAdd({ x: b.vx, y: b.vy }, vCrossSV(b.omega, rB));
          const relV2 = vSub(vB2, vA2);
          const vt = vDot(relV2, tangent);

          const rACrossT = vCross(rA, tangent);
          const rBCrossT = vCross(rB, tangent);
          const kTangent =
            a.invMass +
            b.invMass +
            rACrossT * rACrossT * a.invInertia +
            rBCrossT * rBCrossT * b.invInertia;
          const effMassT = kTangent > 1e-9 ? 1 / kTangent : 0;

          const jt = (-vt * effMassT) / contactCount;
          const oldJt = cp.tangentImpulse;
          const maxStatic = muS * cp.normalImpulse;
          const maxKinetic = muK * cp.normalImpulse;
          let newJt = oldJt + jt;
          if (Math.abs(newJt) > maxStatic) {
            newJt = clamp(newJt, -maxKinetic, maxKinetic);
          }
          cp.tangentImpulse = newJt;
          const dJt = newJt - oldJt;

          const impT = vScale(tangent, dJt);
          applyImpulse(a, b, impT, rA, rB);

          if (iter === iterations - 1) {
            const fT = vScale(impT, invDt);
            a.contactFx -= fT.x;
            a.contactFy -= fT.y;
            b.contactFx += fT.x;
            b.contactFy += fT.y;
          }
        }
      }
    }

    // 7. Integrate Positions
    for (const b of bodies) {
      if (b.isStatic) continue;
      b.x += b.vx * dt;
      b.y += b.vy * dt;
      b.angle += b.omega * dt;
    }
  }

  // Advance clock & compute measured accelerations & traces
  state.time += fullDt;
  state.frame += 1;

  for (const b of bodies) {
    if (b.isStatic) {
      b.ax = 0;
      b.ay = 0;
      b.alpha = 0;
    } else {
      b.ax = (b.vx - (preVx.get(b.id) ?? b.vx)) * invFullDt;
      b.ay = (b.vy - (preVy.get(b.id) ?? b.vy)) * invFullDt;
      b.alpha = (b.omega - (preOmega.get(b.id) ?? b.omega)) * invFullDt;
    }

    if (b.trackCOM && state.frame % 2 === 0) {
      b.tracePoints.push({ x: b.x, y: b.y });
      if (b.tracePoints.length > 600) {
        b.tracePoints.shift();
      }
    }
  }

  return lastManifolds;
}

function applyImpulse(
  a: RigidBody | null,
  b: RigidBody | null,
  impulse: Vec2,
  rA: Vec2,
  rB: Vec2
) {
  if (a && !a.isStatic) {
    a.vx -= impulse.x * a.invMass;
    a.vy -= impulse.y * a.invMass;
    a.omega -= vCross(rA, impulse) * a.invInertia;
  }
  if (b && !b.isStatic) {
    b.vx += impulse.x * b.invMass;
    b.vy += impulse.y * b.invMass;
    b.omega += vCross(rB, impulse) * b.invInertia;
  }
}

function solvePinConstraint(
  bA: RigidBody | null,
  bB: RigidBody | null,
  localA: Vec2,
  localB: Vec2,
  invDt: number
) {
  const pA = bA ? localToWorld(bA, localA) : localA;
  const pB = bB ? localToWorld(bB, localB) : localB;
  const rA = bA ? vSub(pA, { x: bA.x, y: bA.y }) : { x: 0, y: 0 };
  const rB = bB ? vSub(pB, { x: bB.x, y: bB.y }) : { x: 0, y: 0 };

  const invMassA = bA ? bA.invMass : 0;
  const invMassB = bB ? bB.invMass : 0;
  const invIA = bA ? bA.invInertia : 0;
  const invIB = bB ? bB.invInertia : 0;
  if (invMassA + invMassB + invIA + invIB === 0) return;

  const vA = bA ? vAdd({ x: bA.vx, y: bA.vy }, vCrossSV(bA.omega, rA)) : { x: 0, y: 0 };
  const vB = bB ? vAdd({ x: bB.vx, y: bB.vy }, vCrossSV(bB.omega, rB)) : { x: 0, y: 0 };

  const err = vSub(pB, pA);
  const bias = vScale(err, 0.25 * invDt);
  const Cdot = vAdd(vSub(vB, vA), bias);

  // 2x2 effective mass matrix K
  const k11 = invMassA + invMassB + invIA * rA.y * rA.y + invIB * rB.y * rB.y;
  const k12 = -invIA * rA.x * rA.y - invIB * rB.x * rB.y;
  const k22 = invMassA + invMassB + invIA * rA.x * rA.x + invIB * rB.x * rB.x;

  const det = k11 * k22 - k12 * k12;
  if (Math.abs(det) < 1e-9) return;
  const invDet = 1 / det;

  const imp = {
    x: -(k22 * Cdot.x - k12 * Cdot.y) * invDet,
    y: -(-k12 * Cdot.x + k11 * Cdot.y) * invDet,
  };
  applyImpulse(bA, bB, imp, rA, rB);
}

function solveAngularLock(
  bA: RigidBody | null,
  bB: RigidBody | null,
  refAngle: number,
  invDt: number
) {
  const invIA = bA ? bA.invInertia : 0;
  const invIB = bB ? bB.invInertia : 0;
  const k = invIA + invIB;
  if (k < 1e-9) return;

  const angA = bA ? bA.angle : 0;
  const angB = bB ? bB.angle : 0;
  const omA = bA ? bA.omega : 0;
  const omB = bB ? bB.omega : 0;

  let err = angB - angA - refAngle;
  while (err > Math.PI) err -= 2 * Math.PI;
  while (err < -Math.PI) err += 2 * Math.PI;

  const Cdot = omB - omA + 0.25 * invDt * err;
  const imp = -Cdot / k;

  if (bA && !bA.isStatic) bA.omega -= imp * invIA;
  if (bB && !bB.isStatic) bB.omega += imp * invIB;
}

function solveSlotConstraint(
  bA: RigidBody | null,
  bB: RigidBody | null,
  localA: Vec2,
  localB: Vec2,
  slotAngle: number,
  keyed: boolean,
  refAngle: number,
  invDt: number
) {
  if (!bA || bA.isStatic) return;
  const pA = localToWorld(bA, localA);
  const pB = bB ? localToWorld(bB, localB) : localB;
  const worldAxisAngle = bB ? bB.angle + slotAngle : slotAngle;
  // Normal perpendicular to slot rail
  const perp = { x: -Math.sin(worldAxisAngle), y: Math.cos(worldAxisAngle) };

  const rA = vSub(pA, { x: bA.x, y: bA.y });
  const rB = bB ? vSub(pB, { x: bB.x, y: bB.y }) : { x: 0, y: 0 };

  const vA = vAdd({ x: bA.vx, y: bA.vy }, vCrossSV(bA.omega, rA));
  const vB = bB ? vAdd({ x: bB.vx, y: bB.vy }, vCrossSV(bB.omega, rB)) : { x: 0, y: 0 };

  const posErr = vDot(vSub(pA, pB), perp);
  const velErr = vDot(vSub(vA, vB), perp);

  const rACrossP = vCross(rA, perp);
  const rBCrossP = vCross(rB, perp);
  const invIA = bA.invInertia;
  const invIB = bB ? bB.invInertia : 0;
  const invMB = bB ? bB.invMass : 0;

  const k = bA.invMass + invMB + rACrossP * rACrossP * invIA + rBCrossP * rBCrossP * invIB;
  if (k < 1e-9) return;

  const j = -(velErr + 0.25 * invDt * posErr) / k;
  const imp = vScale(perp, j);

  bA.vx += imp.x * bA.invMass;
  bA.vy += imp.y * bA.invMass;
  bA.omega += vCross(rA, imp) * invIA;

  if (bB && !bB.isStatic) {
    bB.vx -= imp.x * invMB;
    bB.vy -= imp.y * invMB;
    bB.omega -= vCross(rB, imp) * invIB;
  }

  if (keyed) {
    solveAngularLock(bA, bB, refAngle, invDt);
  }
}

function solveDistanceConstraint(
  bA: RigidBody | null,
  bB: RigidBody | null,
  localA: Vec2,
  localB: Vec2,
  targetLength: number,
  tensionOnly: boolean,
  invDt: number
) {
  const pA = bA ? localToWorld(bA, localA) : localA;
  const pB = bB ? localToWorld(bB, localB) : localB;
  const delta = vSub(pB, pA);
  const dist = vLen(delta);
  if (tensionOnly && dist <= targetLength) return;

  const n = dist > 1e-6 ? vScale(delta, 1 / dist) : { x: 0, y: 1 };
  const rA = bA ? vSub(pA, { x: bA.x, y: bA.y }) : { x: 0, y: 0 };
  const rB = bB ? vSub(pB, { x: bB.x, y: bB.y }) : { x: 0, y: 0 };

  const vA = bA ? vAdd({ x: bA.vx, y: bA.vy }, vCrossSV(bA.omega, rA)) : { x: 0, y: 0 };
  const vB = bB ? vAdd({ x: bB.vx, y: bB.vy }, vCrossSV(bB.omega, rB)) : { x: 0, y: 0 };

  const invMA = bA ? bA.invMass : 0;
  const invMB = bB ? bB.invMass : 0;
  const invIA = bA ? bA.invInertia : 0;
  const invIB = bB ? bB.invInertia : 0;

  const rACrossN = vCross(rA, n);
  const rBCrossN = vCross(rB, n);
  const k = invMA + invMB + rACrossN * rACrossN * invIA + rBCrossN * rBCrossN * invIB;
  if (k < 1e-9) return;

  const C = dist - targetLength;
  const Cdot = vDot(vSub(vB, vA), n);
  let j = -(Cdot + 0.25 * invDt * C) / k;

  // For tension-only (rope), impulse can only pull A & B closer (j <= 0 along A->B)
  if (tensionOnly && j > 0) j = 0;

  const imp = vScale(n, j);
  applyImpulse(bA, bB, imp, rA, rB);
}

function solveRopeConstraint(
  bA: RigidBody | null,
  bB: RigidBody | null,
  localA: Vec2,
  localB: Vec2,
  maxLength: number,
  pulley: RigidBody | null,
  invDt: number
) {
  if (!pulley) {
    solveDistanceConstraint(bA, bB, localA, localB, maxLength, true, invDt);
    return;
  }

  // Pulley rope: total length |pA - pulleyCenter| + |pB - pulleyCenter| <= maxLength
  const pA = bA ? localToWorld(bA, localA) : localA;
  const pB = bB ? localToWorld(bB, localB) : localB;
  const cP = { x: pulley.x, y: pulley.y };

  const uA = vSub(cP, pA);
  const uB = vSub(cP, pB);
  const dA = vLen(uA);
  const dB = vLen(uB);
  const totalLen = dA + dB;
  if (totalLen <= maxLength) return;

  const nA = dA > 1e-6 ? vScale(uA, 1 / dA) : { x: 0, y: 1 }; // direction from A -> pulley
  const nB = dB > 1e-6 ? vScale(uB, 1 / dB) : { x: 0, y: 1 }; // direction from B -> pulley

  const rA = bA ? vSub(pA, { x: bA.x, y: bA.y }) : { x: 0, y: 0 };
  const rB = bB ? vSub(pB, { x: bB.x, y: bB.y }) : { x: 0, y: 0 };

  const vA = bA ? vAdd({ x: bA.vx, y: bA.vy }, vCrossSV(bA.omega, rA)) : { x: 0, y: 0 };
  const vB = bB ? vAdd({ x: bB.vx, y: bB.vy }, vCrossSV(bB.omega, rB)) : { x: 0, y: 0 };

  const invMA = bA ? bA.invMass : 0;
  const invMB = bB ? bB.invMass : 0;
  const invIA = bA ? bA.invInertia : 0;
  const invIB = bB ? bB.invInertia : 0;

  const rACross = vCross(rA, nA);
  const rBCross = vCross(rB, nB);
  const k = invMA + invMB + rACross * rACross * invIA + rBCross * rBCross * invIB;
  if (k < 1e-9) return;

  // Rate of change of (dA + dB): -dot(vA, nA) - dot(vB, nB)
  const C = totalLen - maxLength;
  const Cdot = -vDot(vA, nA) - vDot(vB, nB);
  const j = Math.max(0, -(Cdot + 0.25 * invDt * C) / k);

  // Pull A toward pulley (+nA) and B toward pulley (+nB)
  if (bA && !bA.isStatic) {
    const impA = vScale(nA, j);
    bA.vx += impA.x * invMA;
    bA.vy += impA.y * invMA;
    bA.omega += vCross(rA, impA) * invIA;
  }
  if (bB && !bB.isStatic) {
    const impB = vScale(nB, j);
    bB.vx += impB.x * invMB;
    bB.vy += impB.y * invMB;
    bB.omega += vCross(rB, impB) * invIB;
  }

  // Rotate pulley visually based on rope motion
  if (!pulley.isStatic) {
    pulley.omega = (vDot(vB, nB) - vDot(vA, nA)) / Math.max(0.1, pulley.radius);
  }
}

function solveMotorConstraint(
  bA: RigidBody | null,
  bB: RigidBody | null,
  targetOmega: number,
  maxTorque: number,
  dt: number
) {
  if (!bA || bA.isStatic) return;
  const invIA = bA.invInertia;
  const invIB = bB ? bB.invInertia : 0;
  const k = invIA + invIB;
  if (k < 1e-9) return;

  const omB = bB ? bB.omega : 0;
  const relOmega = bA.omega - omB;
  const maxImp = Math.abs(maxTorque) * dt;
  const j = clamp((targetOmega - relOmega) / k, -maxImp, maxImp);

  bA.omega += j * invIA;
  if (bB && !bB.isStatic) {
    bB.omega -= j * invIB;
  }
}

function solveGearMeshConstraint(gearA: RigidBody, gearB: RigidBody) {
  const invIA = gearA.invInertia;
  const invIB = gearB.invInertia;
  const rA = Math.max(0.1, gearA.radius);
  const rB = Math.max(0.1, gearB.radius);
  const k = invIA * rA * rA + invIB * rB * rB;
  if (k < 1e-9) return;

  // Meshed external gears enforce: rA * omegaA + rB * omegaB = 0
  const Cdot = rA * gearA.omega + rB * gearB.omega;
  const lambda = -Cdot / k;

  if (!gearA.isStatic) gearA.omega += lambda * rA * invIA;
  if (!gearB.isStatic) gearB.omega += lambda * rB * invIB;
}

export function sampleBodyMetric(
  state: SimulationState,
  meter: MeterWidget
): { v1: number; v2: number; v3: number; labels: [string, string, string]; unit: string } {
  const b = meter.bodyId ? state.bodies.find((item) => item.id === meter.bodyId) : state.bodies[0];

  if (meter.metric === 'time') {
    return { v1: state.time, v2: state.frame, v3: 0, labels: ['t', 'Frame', ''], unit: 's' };
  }
  if (!b) {
    return { v1: 0, v2: 0, v3: 0, labels: ['X', 'Y', '|V|'], unit: '' };
  }

  switch (meter.metric) {
    case 'position':
      return {
        v1: b.x,
        v2: b.y,
        v3: (b.angle * 180) / Math.PI,
        labels: ['X (m)', 'Y (m)', 'θ (°)'],
        unit: 'm / °',
      };
    case 'velocity':
      return {
        v1: b.vx,
        v2: b.vy,
        v3: Math.hypot(b.vx, b.vy),
        labels: ['Vx', 'Vy', '|V|'],
        unit: 'm/s',
      };
    case 'acceleration':
      return {
        v1: b.ax,
        v2: b.ay,
        v3: Math.hypot(b.ax, b.ay),
        labels: ['Ax', 'Ay', '|A|'],
        unit: 'm/s²',
      };
    case 'force': {
      const fx = b.isStatic ? 0 : b.mass * b.ax;
      const fy = b.isStatic ? 0 : b.mass * b.ay;
      return {
        v1: fx,
        v2: fy,
        v3: Math.hypot(fx, fy),
        labels: ['Fx', 'Fy', '|F|'],
        unit: 'N',
      };
    }
    case 'momentum': {
      const px = b.mass * b.vx;
      const py = b.mass * b.vy;
      return {
        v1: px,
        v2: py,
        v3: Math.hypot(px, py),
        labels: ['Px', 'Py', '|P|'],
        unit: 'kg·m/s',
      };
    }
    case 'energy': {
      const ke = 0.5 * b.mass * (b.vx * b.vx + b.vy * b.vy) + 0.5 * b.inertia * b.omega * b.omega;
      const g = Math.abs(state.world.gy);
      const pe = state.world.gravityMode === 'vertical' ? b.mass * g * b.y : 0;
      return {
        v1: ke,
        v2: pe,
        v3: ke + pe,
        labels: ['KE', 'PE', 'Total E'],
        unit: 'J',
      };
    }
  }
}

export function captureTapeFrame(state: SimulationState): TapeFrame {
  return {
    time: state.time,
    frame: state.frame,
    bodies: state.bodies.map((b) => ({
      id: b.id,
      x: b.x,
      y: b.y,
      angle: b.angle,
      vx: b.vx,
      vy: b.vy,
      omega: b.omega,
      ax: b.ax,
      ay: b.ay,
      alpha: b.alpha,
      fx: b.fx,
      fy: b.fy,
      torque: b.torque,
      contactFx: b.contactFx,
      contactFy: b.contactFy,
    })),
  };
}

export function restoreTapeFrame(state: SimulationState, frame: TapeFrame): void {
  state.time = frame.time;
  state.frame = frame.frame;
  const map = new Map(frame.bodies.map((b) => [b.id, b]));
  for (const b of state.bodies) {
    const snap = map.get(b.id);
    if (snap) {
      b.x = snap.x;
      b.y = snap.y;
      b.angle = snap.angle;
      b.vx = snap.vx;
      b.vy = snap.vy;
      b.omega = snap.omega;
      b.ax = snap.ax;
      b.ay = snap.ay;
      b.alpha = snap.alpha;
      b.fx = snap.fx;
      b.fy = snap.fy;
      b.torque = snap.torque;
      b.contactFx = snap.contactFx;
      b.contactFy = snap.contactFy;
    }
  }
}

export function cloneSimulationState(state: SimulationState): SimulationState {
  return JSON.parse(JSON.stringify(state)) as SimulationState;
}
