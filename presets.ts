import { createDefaultWorldSettings } from './engine';
import { computePolygonProperties, getRectVertices, recalcBodyMassProperties } from './math';
import {
  ControlWidget,
  MeterWidget,
  PhysicsConstraint,
  RigidBody,
  SimulationState,
  Vec2,
} from './types';

let uidCounter = 1;
export function nextId(prefix = 'obj'): string {
  uidCounter += 1;
  return `${prefix}_${Date.now().toString(36)}_${uidCounter}`;
}

const PALETTE_COLORS = [
  '#93c5fd', // classic sky blue
  '#fca5a5', // coral red
  '#fde047', // warm yellow
  '#86efac', // mint green
  '#fdba74', // orange
  '#c4b5fd', // lavender
  '#cbd5e1', // steel slate
];

export function pickBodyColor(idx: number): string {
  return PALETTE_COLORS[idx % PALETTE_COLORS.length];
}

export function createBody(
  partial: Partial<RigidBody> & { shape: RigidBody['shape']; x: number; y: number }
): RigidBody {
  const width = partial.width ?? 1.4;
  const height = partial.height ?? 1.0;
  const radius = partial.radius ?? 0.55;
  const density = partial.density ?? 2.0;

  let vertices: Vec2[] = partial.vertices ?? [];
  let defaultMass = partial.mass;

  if (partial.shape === 'rect') {
    vertices = getRectVertices(width, height);
    if (defaultMass === undefined) defaultMass = width * height * density;
  } else if (partial.shape === 'circle' || partial.shape === 'gear' || partial.shape === 'pulley') {
    if (defaultMass === undefined) defaultMass = Math.PI * radius * radius * density;
  } else if (partial.shape === 'point') {
    if (defaultMass === undefined) defaultMass = 0.2;
  } else if (partial.shape === 'polygon') {
    const props = computePolygonProperties(vertices, density);
    vertices = props.vertices;
    if (defaultMass === undefined) defaultMass = props.mass;
  }

  const body: RigidBody = {
    id: partial.id ?? nextId('body'),
    name: partial.name ?? `Object ${uidCounter}`,
    shape: partial.shape,
    x: partial.x,
    y: partial.y,
    angle: partial.angle ?? 0,
    vx: partial.vx ?? 0,
    vy: partial.vy ?? 0,
    omega: partial.omega ?? 0,
    ax: 0,
    ay: 0,
    alpha: 0,
    fx: 0,
    fy: 0,
    torque: 0,
    contactFx: 0,
    contactFy: 0,
    width,
    height,
    radius,
    vertices,
    teeth: partial.teeth ?? 12,
    mass: defaultMass ?? 2.0,
    invMass: 0,
    density,
    inertia: 1,
    invInertia: 0,
    restitution: partial.restitution ?? 0.5,
    staticFriction: partial.staticFriction ?? 0.4,
    kineticFriction: partial.kineticFriction ?? 0.3,
    charge: partial.charge ?? 0,
    isStatic: partial.isStatic ?? false,
    collidable: partial.collidable ?? (partial.shape !== 'point'),
    collisionGroup: partial.collisionGroup ?? 0,
    fillColor: partial.fillColor ?? (partial.isStatic ? '#d1d5db' : pickBodyColor(uidCounter)),
    strokeColor: partial.strokeColor ?? '#1e293b',
    showCOM: partial.showCOM ?? true,
    showName: partial.showName ?? false,
    trackCOM: partial.trackCOM ?? false,
    trackOutline: partial.trackOutline ?? false,
    tracePoints: [],
    thruster: partial.thruster,
  };

  recalcBodyMassProperties(body);
  return body;
}

export interface PresetBundle {
  id: string;
  name: string;
  description: string;
  state: SimulationState;
  meters: MeterWidget[];
  controls: ControlWidget[];
}

export const PRESET_LIST: Array<{ id: string; name: string; description: string }> = [
  { id: 'blank', name: 'Blank Workspace', description: 'Empty workspace with metric grid and Earth gravity' },
  { id: 'pendulum', name: 'Simple & Double Pendulum', description: 'Two linked pendulums with live Kinetic & Potential Energy meter' },
  { id: 'projectile', name: 'Projectile Motion Lab', description: 'Launch sphere with parabolic path trace and velocity vectors' },
  { id: 'inclined', name: 'Inclined Plane & Friction', description: 'Block sliding down an adjustable ramp with force vectors' },
  { id: 'spring', name: 'Harmonic Spring Oscillator', description: 'Mass on a vertical Hooke spring with position & velocity graph' },
  { id: 'atwood', name: 'Atwood Machine (Pulley & Rope)', description: 'Two unequal masses coupled by a rope over a frictionless pulley' },
  { id: 'cradle', name: "Newton's Cradle", description: '5 elastic steel spheres demonstrating conservation of momentum' },
  { id: 'crank', name: 'Motorized Gear & Slider-Crank', description: 'Rotary motor driving meshed gears and horizontal slot slider' },
  { id: 'car', name: 'Suspension Vehicle & Ramp', description: 'Spring-damper chassis with motorized wheels crossing terrain' },
  { id: 'orbit', name: 'Planetary Orbit (Gravity Field)', description: 'Inverse-square central gravity with elliptical orbit trace' },
];

export function loadPreset(presetId: string): PresetBundle {
  const world = createDefaultWorldSettings();

  switch (presetId) {
    case 'pendulum': {
      const bob1 = createBody({
        id: 'bob1',
        name: 'Pendulum Bob 1',
        shape: 'circle',
        x: 2.2,
        y: 1.8,
        radius: 0.4,
        mass: 3,
        fillColor: '#60a5fa',
        trackCOM: true,
      });
      const bob2 = createBody({
        id: 'bob2',
        name: 'Pendulum Bob 2',
        shape: 'circle',
        x: 4.1,
        y: 1.5,
        radius: 0.35,
        mass: 2,
        fillColor: '#f87171',
        trackCOM: true,
      });
      const rod1: PhysicsConstraint = {
        id: 'rod1',
        name: 'Arm 1',
        kind: 'rod',
        bodyAId: null,
        bodyBId: bob1.id,
        localA: { x: 0, y: 3.2 },
        localB: { x: 0, y: 0 },
        restLength: Math.hypot(2.2, 1.8 - 3.2),
        enabled: true,
        color: '#334155',
      };
      const rod2: PhysicsConstraint = {
        id: 'rod2',
        name: 'Arm 2',
        kind: 'rod',
        bodyAId: bob1.id,
        bodyBId: bob2.id,
        localA: { x: 0, y: 0 },
        localB: { x: 0, y: 0 },
        restLength: Math.hypot(4.1 - 2.2, 1.5 - 1.8),
        enabled: true,
        color: '#334155',
      };
      world.airResistance = 0;
      return {
        id: presetId,
        name: 'Simple & Double Pendulum',
        description: 'Energy conservation in a double pendulum',
        state: {
          time: 0,
          frame: 0,
          bodies: [bob1, bob2],
          constraints: [rod1, rod2],
          annotations: [
            { id: 't1', x: -3.5, y: 3.6, text: 'Double Pendulum (Zero Air Drag)', fontSize: 15, color: '#1e293b' },
          ],
          world,
        },
        meters: [
          {
            id: 'm_energy',
            title: 'Bob 2 Energy (KE / PE / Total)',
            bodyId: bob2.id,
            metric: 'energy',
            displayType: 'graph',
            x: 16,
            y: 16,
            width: 260,
            height: 150,
            data: [],
          },
        ],
        controls: [
          {
            id: 'c_grav',
            label: 'Gravity Y',
            targetType: 'world',
            targetId: null,
            property: 'gy',
            min: -20,
            max: 0,
            step: 0.1,
            unit: 'm/s²',
            x: 16,
            y: 180,
          },
        ],
      };
    }

    case 'projectile': {
      const ground = createBody({
        id: 'ground',
        name: 'Ground',
        shape: 'rect',
        x: 0,
        y: -3.4,
        width: 20,
        height: 0.7,
        isStatic: true,
        restitution: 0.7,
        fillColor: '#cbd5e1',
      });
      const ball = createBody({
        id: 'proj',
        name: 'Projectile',
        shape: 'circle',
        x: -5.5,
        y: -2.5,
        radius: 0.35,
        vx: 7.5,
        vy: 9.2,
        mass: 2,
        restitution: 0.75,
        fillColor: '#fb923c',
        trackCOM: true,
      });
      world.showVectors.velocity = true;
      world.airResistance = 0;
      return {
        id: presetId,
        name: 'Projectile Motion Lab',
        description: 'Parabolic motion with velocity vectors and live Vy/Vx graph',
        state: {
          time: 0,
          frame: 0,
          bodies: [ground, ball],
          constraints: [],
          annotations: [
            { id: 't_proj', x: -5.8, y: 3.3, text: 'Projectile Launch (Vx = 7.5 m/s, Vy = 9.2 m/s)', fontSize: 15, color: '#1e293b' },
          ],
          world,
        },
        meters: [
          {
            id: 'm_vel',
            title: 'Projectile Velocity (Vx, Vy)',
            bodyId: ball.id,
            metric: 'velocity',
            displayType: 'graph',
            x: 16,
            y: 16,
            width: 260,
            height: 150,
            data: [],
          },
        ],
        controls: [
          {
            id: 'c_drag',
            label: 'Air Resistance',
            targetType: 'world',
            targetId: null,
            property: 'airResistance',
            min: 0,
            max: 1.5,
            step: 0.05,
            unit: 'N·s/m',
            x: 16,
            y: 180,
          },
        ],
      };
    }

    case 'inclined': {
      const floor = createBody({
        id: 'floor',
        name: 'Base Floor',
        shape: 'rect',
        x: 0,
        y: -3.4,
        width: 18,
        height: 0.6,
        isStatic: true,
        fillColor: '#cbd5e1',
      });
      const ramp = createBody({
        id: 'ramp',
        name: 'Inclined Plane',
        shape: 'polygon',
        x: -1.8,
        y: -1.45,
        vertices: [
          { x: -3.5, y: -1.5 },
          { x: 3.5, y: -1.5 },
          { x: -3.5, y: 1.8 },
        ],
        isStatic: true,
        staticFriction: 0.25,
        kineticFriction: 0.2,
        fillColor: '#94a3b8',
      });
      const block = createBody({
        id: 'block',
        name: 'Sliding Block',
        shape: 'rect',
        x: -3.3,
        y: 0.9,
        width: 1.1,
        height: 0.7,
        angle: -Math.atan2(3.3, 7.0),
        mass: 5,
        staticFriction: 0.22,
        kineticFriction: 0.18,
        restitution: 0.05,
        fillColor: '#fde047',
        trackCOM: true,
      });
      world.showVectors.force = true;
      world.showVectors.velocity = true;
      return {
        id: presetId,
        name: 'Inclined Plane & Friction',
        description: 'Block sliding down ramp with friction and net force vectors',
        state: {
          time: 0,
          frame: 0,
          bodies: [floor, ramp, block],
          constraints: [],
          annotations: [
            { id: 't_inc', x: -5.5, y: 3.2, text: 'Inclined Plane & Coulomb Friction', fontSize: 15, color: '#1e293b' },
          ],
          world,
        },
        meters: [
          {
            id: 'm_acc',
            title: 'Block Acceleration (m/s²)',
            bodyId: block.id,
            metric: 'acceleration',
            displayType: 'digital',
            x: 16,
            y: 16,
            width: 240,
            height: 110,
            data: [],
          },
        ],
        controls: [
          {
            id: 'c_fric',
            label: 'Block Friction μk',
            targetType: 'body',
            targetId: block.id,
            property: 'kineticFriction',
            min: 0,
            max: 1.0,
            step: 0.02,
            unit: '',
            x: 16,
            y: 140,
          },
        ],
      };
    }

    case 'spring': {
      const ceiling = createBody({
        id: 'ceil',
        name: 'Support Beam',
        shape: 'rect',
        x: 0,
        y: 3.2,
        width: 4.0,
        height: 0.45,
        isStatic: true,
        fillColor: '#94a3b8',
      });
      const massBox = createBody({
        id: 'massBox',
        name: 'Oscillator Mass',
        shape: 'rect',
        x: 0,
        y: -0.6,
        width: 1.2,
        height: 1.0,
        mass: 4.0,
        fillColor: '#93c5fd',
        trackCOM: true,
      });
      const spring: PhysicsConstraint = {
        id: 'spr1',
        name: 'Hooke Spring',
        kind: 'spring',
        bodyAId: ceiling.id,
        bodyBId: massBox.id,
        localA: { x: 0, y: -0.22 },
        localB: { x: 0, y: 0.5 },
        stiffness: 45,
        damping: 0.25,
        restLength: 2.0,
        enabled: true,
        color: '#2563eb',
      };
      world.airResistance = 0;
      return {
        id: presetId,
        name: 'Harmonic Spring Oscillator',
        description: 'Hooke spring oscillation with sinusoidal position chart',
        state: {
          time: 0,
          frame: 0,
          bodies: [ceiling, massBox],
          constraints: [spring],
          annotations: [
            { id: 't_spr', x: -3.5, y: 3.8, text: 'Simple Harmonic Oscillator (k = 45 N/m, m = 4 kg)', fontSize: 14, color: '#1e293b' },
          ],
          world,
        },
        meters: [
          {
            id: 'm_pos',
            title: 'Mass Position Y (m)',
            bodyId: massBox.id,
            metric: 'position',
            displayType: 'graph',
            x: 16,
            y: 16,
            width: 270,
            height: 160,
            data: [],
          },
        ],
        controls: [
          {
            id: 'c_k',
            label: 'Spring Stiffness k',
            targetType: 'constraint',
            targetId: spring.id,
            property: 'stiffness',
            min: 5,
            max: 150,
            step: 1,
            unit: 'N/m',
            x: 16,
            y: 190,
          },
        ],
      };
    }

    case 'atwood': {
      const pulley = createBody({
        id: 'pulley1',
        name: 'Pulley Wheel',
        shape: 'pulley',
        x: 0,
        y: 2.2,
        radius: 0.75,
        isStatic: true,
        collidable: false,
        fillColor: '#cbd5e1',
      });
      const m1 = createBody({
        id: 'm1',
        name: 'Mass A (3 kg)',
        shape: 'rect',
        x: -0.75,
        y: -0.2,
        width: 0.9,
        height: 0.9,
        mass: 3.0,
        fillColor: '#86efac',
      });
      const m2 = createBody({
        id: 'm2',
        name: 'Mass B (5 kg)',
        shape: 'rect',
        x: 0.75,
        y: 0.4,
        width: 1.0,
        height: 1.0,
        mass: 5.0,
        fillColor: '#fca5a5',
      });
      const rope: PhysicsConstraint = {
        id: 'rope_atwood',
        name: 'Pulley Rope',
        kind: 'rope',
        bodyAId: m1.id,
        bodyBId: m2.id,
        localA: { x: 0, y: 0.45 },
        localB: { x: 0, y: 0.5 },
        pulleyBodyId: pulley.id,
        maxLength: 3.85,
        enabled: true,
        color: '#92400e',
      };
      return {
        id: presetId,
        name: 'Atwood Machine (Pulley & Rope)',
        description: 'Coupled masses over an ideal pulley',
        state: {
          time: 0,
          frame: 0,
          bodies: [pulley, m1, m2],
          constraints: [rope],
          annotations: [
            { id: 't_atw', x: -3.6, y: 3.5, text: 'Atwood Machine: m1 = 3 kg, m2 = 5 kg', fontSize: 15, color: '#1e293b' },
          ],
          world,
        },
        meters: [
          {
            id: 'm_vel_atw',
            title: 'Mass B Velocity (Vy)',
            bodyId: m2.id,
            metric: 'velocity',
            displayType: 'graph',
            x: 16,
            y: 16,
            width: 250,
            height: 145,
            data: [],
          },
        ],
        controls: [
          {
            id: 'c_m2',
            label: 'Mass B',
            targetType: 'body',
            targetId: m2.id,
            property: 'mass',
            min: 1,
            max: 15,
            step: 0.5,
            unit: 'kg',
            x: 16,
            y: 175,
          },
        ],
      };
    }

    case 'cradle': {
      const bodies: RigidBody[] = [];
      const constraints: PhysicsConstraint[] = [];
      const r = 0.45;
      const pivotY = 3.0;
      const bobY = 0.0;
      for (let i = 0; i < 5; i++) {
        const bx = (i - 2) * (r * 2);
        const startX = i === 0 ? bx - 2.3 : bx;
        const startY = i === 0 ? 1.15 : bobY;
        const ball = createBody({
          id: `cradle_${i}`,
          name: `Sphere ${i + 1}`,
          shape: 'circle',
          x: startX,
          y: startY,
          radius: r,
          mass: 2.0,
          restitution: 1.0,
          staticFriction: 0.0,
          kineticFriction: 0.0,
          fillColor: '#cbd5e1',
        });
        bodies.push(ball);
        constraints.push({
          id: `cradle_rod_${i}`,
          name: `String ${i + 1}`,
          kind: 'rod',
          bodyAId: null,
          bodyBId: ball.id,
          localA: { x: bx, y: pivotY },
          localB: { x: 0, y: 0 },
          restLength: pivotY - bobY,
          enabled: true,
          color: '#475569',
        });
      }
      world.airResistance = 0;
      return {
        id: presetId,
        name: "Newton's Cradle",
        description: 'Elastic momentum transfer between 5 spheres',
        state: {
          time: 0,
          frame: 0,
          bodies,
          constraints,
          annotations: [
            { id: 't_nc', x: -3.2, y: 3.5, text: "Newton's Cradle (e = 1.0)", fontSize: 15, color: '#1e293b' },
          ],
          world,
        },
        meters: [
          {
            id: 'm_mom',
            title: 'Sphere 5 Momentum',
            bodyId: 'cradle_4',
            metric: 'momentum',
            displayType: 'graph',
            x: 16,
            y: 16,
            width: 250,
            height: 145,
            data: [],
          },
        ],
        controls: [],
      };
    }

    case 'crank': {
      const g1 = createBody({
        id: 'gear1',
        name: 'Drive Gear (12T)',
        shape: 'gear',
        x: -2.4,
        y: 0.5,
        radius: 1.1,
        teeth: 12,
        mass: 4.0,
        collidable: false,
        fillColor: '#93c5fd',
      });
      const g2 = createBody({
        id: 'gear2',
        name: 'Driven Gear (18T)',
        shape: 'gear',
        x: 0.35,
        y: 0.5,
        radius: 1.65,
        teeth: 18,
        mass: 6.0,
        collidable: false,
        fillColor: '#fdba74',
      });
      const slider = createBody({
        id: 'slider',
        name: 'Piston Slider',
        shape: 'rect',
        x: 4.2,
        y: 0.5,
        width: 1.1,
        height: 0.8,
        mass: 2.0,
        collidable: false,
        fillColor: '#86efac',
      });
      const pin1: PhysicsConstraint = {
        id: 'pin_g1',
        name: 'Pin Gear 1',
        kind: 'pin',
        bodyAId: g1.id,
        bodyBId: null,
        localA: { x: 0, y: 0 },
        localB: { x: -2.4, y: 0.5 },
        enabled: true,
        color: '#334155',
      };
      const pin2: PhysicsConstraint = {
        id: 'pin_g2',
        name: 'Pin Gear 2',
        kind: 'pin',
        bodyAId: g2.id,
        bodyBId: null,
        localA: { x: 0, y: 0 },
        localB: { x: 0.35, y: 0.5 },
        enabled: true,
        color: '#334155',
      };
      const motor: PhysicsConstraint = {
        id: 'motor1',
        name: 'Drive Motor',
        kind: 'motor',
        bodyAId: g1.id,
        bodyBId: null,
        localA: { x: 0, y: 0 },
        localB: { x: -2.4, y: 0.5 },
        motorTargetOmega: 3.0,
        motorMaxTorque: 120,
        enabled: true,
        color: '#dc2626',
      };
      const mesh: PhysicsConstraint = {
        id: 'mesh1',
        name: 'Gear Mesh',
        kind: 'gearMesh',
        bodyAId: g1.id,
        bodyBId: g2.id,
        localA: { x: 0, y: 0 },
        localB: { x: 0, y: 0 },
        enabled: true,
        color: '#475569',
      };
      const slot: PhysicsConstraint = {
        id: 'slot_slider',
        name: 'Horizontal Track',
        kind: 'slot',
        bodyAId: slider.id,
        bodyBId: null,
        localA: { x: 0, y: 0 },
        localB: { x: 4.2, y: 0.5 },
        slotAngle: 0,
        slotKeyed: true,
        slotLength: 5.0,
        referenceAngle: 0,
        enabled: true,
        color: '#0f172a',
      };
      const linkRod: PhysicsConstraint = {
        id: 'crank_rod',
        name: 'Connecting Rod',
        kind: 'rod',
        bodyAId: g2.id,
        bodyBId: slider.id,
        localA: { x: 1.15, y: 0 },
        localB: { x: 0, y: 0 },
        restLength: Math.hypot(4.2 - (0.35 + 1.15), 0),
        enabled: true,
        color: '#0284c7',
      };
      return {
        id: presetId,
        name: 'Motorized Gear & Slider-Crank',
        description: 'Rotary motor driving meshed gears & piston slider',
        state: {
          time: 0,
          frame: 0,
          bodies: [g1, g2, slider],
          constraints: [pin1, pin2, motor, mesh, slot, linkRod],
          annotations: [
            { id: 't_crk', x: -4.2, y: 3.1, text: 'Motorized Gear Train + Slider-Crank Mechanism', fontSize: 15, color: '#1e293b' },
          ],
          world,
        },
        meters: [
          {
            id: 'm_slider_v',
            title: 'Piston Velocity (Vx)',
            bodyId: slider.id,
            metric: 'velocity',
            displayType: 'graph',
            x: 16,
            y: 16,
            width: 255,
            height: 145,
            data: [],
          },
        ],
        controls: [
          {
            id: 'c_omega',
            label: 'Motor Speed ω',
            targetType: 'constraint',
            targetId: motor.id,
            property: 'motorTargetOmega',
            min: -8,
            max: 8,
            step: 0.5,
            unit: 'rad/s',
            x: 16,
            y: 175,
          },
        ],
      };
    }

    case 'car': {
      const road = createBody({
        id: 'road',
        name: 'Road',
        shape: 'rect',
        x: 2,
        y: -3.2,
        width: 28,
        height: 0.6,
        isStatic: true,
        staticFriction: 0.85,
        kineticFriction: 0.75,
        fillColor: '#cbd5e1',
      });
      const bump = createBody({
        id: 'bump',
        name: 'Speed Bump',
        shape: 'polygon',
        x: 1.5,
        y: -2.75,
        vertices: [
          { x: -1.1, y: -0.2 },
          { x: 1.1, y: -0.2 },
          { x: 0, y: 0.25 },
        ],
        isStatic: true,
        staticFriction: 0.8,
        kineticFriction: 0.7,
        fillColor: '#94a3b8',
      });
      const chassis = createBody({
        id: 'chassis',
        name: 'Car Chassis',
        shape: 'rect',
        x: -4.5,
        y: -1.4,
        width: 3.0,
        height: 0.7,
        mass: 8.0,
        collisionGroup: 2,
        fillColor: '#f87171',
        trackCOM: true,
      });
      const wheelL = createBody({
        id: 'wheelL',
        name: 'Rear Wheel',
        shape: 'circle',
        x: -5.6,
        y: -2.35,
        radius: 0.5,
        mass: 2.0,
        staticFriction: 0.9,
        kineticFriction: 0.85,
        collisionGroup: 1,
        fillColor: '#334155',
      });
      const wheelR = createBody({
        id: 'wheelR',
        name: 'Front Wheel',
        shape: 'circle',
        x: -3.4,
        y: -2.35,
        radius: 0.5,
        mass: 2.0,
        staticFriction: 0.9,
        kineticFriction: 0.85,
        collisionGroup: 1,
        fillColor: '#334155',
      });
      const slotL: PhysicsConstraint = {
        id: 'susp_slot_L',
        name: 'Rear Suspension Slot',
        kind: 'slot',
        bodyAId: wheelL.id,
        bodyBId: chassis.id,
        localA: { x: 0, y: 0 },
        localB: { x: -1.1, y: -0.95 },
        slotAngle: Math.PI / 2,
        slotKeyed: false,
        slotLength: 0.9,
        enabled: true,
        color: '#64748b',
      };
      const slotR: PhysicsConstraint = {
        id: 'susp_slot_R',
        name: 'Front Suspension Slot',
        kind: 'slot',
        bodyAId: wheelR.id,
        bodyBId: chassis.id,
        localA: { x: 0, y: 0 },
        localB: { x: 1.1, y: -0.95 },
        slotAngle: Math.PI / 2,
        slotKeyed: false,
        slotLength: 0.9,
        enabled: true,
        color: '#64748b',
      };
      const sprL: PhysicsConstraint = {
        id: 'susp_spr_L',
        name: 'Rear Spring',
        kind: 'spring',
        bodyAId: chassis.id,
        bodyBId: wheelL.id,
        localA: { x: -1.1, y: -0.1 },
        localB: { x: 0, y: 0 },
        stiffness: 180,
        damping: 14,
        restLength: 0.85,
        enabled: true,
        color: '#2563eb',
      };
      const sprR: PhysicsConstraint = {
        id: 'susp_spr_R',
        name: 'Front Spring',
        kind: 'spring',
        bodyAId: chassis.id,
        bodyBId: wheelR.id,
        localA: { x: 1.1, y: -0.1 },
        localB: { x: 0, y: 0 },
        stiffness: 180,
        damping: 14,
        restLength: 0.85,
        enabled: true,
        color: '#2563eb',
      };
      const motorL: PhysicsConstraint = {
        id: 'car_motor',
        name: 'Wheel Drive Motor',
        kind: 'motor',
        bodyAId: wheelL.id,
        bodyBId: chassis.id,
        localA: { x: 0, y: 0 },
        localB: { x: -1.1, y: -0.95 },
        motorTargetOmega: -6.0,
        motorMaxTorque: 85,
        enabled: true,
        color: '#dc2626',
      };
      return {
        id: presetId,
        name: 'Suspension Vehicle & Ramp',
        description: 'Spring-damper car crossing a speed bump',
        state: {
          time: 0,
          frame: 0,
          bodies: [road, bump, chassis, wheelL, wheelR],
          constraints: [slotL, slotR, sprL, sprR, motorL],
          annotations: [
            { id: 't_car', x: -6.2, y: 2.8, text: 'Spring-Damper Suspension Vehicle', fontSize: 15, color: '#1e293b' },
          ],
          world,
        },
        meters: [
          {
            id: 'm_car_y',
            title: 'Chassis Ride Height Y',
            bodyId: chassis.id,
            metric: 'position',
            displayType: 'graph',
            x: 16,
            y: 16,
            width: 250,
            height: 145,
            data: [],
          },
        ],
        controls: [
          {
            id: 'c_car_spd',
            label: 'Drive Motor ω',
            targetType: 'constraint',
            targetId: motorL.id,
            property: 'motorTargetOmega',
            min: -12,
            max: 12,
            step: 0.5,
            unit: 'rad/s',
            x: 16,
            y: 175,
          },
        ],
      };
    }

    case 'orbit': {
      world.gravityMode = 'planetary';
      world.gravityG = 1.2;
      world.airResistance = 0;
      const sun = createBody({
        id: 'sun',
        name: 'Central Star',
        shape: 'circle',
        x: 0,
        y: 0,
        radius: 0.85,
        mass: 50,
        isStatic: true,
        fillColor: '#fde047',
      });
      const planet = createBody({
        id: 'planet',
        name: 'Planet',
        shape: 'circle',
        x: 3.4,
        y: 0,
        radius: 0.32,
        vy: 4.1,
        mass: 1.0,
        fillColor: '#60a5fa',
        trackCOM: true,
      });
      const comet = createBody({
        id: 'comet',
        name: 'Comet',
        shape: 'circle',
        x: -4.8,
        y: 0,
        radius: 0.2,
        vy: -2.4,
        mass: 0.4,
        fillColor: '#f87171',
        trackCOM: true,
      });
      world.showVectors.velocity = true;
      return {
        id: presetId,
        name: 'Planetary Orbit',
        description: 'Planetary inverse-square gravity orbits',
        state: {
          time: 0,
          frame: 0,
          bodies: [sun, planet, comet],
          constraints: [],
          annotations: [
            { id: 't_orb', x: -4.8, y: 3.5, text: 'Planetary Gravitation F = G·m1·m2 / r²', fontSize: 15, color: '#1e293b' },
          ],
          world,
        },
        meters: [
          {
            id: 'm_orb_v',
            title: 'Comet Speed |V|',
            bodyId: comet.id,
            metric: 'velocity',
            displayType: 'graph',
            x: 16,
            y: 16,
            width: 250,
            height: 145,
            data: [],
          },
        ],
        controls: [],
      };
    }

    case 'blank':
    default: {
      return {
        id: 'blank',
        name: 'Blank Workspace',
        description: 'Clean Interactive Physics workspace',
        state: {
          time: 0,
          frame: 0,
          bodies: [],
          constraints: [],
          annotations: [],
          world,
        },
        meters: [],
        controls: [],
      };
    }
  }
}
