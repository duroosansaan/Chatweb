export type ToolId =
  | 'select'
  | 'rotate'
  | 'zoom'
  | 'pan'
  | 'rect'
  | 'square'
  | 'circle'
  | 'polygon'
  | 'point'
  | 'anchor'
  | 'pin'
  | 'weld'
  | 'hslot'
  | 'vslot'
  | 'slot'
  | 'spring'
  | 'damper'
  | 'rope'
  | 'rod'
  | 'actuator'
  | 'motor'
  | 'gear'
  | 'pulley'
  | 'force'
  | 'torque'
  | 'thruster'
  | 'text';

export type BodyShape = 'rect' | 'circle' | 'polygon' | 'gear' | 'pulley' | 'point';

export interface Vec2 {
  x: number;
  y: number;
}

export interface RigidBody {
  id: string;
  name: string;
  shape: BodyShape;
  x: number; // world meters
  y: number; // world meters (Y-up)
  angle: number; // radians
  vx: number; // m/s
  vy: number; // m/s
  omega: number; // rad/s
  ax: number; // m/s^2 (measured)
  ay: number; // m/s^2 (measured)
  alpha: number; // rad/s^2 (measured)
  fx: number; // N (accumulated / measured net force)
  fy: number; // N
  torque: number; // N*m
  contactFx: number; // N (from solver impulses)
  contactFy: number; // N

  // Geometry
  width: number; // for rect
  height: number; // for rect
  radius: number; // for circle / gear / pulley / point
  vertices: Vec2[]; // local space CCW vertices for polygon / rect
  teeth: number; // for gear

  // Physical material
  mass: number; // kg
  invMass: number; // 1/kg (0 if static)
  density: number; // kg/m^2
  inertia: number; // kg*m^2
  invInertia: number; // 1/(kg*m^2) (0 if static)
  restitution: number; // 0..1
  staticFriction: number; // mu_s
  kineticFriction: number; // mu_k
  charge: number; // Coulombs (micro/unit)
  isStatic: boolean;
  collidable: boolean;
  collisionGroup: number; // 0 = collide with all, >0 = layer mask

  // Appearance & Tracking
  fillColor: string;
  strokeColor: string;
  showCOM: boolean;
  showName: boolean;
  trackCOM: boolean;
  trackOutline: boolean;
  tracePoints: Vec2[];

  // Optional attached thruster
  thruster?: {
    enabled: boolean;
    fx: number;
    fy: number;
    localX: number;
    localY: number;
  };
}

export type ConstraintKind =
  | 'pin'
  | 'weld'
  | 'slot'
  | 'spring'
  | 'damper'
  | 'rope'
  | 'rod'
  | 'actuator'
  | 'motor'
  | 'force'
  | 'torque'
  | 'gearMesh';

export interface PhysicsConstraint {
  id: string;
  name: string;
  kind: ConstraintKind;
  bodyAId: string | null; // null = world background
  bodyBId: string | null; // null = world background
  localA: Vec2; // anchor on bodyA (or world if null)
  localB: Vec2; // anchor on bodyB (or world if null)
  enabled: boolean;
  color: string;

  // Weld / Pin
  referenceAngle?: number;

  // Slot
  slotAngle?: number; // world or bodyB axis angle (radians)
  slotKeyed?: boolean; // if true, locks rotation too
  slotLength?: number; // visual track length

  // Spring / Damper / Rope / Rod / Actuator
  stiffness?: number; // k (N/m)
  damping?: number; // c (N*s/m)
  restLength?: number; // L0 (m)
  maxLength?: number; // for rope (m)
  actuatorMode?: 'sine' | 'velocity';
  actuatorAmplitude?: number; // m
  actuatorFrequency?: number; // rad/s
  actuatorVelocity?: number; // m/s

  // Motor
  motorTargetOmega?: number; // rad/s
  motorMaxTorque?: number; // N*m

  // Force / Torque arrow
  forceX?: number; // N
  forceY?: number; // N
  forceFollowBody?: boolean;
  torqueValue?: number; // N*m

  // Pulley support: optional intermediate pulley body
  pulleyBodyId?: string | null;
}

export interface TextAnnotation {
  id: string;
  x: number;
  y: number;
  text: string;
  fontSize: number;
  color: string;
}

export type MeterMetric =
  | 'time'
  | 'position'
  | 'velocity'
  | 'acceleration'
  | 'force'
  | 'momentum'
  | 'energy';

export type MeterDisplayType = 'graph' | 'digital' | 'bar';

export interface MeterDataPoint {
  t: number;
  v1: number;
  v2: number;
  v3: number;
}

export interface MeterWidget {
  id: string;
  title: string;
  bodyId: string | null;
  metric: MeterMetric;
  displayType: MeterDisplayType;
  x: number; // screen px relative to canvas wrapper
  y: number;
  width: number;
  height: number;
  data: MeterDataPoint[];
}

export interface ControlWidget {
  id: string;
  label: string;
  targetType: 'world' | 'body' | 'constraint';
  targetId: string | null;
  property:
    | 'gy'
    | 'gx'
    | 'airResistance'
    | 'mass'
    | 'restitution'
    | 'kineticFriction'
    | 'stiffness'
    | 'damping'
    | 'motorTargetOmega'
    | 'forceX'
    | 'forceY'
    | 'charge';
  min: number;
  max: number;
  step: number;
  unit: string;
  x: number;
  y: number;
}

export interface ScriptRule {
  id: string;
  name: string;
  enabled: boolean;
  targetType: 'body' | 'constraint' | 'world';
  targetId: string | null;
  property: string;
  expression: string; // e.g. "-12 * x" or "5 * sin(2 * t)"
}

export type GravityMode = 'vertical' | 'planetary' | 'none';
export type UnitSystem = 'SI' | 'CGS' | 'Imperial';

export interface WorldSettings {
  gravityMode: GravityMode;
  gx: number; // m/s^2
  gy: number; // m/s^2 (default -9.81)
  gravityG: number; // universal G multiplier for planetary
  airResistance: number; // drag coeff
  electrostaticsEnabled: boolean;
  electrostaticK: number;
  dt: number; // integration timestep e.g. 1/60
  substeps: number; // solver substeps per frame
  solverIterations: number;
  autoStopTime: number | null; // optional pause at time T

  // View settings
  showGrid: boolean;
  gridSpacing: number; // meters
  snapToGrid: boolean;
  showRulers: boolean;
  showAxes: boolean;
  showCOM: boolean;
  showCoordinates: boolean;
  showTrackPoints: boolean;
  showVectors: {
    velocity: boolean;
    acceleration: boolean;
    force: boolean;
    gravity: boolean;
    contact: boolean;
  };
  vectorScale: number;
  unitSystem: UnitSystem;
  precision: number;
}

export interface SimulationState {
  time: number;
  frame: number;
  bodies: RigidBody[];
  constraints: PhysicsConstraint[];
  annotations: TextAnnotation[];
  world: WorldSettings;
}

export interface TapeFrame {
  time: number;
  frame: number;
  bodies: Array<{
    id: string;
    x: number;
    y: number;
    angle: number;
    vx: number;
    vy: number;
    omega: number;
    ax: number;
    ay: number;
    alpha: number;
    fx: number;
    fy: number;
    torque: number;
    contactFx: number;
    contactFy: number;
  }>;
}

export interface CameraState {
  panX: number; // world X at screen center
  panY: number; // world Y at screen center
  ppm: number; // pixels per meter (default 50)
}
