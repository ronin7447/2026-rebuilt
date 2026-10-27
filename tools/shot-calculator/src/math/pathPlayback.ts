import { driveLimits, type Point, type Pose2d } from './field';

export interface PathSample {
  timeSec: number;
  pose: Pose2d;
  velocity: Point;
  acceleration: Point;
  omegaRadPerSec: number;
  angularAccelRadPerSec2: number;
}

export interface LoadedPath {
  name: string;
  source: 'PathPlanner' | 'Choreo';
  totalTimeSec: number;
  samples: PathSample[];
  message: string;
}

interface PathPlannerWaypoint {
  anchor: Point;
  prevControl?: Point | null;
  nextControl?: Point | null;
}

export function parsePathFile(fileName: string, rawText: string): LoadedPath {
  const json = JSON.parse(rawText) as unknown;
  const name = fileName.replace(/\.(path|traj|chor|choreo|json)$/i, '');
  const pathPlanner = parsePathPlannerPath(name, json);
  if (pathPlanner) return pathPlanner;

  const choreo = parseChoreoTrajectory(name, json);
  if (choreo) return choreo;

  throw new Error('Unsupported path format. Expected a PathPlanner .path or Choreo-style sampled trajectory JSON.');
}

export function sampleLoadedPath(path: LoadedPath, timeSec: number): PathSample {
  if (path.samples.length === 0) {
    throw new Error('Cannot sample empty path');
  }
  if (path.samples.length === 1) return path.samples[0];

  const t = clamp(timeSec, 0, path.totalTimeSec);
  let upperIndex = path.samples.findIndex((sample) => sample.timeSec >= t);
  if (upperIndex <= 0) return path.samples[0];
  if (upperIndex < 0) return path.samples[path.samples.length - 1];

  const lower = path.samples[upperIndex - 1];
  const upper = path.samples[upperIndex];
  const span = Math.max(upper.timeSec - lower.timeSec, 1e-9);
  const alpha = (t - lower.timeSec) / span;
  return {
    timeSec: t,
    pose: {
      x: lerp(lower.pose.x, upper.pose.x, alpha),
      y: lerp(lower.pose.y, upper.pose.y, alpha),
      headingRad: interpolateAngle(lower.pose.headingRad, upper.pose.headingRad, alpha),
    },
    velocity: {
      x: lerp(lower.velocity.x, upper.velocity.x, alpha),
      y: lerp(lower.velocity.y, upper.velocity.y, alpha),
    },
    acceleration: {
      x: lerp(lower.acceleration.x, upper.acceleration.x, alpha),
      y: lerp(lower.acceleration.y, upper.acceleration.y, alpha),
    },
    omegaRadPerSec: lerp(lower.omegaRadPerSec, upper.omegaRadPerSec, alpha),
    angularAccelRadPerSec2: lerp(lower.angularAccelRadPerSec2, upper.angularAccelRadPerSec2, alpha),
  };
}

function parsePathPlannerPath(name: string, json: unknown): LoadedPath | null {
  if (!isObject(json) || !Array.isArray(json.waypoints)) return null;
  const waypoints = json.waypoints
    .map(parseWaypoint)
    .filter((waypoint): waypoint is PathPlannerWaypoint => waypoint !== null);
  if (waypoints.length < 2) return null;

  const constraints = isObject(json.globalConstraints) ? json.globalConstraints : {};
  const maxVelocityMps = numberProp(constraints, 'maxVelocity', driveLimits.maxFreeSpeedMps);
  const maxAccelerationMps2 = numberProp(constraints, 'maxAcceleration', driveLimits.maxLinearAccelMps2);
  const initialVelocityMps = isObject(json.idealStartingState)
    ? numberProp(json.idealStartingState, 'velocity', 0)
    : 0;
  const finalVelocityMps = isObject(json.goalEndState)
    ? numberProp(json.goalEndState, 'velocity', 0)
    : 0;

  const points = samplePathPlannerGeometry(waypoints);
  const rotations = rotationKeyframes(json);
  const samples = timedSamplesFromPolyline(
    points,
    rotations,
    maxVelocityMps,
    maxAccelerationMps2,
    initialVelocityMps,
    finalVelocityMps,
  );

  return {
    name,
    source: 'PathPlanner',
    totalTimeSec: samples[samples.length - 1]?.timeSec ?? 0,
    samples,
    message: `Loaded ${waypoints.length} waypoints with ${maxVelocityMps.toFixed(1)} m/s, ${maxAccelerationMps2.toFixed(1)} m/s^2 constraints.`,
  };
}

function parseChoreoTrajectory(name: string, json: unknown): LoadedPath | null {
  const samplesUnknown = findSampleArray(json);
  if (!samplesUnknown) return null;

  const parsed = samplesUnknown
    .map(parseChoreoSample)
    .filter((sample): sample is PathSample => sample !== null)
    .sort((a, b) => a.timeSec - b.timeSec);
  if (parsed.length < 2) return null;

  fillMissingDerivatives(parsed);
  return {
    name,
    source: 'Choreo',
    totalTimeSec: parsed[parsed.length - 1].timeSec,
    samples: parsed,
    message: `Loaded ${parsed.length} Choreo-style samples.`,
  };
}

function samplePathPlannerGeometry(waypoints: PathPlannerWaypoint[]): Point[] {
  const points: Point[] = [];
  for (let i = 0; i < waypoints.length - 1; i += 1) {
    const start = waypoints[i];
    const end = waypoints[i + 1];
    const p0 = start.anchor;
    const p1 = start.nextControl ?? start.anchor;
    const p2 = end.prevControl ?? end.anchor;
    const p3 = end.anchor;
    for (let step = 0; step <= 40; step += 1) {
      if (i > 0 && step === 0) continue;
      points.push(cubicBezier(p0, p1, p2, p3, step / 40));
    }
  }
  return points;
}

function timedSamplesFromPolyline(
  points: Point[],
  rotations: RotationKeyframe[],
  maxVelocityMps: number,
  maxAccelerationMps2: number,
  initialVelocityMps: number,
  finalVelocityMps: number,
): PathSample[] {
  const distances = [0];
  for (let i = 1; i < points.length; i += 1) {
    distances.push(distances[i - 1] + distance(points[i - 1], points[i]));
  }
  const totalDistance = distances[distances.length - 1] ?? 0;
  const velocities = points.map(() => Math.max(0, maxVelocityMps));
  velocities[0] = clamp(initialVelocityMps, 0, maxVelocityMps);
  velocities[velocities.length - 1] = clamp(finalVelocityMps, 0, maxVelocityMps);

  for (let i = 1; i < velocities.length; i += 1) {
    const ds = Math.max(distances[i] - distances[i - 1], 0);
    velocities[i] = Math.min(velocities[i], Math.sqrt((velocities[i - 1] ** 2) + (2 * maxAccelerationMps2 * ds)));
  }
  for (let i = velocities.length - 2; i >= 0; i -= 1) {
    const ds = Math.max(distances[i + 1] - distances[i], 0);
    velocities[i] = Math.min(velocities[i], Math.sqrt((velocities[i + 1] ** 2) + (2 * maxAccelerationMps2 * ds)));
  }

  const times = [0];
  for (let i = 1; i < points.length; i += 1) {
    const ds = Math.max(distances[i] - distances[i - 1], 0);
    const avgVelocity = Math.max((velocities[i] + velocities[i - 1]) / 2, 0.05);
    times.push(times[i - 1] + (ds / avgVelocity));
  }

  const samples = points.map((point, index): PathSample => {
    const headingRad = headingForDistance(rotations, distances[index], totalDistance);
    const tangent = tangentAt(points, index);
    return {
      timeSec: times[index],
      pose: { x: point.x, y: point.y, headingRad },
      velocity: { x: tangent.x * velocities[index], y: tangent.y * velocities[index] },
      acceleration: { x: 0, y: 0 },
      omegaRadPerSec: 0,
      angularAccelRadPerSec2: 0,
    };
  });

  fillMissingDerivatives(samples);
  return samples;
}

function fillMissingDerivatives(samples: PathSample[]): void {
  for (let i = 1; i < samples.length; i += 1) {
    const previous = samples[i - 1];
    const current = samples[i];
    const dt = Math.max(current.timeSec - previous.timeSec, 1e-9);
    current.acceleration = {
      x: (current.velocity.x - previous.velocity.x) / dt,
      y: (current.velocity.y - previous.velocity.y) / dt,
    };
    current.omegaRadPerSec = angleDelta(previous.pose.headingRad, current.pose.headingRad) / dt;
    current.angularAccelRadPerSec2 = (current.omegaRadPerSec - previous.omegaRadPerSec) / dt;
  }
  if (samples.length > 1) {
    samples[0].acceleration = samples[1].acceleration;
    samples[0].omegaRadPerSec = samples[1].omegaRadPerSec;
    samples[0].angularAccelRadPerSec2 = samples[1].angularAccelRadPerSec2;
  }
}

interface RotationKeyframe {
  distanceFraction: number;
  headingRad: number;
}

function rotationKeyframes(json: Record<string, unknown>): RotationKeyframe[] {
  const keyframes: RotationKeyframe[] = [];
  if (isObject(json.idealStartingState)) {
    keyframes.push({
      distanceFraction: 0,
      headingRad: degreesToRadians(numberProp(json.idealStartingState, 'rotation', 0)),
    });
  }
  if (Array.isArray(json.rotationTargets)) {
    for (const target of json.rotationTargets) {
      if (!isObject(target)) continue;
      keyframes.push({
        distanceFraction: clamp(numberProp(target, 'waypointRelativePos', 0), 0, 1),
        headingRad: degreesToRadians(numberProp(target, 'rotationDegrees', 0)),
      });
    }
  }
  if (isObject(json.goalEndState)) {
    keyframes.push({
      distanceFraction: 1,
      headingRad: degreesToRadians(numberProp(json.goalEndState, 'rotation', 0)),
    });
  }
  if (keyframes.length === 0) keyframes.push({ distanceFraction: 0, headingRad: 0 });
  return keyframes.sort((a, b) => a.distanceFraction - b.distanceFraction);
}

function headingForDistance(keyframes: RotationKeyframe[], distanceMeters: number, totalDistanceMeters: number): number {
  const fraction = totalDistanceMeters > 1e-9 ? distanceMeters / totalDistanceMeters : 0;
  let upperIndex = keyframes.findIndex((keyframe) => keyframe.distanceFraction >= fraction);
  if (upperIndex <= 0) return keyframes[0].headingRad;
  if (upperIndex < 0) return keyframes[keyframes.length - 1].headingRad;
  const lower = keyframes[upperIndex - 1];
  const upper = keyframes[upperIndex];
  const span = Math.max(upper.distanceFraction - lower.distanceFraction, 1e-9);
  return interpolateAngle(lower.headingRad, upper.headingRad, (fraction - lower.distanceFraction) / span);
}

function parseWaypoint(value: unknown): PathPlannerWaypoint | null {
  if (!isObject(value) || !isPoint(value.anchor)) return null;
  return {
    anchor: value.anchor,
    prevControl: isPoint(value.prevControl) ? value.prevControl : null,
    nextControl: isPoint(value.nextControl) ? value.nextControl : null,
  };
}

function findSampleArray(json: unknown): unknown[] | null {
  if (Array.isArray(json)) return json;
  if (!isObject(json)) return null;
  for (const key of ['samples', 'states', 'trajectory']) {
    const value = json[key];
    if (Array.isArray(value)) return value;
    if (isObject(value)) {
      const nested = findSampleArray(value);
      if (nested) return nested;
    }
  }
  return null;
}

function parseChoreoSample(value: unknown): PathSample | null {
  if (!isObject(value)) return null;
  const pose = isObject(value.pose) ? value.pose : value;
  const x = numberFromKeys(pose, ['x', 'translationX', 'fieldX']);
  const y = numberFromKeys(pose, ['y', 'translationY', 'fieldY']);
  if (x === null || y === null) return null;
  const headingRad = headingFromObject(pose);
  const timeSec = numberFromKeys(value, ['timestamp', 'time', 't', 'timestampSeconds']) ?? 0;
  const vx = numberFromKeys(value, ['vx', 'velocityX', 'fieldVx']) ?? 0;
  const vy = numberFromKeys(value, ['vy', 'velocityY', 'fieldVy']) ?? 0;
  const ax = numberFromKeys(value, ['ax', 'accelerationX', 'fieldAx']) ?? 0;
  const ay = numberFromKeys(value, ['ay', 'accelerationY', 'fieldAy']) ?? 0;
  return {
    timeSec,
    pose: { x, y, headingRad },
    velocity: { x: vx, y: vy },
    acceleration: { x: ax, y: ay },
    omegaRadPerSec: numberFromKeys(value, ['omega', 'angularVelocity', 'angularVelocityRadPerSec']) ?? 0,
    angularAccelRadPerSec2: numberFromKeys(value, ['alpha', 'angularAcceleration', 'angularAccelRadPerSec2']) ?? 0,
  };
}

function headingFromObject(value: Record<string, unknown>): number {
  const rotation = isObject(value.rotation) ? value.rotation : value;
  const radians = numberFromKeys(rotation, ['radians', 'headingRad', 'rotationRad']);
  if (radians !== null) return radians;
  return degreesToRadians(numberFromKeys(rotation, ['degrees', 'heading', 'headingDeg', 'rotationDegrees']) ?? 0);
}

function cubicBezier(p0: Point, p1: Point, p2: Point, p3: Point, t: number): Point {
  const u = 1 - t;
  return {
    x: (u ** 3 * p0.x) + (3 * u * u * t * p1.x) + (3 * u * t * t * p2.x) + (t ** 3 * p3.x),
    y: (u ** 3 * p0.y) + (3 * u * u * t * p1.y) + (3 * u * t * t * p2.y) + (t ** 3 * p3.y),
  };
}

function tangentAt(points: Point[], index: number): Point {
  const previous = points[Math.max(0, index - 1)];
  const next = points[Math.min(points.length - 1, index + 1)];
  const dx = next.x - previous.x;
  const dy = next.y - previous.y;
  const length = Math.hypot(dx, dy);
  return length > 1e-9 ? { x: dx / length, y: dy / length } : { x: 1, y: 0 };
}

function isObject(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null;
}

function isPoint(value: unknown): value is Point {
  return isObject(value)
    && typeof value.x === 'number'
    && typeof value.y === 'number';
}

function numberProp(value: Record<string, unknown>, key: string, fallback: number): number {
  return typeof value[key] === 'number' ? value[key] : fallback;
}

function numberFromKeys(value: Record<string, unknown>, keys: string[]): number | null {
  for (const key of keys) {
    if (typeof value[key] === 'number') return value[key];
  }
  return null;
}

function distance(a: Point, b: Point): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function lerp(a: number, b: number, alpha: number): number {
  return a + ((b - a) * alpha);
}

function interpolateAngle(a: number, b: number, alpha: number): number {
  return a + (angleDelta(a, b) * alpha);
}

function angleDelta(a: number, b: number): number {
  return Math.atan2(Math.sin(b - a), Math.cos(b - a));
}

function degreesToRadians(degrees: number): number {
  return (degrees * Math.PI) / 180.0;
}

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}
