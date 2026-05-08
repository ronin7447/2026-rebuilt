import {
  clampPoseInsideField,
  driveLimits,
  isReachablePose,
  type Point,
  type Pose2d,
} from './field';

export interface KeyboardState {
  keys: Set<string>;
}

export interface DriveSimState {
  pose: Pose2d;
  velocity: Point;
  omegaRadPerSec: number;
  acceleration: Point;
  angularAccelRadPerSec2: number;
}

export interface SimulatorOptions {
  passiveLinearDecelMps2: number;
  passiveAngularDecelRadPerSec2: number;
  maxFreeSpeedMps?: number;
  maxLinearAccelMps2?: number;
  maxAngularRateRadPerSec?: number;
  maxAngularAccelRadPerSec2?: number;
}

export function createDriveSimState(pose: Pose2d): DriveSimState {
  return {
    pose,
    velocity: { x: 0, y: 0 },
    omegaRadPerSec: 0,
    acceleration: { x: 0, y: 0 },
    angularAccelRadPerSec2: 0,
  };
}

export function stepDriveSimulator(
  state: DriveSimState,
  keyboard: KeyboardState,
  dtSec: number,
  options: SimulatorOptions = driveLimits,
): DriveSimState {
  const boundedDt = Math.max(0, Math.min(dtSec, 0.05));
  const maxFreeSpeedMps = options.maxFreeSpeedMps ?? driveLimits.maxFreeSpeedMps;
  const maxLinearAccelMps2 = options.maxLinearAccelMps2 ?? driveLimits.maxLinearAccelMps2;
  const maxAngularRateRadPerSec = options.maxAngularRateRadPerSec ?? driveLimits.maxAngularRateRadPerSec;
  const maxAngularAccelRadPerSec2 = options.maxAngularAccelRadPerSec2 ?? driveLimits.maxAngularAccelRadPerSec2;
  const targetVelocity = targetFieldVelocity(state.pose.headingRad, keyboard, maxFreeSpeedMps);
  const linearAccelLimit = Math.hypot(targetVelocity.x, targetVelocity.y) <= 1e-9
    ? options.passiveLinearDecelMps2
    : maxLinearAccelMps2;
  const nextVelocity = slewVector(
    state.velocity,
    clampVectorMagnitude(targetVelocity, maxFreeSpeedMps),
    linearAccelLimit * boundedDt,
  );
  const targetOmega = targetAngularRate(keyboard, maxAngularRateRadPerSec);
  const angularAccelLimit = Math.abs(targetOmega) <= 1e-9
    ? options.passiveAngularDecelRadPerSec2
    : maxAngularAccelRadPerSec2;
  const nextOmega = slewScalar(
    state.omegaRadPerSec,
    targetOmega,
    angularAccelLimit * boundedDt,
  );
  const nextPose = clampPoseInsideField({
    x: state.pose.x + (nextVelocity.x * boundedDt),
    y: state.pose.y + (nextVelocity.y * boundedDt),
    headingRad: wrapAngle(state.pose.headingRad + (nextOmega * boundedDt)),
  });

  if (!isReachablePose(nextPose)) {
    return {
      pose: state.pose,
      velocity: { x: 0, y: 0 },
      omegaRadPerSec: 0,
      acceleration: {
        x: -state.velocity.x / Math.max(boundedDt, 1e-9),
        y: -state.velocity.y / Math.max(boundedDt, 1e-9),
      },
      angularAccelRadPerSec2: -state.omegaRadPerSec / Math.max(boundedDt, 1e-9),
    };
  }

  return {
    pose: nextPose,
    velocity: nextVelocity,
    omegaRadPerSec: nextOmega,
    acceleration: {
      x: (nextVelocity.x - state.velocity.x) / Math.max(boundedDt, 1e-9),
      y: (nextVelocity.y - state.velocity.y) / Math.max(boundedDt, 1e-9),
    },
    angularAccelRadPerSec2: (nextOmega - state.omegaRadPerSec) / Math.max(boundedDt, 1e-9),
  };
}

function targetFieldVelocity(headingRad: number, keyboard: KeyboardState, maxFreeSpeedMps: number): Point {
  const fieldCommand = {
    x: axis(keyboard, 'w', 's'),
    y: axis(keyboard, 'a', 'd'),
  };
  const robotCommand = {
    x: axis(keyboard, 'i', 'k'),
    y: axis(keyboard, 'j', 'l'),
  };
  const cos = Math.cos(headingRad);
  const sin = Math.sin(headingRad);
  const robotCommandField = {
    x: (robotCommand.x * cos) - (robotCommand.y * sin),
    y: (robotCommand.x * sin) + (robotCommand.y * cos),
  };
  return clampVectorMagnitude({
    x: (fieldCommand.x + robotCommandField.x) * maxFreeSpeedMps,
    y: (fieldCommand.y + robotCommandField.y) * maxFreeSpeedMps,
  }, maxFreeSpeedMps);
}

function targetAngularRate(keyboard: KeyboardState, maxAngularRateRadPerSec: number): number {
  return axis(keyboard, 'q', 'e') * maxAngularRateRadPerSec;
}

function axis(keyboard: KeyboardState, positiveKey: string, negativeKey: string): number {
  return (keyboard.keys.has(positiveKey) ? 1 : 0) - (keyboard.keys.has(negativeKey) ? 1 : 0);
}

function slewVector(current: Point, target: Point, maxDelta: number): Point {
  const delta = { x: target.x - current.x, y: target.y - current.y };
  const limitedDelta = clampVectorMagnitude(delta, maxDelta);
  return { x: current.x + limitedDelta.x, y: current.y + limitedDelta.y };
}

function slewScalar(current: number, target: number, maxDelta: number): number {
  return current + Math.max(-maxDelta, Math.min(maxDelta, target - current));
}

function clampVectorMagnitude(vector: Point, maxMagnitude: number): Point {
  const magnitude = Math.hypot(vector.x, vector.y);
  if (magnitude <= maxMagnitude || magnitude <= 1e-9) return vector;
  const scale = maxMagnitude / magnitude;
  return { x: vector.x * scale, y: vector.y * scale };
}

function wrapAngle(angleRad: number): number {
  return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
}
