import type { Point, Pose2d } from './field';

export interface TurretConstants {
  motorRotationsPerTurretRotation: number;
  softLimitLowRot: number;
  softLimitHighRot: number;
}

export interface TurretShotInput {
  shooterPoint: Point;
  targetPoint: Point;
  predictedRobotPose: Pose2d;
  tangentialVelocityMps: number;
  launchAngleDeg: number | null;
  exitSpeedMps: number | null;
  pitchDeg: number;
  rollDeg: number;
  turretZeroOffsetDeg: number;
  currentTurretAngleDeg: number;
  constants?: TurretConstants;
}

export interface TurretSolution {
  valid: boolean;
  reason: string;
  stationaryFieldYawDeg: number;
  desiredFieldYawDeg: number;
  desiredRawTurretYawDeg: number;
  legalGoalTurretYawDeg: number;
  legalGoalFieldYawDeg: number;
  leadYawDeg: number;
  localLaunchPitchDeg: number;
  launchVector: Vector3;
  isClamped: boolean;
}

export interface TurretControllerConfig {
  kP: number;
  kI: number;
  kD: number;
  kS: number;
  kV: number;
  kA: number;
  motorRotationsPerTurretRotation: number;
  maxVelocityDegPerSec: number;
  maxAccelerationDegPerSec2: number;
  maxJerkDegPerSec3: number;
  effectiveMotionScale: number;
  aimToleranceDeg: number;
  voltageLimit: number;
  supplyCurrentLimitA: number;
  statorCurrentLimitA: number;
}

export interface TurretSimState {
  angleDeg: number;
  velocityDegPerSec: number;
  accelerationDegPerSec2: number;
  integralErrorTurretRotSec: number;
  previousErrorDeg: number;
  previousGoalDeg: number;
  previousGoalVelocityDegPerSec: number;
  goalDeg: number;
  voltage: number;
  desiredGoalVelocityDegPerSec: number;
  desiredGoalAccelerationDegPerSec2: number;
  estimatedCurrentA: number;
  aimErrorDeg: number;
  timeToReadySec: number;
  atGoal: boolean;
  trackingRiskReason: string;
}

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

const EPSILON = 1e-9;
const FULL_TURRET_ROTATION = 1.0;

export const defaultTurretConstants: TurretConstants = {
  motorRotationsPerTurretRotation: 4.0,
  softLimitLowRot: Math.min((-1.90 * -1.0) / 4.0, (4.235 * -1.0) / 4.0),
  softLimitHighRot: Math.max((-1.90 * -1.0) / 4.0, (4.235 * -1.0) / 4.0),
};

export const defaultTurretControllerConfig: TurretControllerConfig = {
  kP: 5.75,
  kI: 1.075,
  kD: 0.50,
  kS: 0.15,
  kV: 0.12,
  kA: 0.01,
  motorRotationsPerTurretRotation: defaultTurretConstants.motorRotationsPerTurretRotation,
  maxVelocityDegPerSec: 22.5 * 360.0,
  maxAccelerationDegPerSec2: 175.0 * 360.0,
  maxJerkDegPerSec3: 1000.0 * 360.0,
  effectiveMotionScale: 0.45,
  aimToleranceDeg: 0.5,
  voltageLimit: 12.0,
  supplyCurrentLimitA: 20.0,
  statorCurrentLimitA: 50.0,
};

export function solveTurretShot(input: TurretShotInput): TurretSolution {
  const constants = input.constants ?? defaultTurretConstants;
  const dx = input.targetPoint.x - input.shooterPoint.x;
  const dy = input.targetPoint.y - input.shooterPoint.y;
  const distanceMeters = Math.hypot(dx, dy);
  const stationaryFieldYawRad = distanceMeters > EPSILON ? Math.atan2(dy, dx) : 0.0;
  const stationaryTurretYawDeg = wrapDegrees(
    radiansToDegrees(stationaryFieldYawRad)
      - radiansToDegrees(input.predictedRobotPose.headingRad)
      - input.turretZeroOffsetDeg,
  );

  if (
    input.launchAngleDeg === null
    || input.exitSpeedMps === null
    || !Number.isFinite(input.launchAngleDeg)
    || !Number.isFinite(input.exitSpeedMps)
    || input.exitSpeedMps <= 0
  ) {
    return wrapSolutionGoal({
      valid: false,
      reason: 'missing-shot-command',
      stationaryFieldYawDeg: radiansToDegrees(stationaryFieldYawRad),
      desiredFieldYawDeg: radiansToDegrees(stationaryFieldYawRad),
      desiredRawTurretYawDeg: stationaryTurretYawDeg,
      leadYawDeg: 0.0,
      localLaunchPitchDeg: 0.0,
      launchVector: {
        x: Math.cos(stationaryFieldYawRad),
        y: Math.sin(stationaryFieldYawRad),
        z: 0.0,
      },
    }, input.currentTurretAngleDeg, input.predictedRobotPose.headingRad, input.turretZeroOffsetDeg, constants);
  }

  const launchAngleRad = degreesToRadians(input.launchAngleDeg);
  const horizontalLaunchSpeed = input.exitSpeedMps * Math.cos(launchAngleRad);
  const verticalLaunchSpeed = input.exitSpeedMps * Math.sin(launchAngleRad);
  const requiredTangentialLaunchSpeed = -input.tangentialVelocityMps;
  if (distanceMeters <= EPSILON) {
    return invalidWithStationaryGoal('target-too-close', input, stationaryFieldYawRad, constants);
  }
  if (Math.abs(requiredTangentialLaunchSpeed) >= horizontalLaunchSpeed) {
    return invalidWithStationaryGoal('tangential-speed-too-high', input, stationaryFieldYawRad, constants);
  }

  const ux = dx / distanceMeters;
  const uy = dy / distanceMeters;
  const radialLaunchSpeed = Math.sqrt(
    (horizontalLaunchSpeed * horizontalLaunchSpeed)
      - (requiredTangentialLaunchSpeed * requiredTangentialLaunchSpeed),
  );
  const relativeHorizontalLaunchVelocity = {
    x: (ux * radialLaunchSpeed) + (-uy * requiredTangentialLaunchSpeed),
    y: (uy * radialLaunchSpeed) + (ux * requiredTangentialLaunchSpeed),
  };
  const launchVector = {
    x: relativeHorizontalLaunchVelocity.x,
    y: relativeHorizontalLaunchVelocity.y,
    z: verticalLaunchSpeed,
  };
  const robotVector = fieldVectorToRobotFrame(launchVector, {
    yawRad: input.predictedRobotPose.headingRad,
    pitchRad: degreesToRadians(input.pitchDeg),
    rollRad: degreesToRadians(input.rollDeg),
  });
  const robotHorizontalNorm = Math.hypot(robotVector.x, robotVector.y);
  const desiredFieldYawRad = Math.atan2(relativeHorizontalLaunchVelocity.y, relativeHorizontalLaunchVelocity.x);
  const desiredRawTurretYawDeg = wrapDegrees(
    radiansToDegrees(Math.atan2(robotVector.y, robotVector.x))
      - input.turretZeroOffsetDeg,
  );
  const localLaunchPitchDeg = radiansToDegrees(Math.atan2(robotVector.z, robotHorizontalNorm));
  const leadYawDeg = wrapDegrees(radiansToDegrees(desiredFieldYawRad - stationaryFieldYawRad));

  return wrapSolutionGoal({
    valid: true,
    reason: 'ready',
    stationaryFieldYawDeg: radiansToDegrees(stationaryFieldYawRad),
    desiredFieldYawDeg: radiansToDegrees(desiredFieldYawRad),
    desiredRawTurretYawDeg,
    leadYawDeg,
    localLaunchPitchDeg,
    launchVector,
  }, input.currentTurretAngleDeg, input.predictedRobotPose.headingRad, input.turretZeroOffsetDeg, constants);
}

export function createTurretSimState(angleDeg = 0.0): TurretSimState {
  return {
    angleDeg,
    velocityDegPerSec: 0.0,
    accelerationDegPerSec2: 0.0,
    integralErrorTurretRotSec: 0.0,
    previousErrorDeg: 0.0,
    previousGoalDeg: angleDeg,
    previousGoalVelocityDegPerSec: 0.0,
    goalDeg: angleDeg,
    voltage: 0.0,
    desiredGoalVelocityDegPerSec: 0.0,
    desiredGoalAccelerationDegPerSec2: 0.0,
    estimatedCurrentA: 0.0,
    aimErrorDeg: 0.0,
    timeToReadySec: 0.0,
    atGoal: true,
    trackingRiskReason: '',
  };
}

export function stepTurretSimulator(
  state: TurretSimState,
  goalDeg: number,
  dtSec: number,
  config: TurretControllerConfig,
): TurretSimState {
  const dt = Math.max(0.0, Math.min(dtSec, 0.05));
  if (dt <= EPSILON) return { ...state, goalDeg };
  const motionScale = clamp(config.effectiveMotionScale, 0.05, 1.0);
  const supplyCurrentLimitA = Math.max(0.1, config.supplyCurrentLimitA);
  const statorCurrentLimitA = Math.max(supplyCurrentLimitA, config.statorCurrentLimitA);
  const currentTorqueScale = clamp(supplyCurrentLimitA / statorCurrentLimitA, 0.12, 1.0);
  const effectiveMaxVelocityDegPerSec = Math.max(1.0, config.maxVelocityDegPerSec * motionScale);
  const effectiveMaxAccelerationDegPerSec2 = Math.max(
    1.0,
    config.maxAccelerationDegPerSec2 * motionScale * currentTorqueScale,
  );
  const effectiveMaxJerkDegPerSec3 = Math.max(
    1.0,
    config.maxJerkDegPerSec3 * motionScale * currentTorqueScale,
  );

  const desiredGoalVelocityDegPerSec = (goalDeg - state.previousGoalDeg) / dt;
  const desiredGoalAccelerationDegPerSec2 = (
    desiredGoalVelocityDegPerSec - state.previousGoalVelocityDegPerSec
  ) / dt;

  const errorDeg = goalDeg - state.angleDeg;
  const integralErrorTurretRotSec = state.integralErrorTurretRotSec + (degreesToRotations(errorDeg) * dt);
  const errorDerivativeDegPerSec = (errorDeg - state.previousErrorDeg) / dt;
  const errorMotorRot = degreesToRotations(errorDeg) * config.motorRotationsPerTurretRotation;
  const integralErrorMotorRotSec = integralErrorTurretRotSec * config.motorRotationsPerTurretRotation;
  const derivativeMotorRps = degreesToRotations(errorDerivativeDegPerSec)
    * config.motorRotationsPerTurretRotation;
  const feedforwardVelocityMotorRps = degreesToRotations(
    clamp(desiredGoalVelocityDegPerSec, -effectiveMaxVelocityDegPerSec, effectiveMaxVelocityDegPerSec),
  ) * config.motorRotationsPerTurretRotation;
  const feedforwardAccelMotorRps2 = degreesToRotations(
    clamp(
      desiredGoalAccelerationDegPerSec2,
      -effectiveMaxAccelerationDegPerSec2,
      effectiveMaxAccelerationDegPerSec2,
    ),
  ) * config.motorRotationsPerTurretRotation;

  const direction = Math.abs(feedforwardVelocityMotorRps) > EPSILON
    ? Math.sign(feedforwardVelocityMotorRps)
    : Math.sign(errorMotorRot);
  const feedforwardVoltage = (direction === 0 ? 0.0 : config.kS * direction)
    + (config.kV * feedforwardVelocityMotorRps)
    + (config.kA * feedforwardAccelMotorRps2);
  const feedbackVoltage = (config.kP * errorMotorRot)
    + (config.kI * integralErrorMotorRotSec)
    + (config.kD * derivativeMotorRps);
  const voltageLimit = Math.max(0.1, Math.abs(config.voltageLimit));
  const voltage = clamp(feedbackVoltage + feedforwardVoltage, -voltageLimit, voltageLimit);
  const estimatedCurrentA = clamp(
    (Math.abs(voltage) / voltageLimit) * statorCurrentLimitA,
    0.0,
    statorCurrentLimitA,
  );
  const currentLimitedVoltageScale = clamp(
    supplyCurrentLimitA / Math.max(estimatedCurrentA, supplyCurrentLimitA),
    currentTorqueScale,
    1.0,
  );
  const requestedAccel = Math.sign(voltage) * effectiveMaxAccelerationDegPerSec2
    * clamp(Math.abs(voltage) / voltageLimit, 0.0, 1.0);
  const nextAcceleration = slewScalar(
    state.accelerationDegPerSec2,
    requestedAccel * currentLimitedVoltageScale,
    effectiveMaxJerkDegPerSec3 * dt,
  );
  let nextVelocity = clamp(
    state.velocityDegPerSec + (nextAcceleration * dt),
    -effectiveMaxVelocityDegPerSec,
    effectiveMaxVelocityDegPerSec,
  );
  let nextAngle = state.angleDeg + (nextVelocity * dt);
  const nextError = goalDeg - nextAngle;
  if (Math.sign(errorDeg) !== 0 && Math.sign(errorDeg) !== Math.sign(nextError)) {
    nextAngle = goalDeg;
    nextVelocity = 0.0;
  }

  const aimErrorDeg = goalDeg - nextAngle;
  const atGoal = Math.abs(aimErrorDeg) <= config.aimToleranceDeg
    && Math.abs(nextVelocity) <= Math.max(1.0, config.aimToleranceDeg * 2.0);
  return {
    angleDeg: nextAngle,
    velocityDegPerSec: nextVelocity,
    accelerationDegPerSec2: nextAcceleration,
    integralErrorTurretRotSec,
    previousErrorDeg: errorDeg,
    previousGoalDeg: goalDeg,
    previousGoalVelocityDegPerSec: desiredGoalVelocityDegPerSec,
    goalDeg,
    voltage,
    desiredGoalVelocityDegPerSec,
    desiredGoalAccelerationDegPerSec2,
    estimatedCurrentA,
    aimErrorDeg,
    timeToReadySec: estimateTimeToReadySec(aimErrorDeg, nextVelocity, config),
    atGoal,
    trackingRiskReason: trackingRiskReason(
      desiredGoalVelocityDegPerSec,
      desiredGoalAccelerationDegPerSec2,
      config,
      effectiveMaxVelocityDegPerSec,
      effectiveMaxAccelerationDegPerSec2,
      estimatedCurrentA,
    ),
  };
}

export function getNearestEquivalentLegalTurretRotation(
  desiredTurretRotations: number,
  currentTurretRotations: number,
  legalLowTurretRotations: number,
  legalHighTurretRotations: number,
): number {
  const minTurns = Math.ceil(legalLowTurretRotations - desiredTurretRotations - EPSILON);
  const maxTurns = Math.floor(legalHighTurretRotations - desiredTurretRotations + EPSILON);
  let bestCandidate = Number.NaN;
  let bestDistance = Number.POSITIVE_INFINITY;

  for (let turns = minTurns; turns <= maxTurns; turns += 1) {
    const candidate = desiredTurretRotations + (turns * FULL_TURRET_ROTATION);
    const distance = Math.abs(candidate - currentTurretRotations);
    if (distance + EPSILON < bestDistance) {
      bestCandidate = candidate;
      bestDistance = distance;
    }
  }

  if (!Number.isNaN(bestCandidate)) return bestCandidate;
  const nearestEquivalent = desiredTurretRotations
    + (Math.round((currentTurretRotations - desiredTurretRotations) / FULL_TURRET_ROTATION)
      * FULL_TURRET_ROTATION);
  return clamp(nearestEquivalent, legalLowTurretRotations, legalHighTurretRotations);
}

export function fieldVectorToRobotFrame(
  fieldVector: Vector3,
  orientation: { rollRad: number; pitchRad: number; yawRad: number },
): Vector3 {
  const yawRemoved = rotateZ(fieldVector, -orientation.yawRad);
  const pitchRemoved = rotateY(yawRemoved, -orientation.pitchRad);
  return rotateX(pitchRemoved, -orientation.rollRad);
}

function invalidWithStationaryGoal(
  reason: string,
  input: TurretShotInput,
  stationaryFieldYawRad: number,
  constants: TurretConstants,
): TurretSolution {
  const stationaryTurretYawDeg = wrapDegrees(
    radiansToDegrees(stationaryFieldYawRad)
      - radiansToDegrees(input.predictedRobotPose.headingRad)
      - input.turretZeroOffsetDeg,
  );
  return wrapSolutionGoal({
    valid: false,
    reason,
    stationaryFieldYawDeg: radiansToDegrees(stationaryFieldYawRad),
    desiredFieldYawDeg: radiansToDegrees(stationaryFieldYawRad),
    desiredRawTurretYawDeg: stationaryTurretYawDeg,
    leadYawDeg: 0.0,
    localLaunchPitchDeg: 0.0,
    launchVector: {
      x: Math.cos(stationaryFieldYawRad),
      y: Math.sin(stationaryFieldYawRad),
      z: 0.0,
    },
  }, input.currentTurretAngleDeg, input.predictedRobotPose.headingRad, input.turretZeroOffsetDeg, constants);
}

function wrapSolutionGoal(
  partial: Omit<TurretSolution, 'legalGoalTurretYawDeg' | 'legalGoalFieldYawDeg' | 'isClamped'>,
  currentTurretAngleDeg: number,
  robotHeadingRad: number,
  turretZeroOffsetDeg: number,
  constants: TurretConstants,
): TurretSolution {
  const desiredRot = degreesToRotations(partial.desiredRawTurretYawDeg);
  const currentRot = degreesToRotations(currentTurretAngleDeg);
  const legalGoalRot = getNearestEquivalentLegalTurretRotation(
    desiredRot,
    currentRot,
    constants.softLimitLowRot,
    constants.softLimitHighRot,
  );
  const legalGoalTurretYawDeg = rotationsToDegrees(legalGoalRot);
  const isClamped = Math.abs(wrapDegrees(legalGoalTurretYawDeg - partial.desiredRawTurretYawDeg)) > 1e-3;
  return {
    ...partial,
    valid: partial.valid && !isClamped,
    reason: isClamped ? 'turret-goal-clamped-by-soft-limit' : partial.reason,
    legalGoalTurretYawDeg,
    legalGoalFieldYawDeg: wrapDegrees(
      radiansToDegrees(robotHeadingRad) + legalGoalTurretYawDeg + turretZeroOffsetDeg,
    ),
    isClamped,
  };
}

function rotateX(vector: Vector3, angleRad: number): Vector3 {
  const cos = Math.cos(angleRad);
  const sin = Math.sin(angleRad);
  return {
    x: vector.x,
    y: (vector.y * cos) - (vector.z * sin),
    z: (vector.y * sin) + (vector.z * cos),
  };
}

function rotateY(vector: Vector3, angleRad: number): Vector3 {
  const cos = Math.cos(angleRad);
  const sin = Math.sin(angleRad);
  return {
    x: (vector.x * cos) + (vector.z * sin),
    y: vector.y,
    z: (-vector.x * sin) + (vector.z * cos),
  };
}

function rotateZ(vector: Vector3, angleRad: number): Vector3 {
  const cos = Math.cos(angleRad);
  const sin = Math.sin(angleRad);
  return {
    x: (vector.x * cos) - (vector.y * sin),
    y: (vector.x * sin) + (vector.y * cos),
    z: vector.z,
  };
}

function estimateTimeToReadySec(
  aimErrorDeg: number,
  velocityDegPerSec: number,
  config: TurretControllerConfig,
): number {
  const remaining = Math.max(0.0, Math.abs(aimErrorDeg) - config.aimToleranceDeg);
  if (remaining <= EPSILON) return 0.0;
  const usableVelocity = Math.max(
    Math.abs(velocityDegPerSec),
    config.maxVelocityDegPerSec * clamp(config.effectiveMotionScale, 0.05, 1.0) * 0.35,
    1.0,
  );
  return remaining / usableVelocity;
}

function trackingRiskReason(
  desiredGoalVelocityDegPerSec: number,
  desiredGoalAccelerationDegPerSec2: number,
  config: TurretControllerConfig,
  effectiveMaxVelocityDegPerSec: number,
  effectiveMaxAccelerationDegPerSec2: number,
  estimatedCurrentA: number,
): string {
  if (estimatedCurrentA > config.supplyCurrentLimitA * 1.15) {
    return 'estimated current demand is above Java supply-current limit';
  }
  if (Math.abs(desiredGoalVelocityDegPerSec) > effectiveMaxVelocityDegPerSec * 0.90) {
    return 'desired turret goal velocity near/exceeds configured max';
  }
  if (Math.abs(desiredGoalAccelerationDegPerSec2) > effectiveMaxAccelerationDegPerSec2 * 0.90) {
    return 'desired turret goal acceleration near/exceeds configured max';
  }
  return '';
}

function slewScalar(current: number, target: number, maxDelta: number): number {
  return current + clamp(target - current, -maxDelta, maxDelta);
}

function degreesToRotations(degrees: number): number {
  return degrees / 360.0;
}

function rotationsToDegrees(rotations: number): number {
  return rotations * 360.0;
}

function degreesToRadians(degrees: number): number {
  return (degrees * Math.PI) / 180.0;
}

function radiansToDegrees(radians: number): number {
  return (radians * 180.0) / Math.PI;
}

function wrapDegrees(degrees: number): number {
  return radiansToDegrees(Math.atan2(Math.sin(degreesToRadians(degrees)), Math.cos(degreesToRadians(degrees))));
}

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}
