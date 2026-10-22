import { describe, expect, test } from 'vitest';
import {
  defaultTurretConstants,
  fieldVectorToRobotFrame,
  getNearestEquivalentLegalTurretRotation,
  solveTurretShot,
  type TurretShotInput,
} from '../math/turret';

const baseInput: TurretShotInput = {
  shooterPoint: { x: 0, y: 0 },
  targetPoint: { x: 4, y: 0 },
  predictedRobotPose: { x: 0, y: 0, headingRad: 0 },
  tangentialVelocityMps: 0,
  launchAngleDeg: 45,
  exitSpeedMps: 8,
  pitchDeg: 0,
  rollDeg: 0,
  turretZeroOffsetDeg: 0,
  currentTurretAngleDeg: 0,
};

describe('turret shot solver', () => {
  test('zero tangential velocity produces no lead from stationary yaw', () => {
    const solution = solveTurretShot(baseInput);
    expect(solution.valid).toBe(true);
    expect(solution.leadYawDeg).toBeCloseTo(0);
    expect(solution.desiredFieldYawDeg).toBeCloseTo(solution.stationaryFieldYawDeg);
    expect(solution.desiredRawTurretYawDeg).toBeCloseTo(0);
  });

  test('positive tangential velocity requires negative lead like Java RobustShotModel', () => {
    const solution = solveTurretShot({
      ...baseInput,
      tangentialVelocityMps: 1.0,
    });
    expect(solution.valid).toBe(true);
    expect(solution.leadYawDeg).toBeLessThan(0);
    expect(solution.desiredFieldYawDeg).toBeLessThan(solution.stationaryFieldYawDeg);
  });

  test('acceleration affects turret lead through predicted release velocity', () => {
    const latencySec = 0.20;
    const tangentialVelocityAtRelease = 5.0 * latencySec;
    const stationary = solveTurretShot(baseInput);
    const accelerating = solveTurretShot({
      ...baseInput,
      tangentialVelocityMps: tangentialVelocityAtRelease,
    });
    expect(accelerating.valid).toBe(true);
    expect(accelerating.desiredFieldYawDeg).not.toBeCloseTo(stationary.desiredFieldYawDeg);
  });

  test('zero pitch and roll frame transform matches simple yaw subtraction', () => {
    const solution = solveTurretShot({
      ...baseInput,
      predictedRobotPose: { x: 0, y: 0, headingRad: Math.PI / 6 },
    });
    expect(solution.valid).toBe(true);
    expect(solution.desiredRawTurretYawDeg).toBeCloseTo(-30);

    const robotVector = fieldVectorToRobotFrame(
      { x: 1, y: 0, z: 0 },
      { rollRad: 0, pitchRad: 0, yawRad: Math.PI / 6 },
    );
    expect(Math.atan2(robotVector.y, robotVector.x) * 180 / Math.PI).toBeCloseTo(-30);
  });

  test('soft-limit wrapping matches representative Java TurretMath cases', () => {
    expect(getNearestEquivalentLegalTurretRotation(380 / 360, 0, 0, 1)).toBeCloseTo(20 / 360);
    expect(getNearestEquivalentLegalTurretRotation(-20 / 360, 0, 0, 1)).toBeCloseTo(340 / 360);
    expect(getNearestEquivalentLegalTurretRotation(720 / 360, 350 / 360, 0, 1)).toBeCloseTo(1);
    expect(getNearestEquivalentLegalTurretRotation(
      220 / 360,
      0,
      defaultTurretConstants.softLimitLowRot,
      defaultTurretConstants.softLimitHighRot,
    )).toBeCloseTo(-140 / 360);
  });

  test('rejects tangential velocity that exceeds horizontal launch speed', () => {
    const solution = solveTurretShot({
      ...baseInput,
      launchAngleDeg: 80,
      exitSpeedMps: 4,
      tangentialVelocityMps: 1.0,
    });
    expect(solution.valid).toBe(false);
    expect(solution.reason).toBe('tangential-speed-too-high');
  });
});
