import { describe, expect, test } from 'vitest';
import fs from 'node:fs';
import { defaultConfig } from '../math/trajectory';
import { passbackConfig, robotGeometry } from '../math/field';
import { evaluateFinalTrajectory, evaluateReferenceShot, type ReferenceShotInput } from '../math/evaluator';

const base: ReferenceShotInput = {
  pose: { x: 1.0, y: 4.0, headingRad: 0 }, velocity: { x: 0, y: 0 }, acceleration: { x: 0, y: 0 },
  omegaRadPerSec: 0, angularAccelRadPerSec2: 0, releaseLatencySec: 0.14,
  shooterOffset: robotGeometry.shooterOffset, targetPoint: { x: 4.02844, y: 4.03479 }, config: defaultConfig,
  usesPolynomial: true, candidateSelection: 'robust', robotPitchDeg: 0, robotRollDeg: 0,
  turretZeroOffsetDeg: 0, currentTurretAngleDeg: 0,
};

describe('pure reference evaluator', () => {
  test('uses one target-plane crossing time for the final compensated trajectory', () => {
    const result = evaluateReferenceShot(base);
    expect(result.commandSource).toBe('generated-polynomial');
    expect(result.targetPlaneCrossingTimeSec).not.toBeNull();
    expect(result.targetPlaneCrossingTimeSec!).toBeGreaterThan(0);
  });

  test('lateral compensation remains geometrically evaluated', () => {
    const result = evaluateReferenceShot({ ...base, velocity: { x: 0, y: 1.5 } });
    expect(result.turret).not.toBeNull();
    expect(Math.abs(result.turret!.leadYawDeg)).toBeGreaterThan(1);
    expect(typeof result.feasible).toBe('boolean');
  });

  test('supports flat-fast passback without generated coefficients', () => {
    const result = evaluateReferenceShot({ ...base, pose: { x: 6, y: 4, headingRad: 0 },
      targetPoint: { x: 0.48, y: 6 }, config: passbackConfig, usesPolynomial: false,
      candidateSelection: 'flat-fast' });
    expect(result.commandSource).toBe('direct-candidate');
    expect(result.targetPlaneCrossingTimeSec).not.toBeNull();
  });

  test('rejects nonfinite control state', () => {
    expect(evaluateReferenceShot({ ...base, omegaRadPerSec: Number.NaN }).reason).toBe('nonfinite-input');
  });

  test('passback selects a compensated shot rather than retaining a radial-only speed', () => {
    for (const distance of [2, 4, 6, 8]) {
      for (const lateral of [-2, 2]) {
        const result = evaluateReferenceShot({ ...base,
          pose: { x: 0, y: 0, headingRad: 0 }, shooterOffset: { x: 0, y: 0 },
          velocity: { x: 0, y: lateral }, releaseLatencySec: 0,
          targetPoint: { x: distance, y: 0 }, config: passbackConfig,
          usesPolynomial: false, candidateSelection: 'flat-fast' });
        expect(result.feasible, `distance=${distance}, lateral=${lateral}: ${result.reason}`).toBe(true);
        const launch = result.turret!.launchVector;
        const time = distance / launch.x;
        const height = passbackConfig.shooterExitHeightMeters + launch.z * time - 0.5 * 9.81 * time ** 2;
        expect(Math.abs(height - 0.5)).toBeLessThanOrEqual(0.24);
        expect((launch.y + lateral) * time).toBeCloseTo(0, 9);
        expect(result.targetPlaneCrossingTimeSec).toBeCloseTo(time, 9);
        expect(Math.abs(result.rollerRps!)).toBeLessThanOrEqual(100);
      }
    }
  });

  test('matches an independently calculated ballistic target-plane sample', () => {
    const radialSpeed = 5.0;
    const verticalSpeed = 4.0;
    const distanceMeters = 4.0;
    const expectedTime = distanceMeters / radialSpeed;
    const expectedHeight = passbackConfig.shooterExitHeightMeters
      + verticalSpeed * expectedTime
      - 0.5 * passbackConfig.gravityMps2 * expectedTime * expectedTime;
    const result = evaluateFinalTrajectory(passbackConfig, distanceMeters, radialSpeed, 0, verticalSpeed);
    expect(result.targetPlaneCrossingTimeSec).toBeCloseTo(expectedTime, 12);
    expect(result.impactHeightMeters).toBeCloseTo(expectedHeight, 12);
  });

  test('matches the versioned cross-language fixtures', () => {
    const fixturePath = new URL('../../../../shared/shot-evaluator-fixtures-v1.csv', import.meta.url);
    const lines = fs.readFileSync(fixturePath, 'utf8').trim().split(/\r?\n/).slice(1);
    for (const line of lines) {
      const value = line.split(',');
      const isHub = value[2] === 'HUB';
      const result = evaluateReferenceShot({
        ...base,
        pose: { x: Number(value[3]), y: Number(value[4]), headingRad: Number(value[5]) * Math.PI / 180 },
        velocity: { x: Number(value[6]), y: Number(value[7]) },
        omegaRadPerSec: Number(value[8]),
        acceleration: { x: Number(value[9]), y: Number(value[10]) },
        angularAccelRadPerSec2: Number(value[11]),
        robotPitchDeg: Number(value[12]),
        robotRollDeg: Number(value[13]),
        targetPoint: { x: Number(value[14]), y: Number(value[15]) },
        config: isHub ? defaultConfig : passbackConfig,
        usesPolynomial: isHub,
        candidateSelection: isHub ? 'robust' : 'flat-fast',
      });
      expect(result.feasible, value[1]).toBe(value[17] === 'true');
      expect(result.commandSource, value[1]).toBe(value[18].toLowerCase().replaceAll('_', '-'));
      if (Number.isFinite(Number(value[19]))) {
        expect(result.targetPlaneCrossingTimeSec, value[1]).toBeCloseTo(Number(value[19]), 5);
      }
    }
  });
});
