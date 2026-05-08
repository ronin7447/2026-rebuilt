import { describe, expect, test } from 'vitest';
import { defaultConfig, findBestCandidate, generateShotSamples, simulateCandidate } from '../math/trajectory';
import { evaluatePolynomial, fitShotMap } from '../math/polynomial';

describe('trajectory solver', () => {
  test('validates a scoring candidate through the goal band', () => {
    const candidate = findBestCandidate(defaultConfig, { distanceMeters: 4, radialVelocityMps: 0 });
    expect(candidate).not.toBeNull();
    expect(candidate?.heightMarginMeters).toBeGreaterThanOrEqual(0);
  });

  test('selects a robust shot for a normal moving input', () => {
    const candidate = findBestCandidate(defaultConfig, { distanceMeters: 5, radialVelocityMps: -0.75 });
    expect(candidate).not.toBeNull();
    expect(candidate?.robustnessMargin).toBeGreaterThan(0);
    expect(Math.abs(candidate!.rollerRps)).toBeLessThanOrEqual(defaultConfig.maxRollerRps);
  });

  test('rejects shots outside the generated distance domain', () => {
    const candidate = findBestCandidate(defaultConfig, { distanceMeters: 0.9, radialVelocityMps: 0 });
    expect(candidate).toBeNull();
  });

  test('rejects low entry shots blocked by the hub opening lip', () => {
    const candidate = simulateCandidate(defaultConfig, { distanceMeters: 4, radialVelocityMps: 0 }, -2.2, 10.0);
    expect(candidate).toBeNull();
  });

  test('rejects shots above the 100 rps roller cap', () => {
    const candidate = simulateCandidate(defaultConfig, { distanceMeters: 4, radialVelocityMps: 0 }, -1.2, 12.0);
    expect(candidate).toBeNull();
  });

  test('passback flattest selector prefers the lower launch angle', () => {
    const passbackConfig = {
      ...defaultConfig,
      targetHeightMeters: 0.5,
      goalRadiusMeters: 0.24,
      minDistanceMeters: 0.4,
      maxDistanceMeters: 12.0,
      requiresHubOpeningClearance: false,
    };
    const input = { distanceMeters: 4, radialVelocityMps: 0 };
    const robustCandidate = findBestCandidate(passbackConfig, input, 'robust');
    const flattestCandidate = findBestCandidate(passbackConfig, input, 'flattest');
    expect(robustCandidate).not.toBeNull();
    expect(flattestCandidate).not.toBeNull();
    expect(flattestCandidate!.launchAngleDeg).toBeLessThanOrEqual(robustCandidate!.launchAngleDeg);
  });

  test('passback flat-fast selector prefers shorter flight time than flattest selector', () => {
    const passbackConfig = {
      ...defaultConfig,
      targetHeightMeters: 0.5,
      goalRadiusMeters: 0.24,
      minDistanceMeters: 0.4,
      maxDistanceMeters: 12.0,
      requiresHubOpeningClearance: false,
    };
    const input = { distanceMeters: 4, radialVelocityMps: 0 };
    const flattestCandidate = findBestCandidate(passbackConfig, input, 'flattest');
    const flatFastCandidate = findBestCandidate(passbackConfig, input, 'flat-fast');
    expect(flattestCandidate).not.toBeNull();
    expect(flatFastCandidate).not.toBeNull();
    expect(flatFastCandidate!.flightTimeSec).toBeLessThanOrEqual(flattestCandidate!.flightTimeSec);
    expect(flatFastCandidate!.exitSpeedMps).toBeGreaterThanOrEqual(flattestCandidate!.exitSpeedMps);
  });

  test('fits polynomial coefficients with finite residuals', () => {
    const samples = generateShotSamples(defaultConfig);
    const fit = fitShotMap(samples);
    const sample = samples[Math.floor(samples.length / 2)];
    expect(samples.length).toBeGreaterThan(10);
    expect(Number.isFinite(evaluatePolynomial(fit.hoodCoefficients, sample.input))).toBe(true);
    expect(Number.isFinite(fit.maxRollerResidualRps)).toBe(true);
  });
});
