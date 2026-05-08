import { describe, expect, test } from 'vitest';
import {
  driveLimits,
  field,
  isReachablePose,
  robotGeometry,
  shooterFieldPosition,
  type Pose2d,
} from '../math/field';
import { createDriveSimState, stepDriveSimulator } from '../math/simulator';

describe('field geometry and keyboard simulator', () => {
  test('applies the real shooter pivot offset at zero heading', () => {
    const pose: Pose2d = { x: 2.0, y: 4.0, headingRad: 0.0 };
    const shooter = shooterFieldPosition(pose);
    expect(shooter.x).toBeCloseTo(pose.x + robotGeometry.shooterOffset.x);
    expect(shooter.y).toBeCloseTo(pose.y + robotGeometry.shooterOffset.y);
  });

  test('rejects a robot footprint centered on the physical hub', () => {
    expect(isReachablePose({
      x: field.blueHubCenter.x,
      y: field.blueHubCenter.y,
      headingRad: 0,
    })).toBe(false);
  });

  test('keyboard simulator clamps acceleration and top speed', () => {
    let state = createDriveSimState({ x: 2.0, y: field.widthMeters / 2.0, headingRad: 0.0 });
    const keyboard = { keys: new Set(['w']) };
    state = stepDriveSimulator(state, keyboard, 0.02);
    expect(Math.hypot(state.velocity.x, state.velocity.y)).toBeLessThanOrEqual((driveLimits.maxLinearAccelMps2 * 0.02) + 1e-9);

    for (let i = 0; i < 200; i += 1) {
      state = stepDriveSimulator(state, keyboard, 0.02);
    }
    expect(Math.hypot(state.velocity.x, state.velocity.y)).toBeLessThanOrEqual(driveLimits.maxFreeSpeedMps + 1e-9);
  });
});
