import { describe, expect, test } from 'vitest';
import {
  driveLimits,
  field,
  fieldElements,
  isBlockingElement,
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

  test('allows the robot to drive under trench low-clearance geometry', () => {
    const trench = fieldElements.find((element) => element.kind === 'low-clearance' && element.alliance === 'blue');
    expect(trench).toBeDefined();
    expect(isBlockingElement(trench!)).toBe(false);
    expect(isReachablePose({
      x: trench!.x + (trench!.width / 2),
      y: trench!.y + (trench!.height / 2),
      headingRad: 0,
    })).toBe(true);
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

  test('keyboard simulator uses passive friction when the driver releases controls', () => {
    let state = createDriveSimState({ x: 2.0, y: field.widthMeters / 2.0, headingRad: 0.0 });
    const keyboard = { keys: new Set(['w']) };
    for (let i = 0; i < 20; i += 1) {
      state = stepDriveSimulator(state, keyboard, 0.02);
    }

    const speedBefore = Math.hypot(state.velocity.x, state.velocity.y);
    keyboard.keys.clear();
    state = stepDriveSimulator(state, keyboard, 0.02, {
      passiveLinearDecelMps2: 6.0,
      passiveAngularDecelRadPerSec2: 8.0,
    });

    expect(Math.hypot(state.velocity.x, state.velocity.y)).toBeLessThan(speedBefore);
  });

  test('keyboard simulator can apply visual-only shoot-on-move caps', () => {
    let state = createDriveSimState({ x: 2.0, y: field.widthMeters / 2.0, headingRad: 0.0 });
    const keyboard = { keys: new Set(['w']) };

    for (let i = 0; i < 200; i += 1) {
      state = stepDriveSimulator(state, keyboard, 0.02, {
        passiveLinearDecelMps2: 3.0,
        passiveAngularDecelRadPerSec2: 8.0,
        maxFreeSpeedMps: 1.25,
        maxLinearAccelMps2: 1.0,
      });
    }

    expect(Math.hypot(state.velocity.x, state.velocity.y)).toBeLessThanOrEqual(1.25 + 1e-9);
  });
});
