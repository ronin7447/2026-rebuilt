import { describe, expect, test } from 'vitest';
import { parsePathFile, sampleLoadedPath } from '../math/pathPlayback';

describe('path playback parser', () => {
  test('loads a PathPlanner path and derives motion samples with acceleration', () => {
    const path = parsePathFile('centerline.path', JSON.stringify({
      waypoints: [
        {
          anchor: { x: 2.0, y: 4.0 },
          nextControl: { x: 3.0, y: 4.0 },
        },
        {
          prevControl: { x: 4.0, y: 4.5 },
          anchor: { x: 5.0, y: 4.5 },
        },
      ],
      globalConstraints: {
        maxVelocity: 2.0,
        maxAcceleration: 1.5,
      },
      idealStartingState: {
        velocity: 0,
        rotation: 0,
      },
      goalEndState: {
        velocity: 0,
        rotation: 90,
      },
    }));

    expect(path.source).toBe('PathPlanner');
    expect(path.samples.length).toBeGreaterThan(10);
    expect(path.totalTimeSec).toBeGreaterThan(0);
    const midpoint = sampleLoadedPath(path, path.totalTimeSec / 2.0);
    expect(midpoint.pose.x).toBeGreaterThan(2.0);
    expect(midpoint.pose.x).toBeLessThan(5.0);
    expect(Number.isFinite(midpoint.acceleration.x)).toBe(true);
    expect(Number.isFinite(midpoint.angularAccelRadPerSec2)).toBe(true);
  });
});
