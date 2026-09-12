import type { ShotCalculatorConfig, ShotInput, TrajectoryCandidate } from './types';

export type CandidateSelection = 'robust' | 'flattest' | 'flat-fast';

export const defaultConfig: ShotCalculatorConfig = {
  gravityMps2: 9.81,
  shooterExitHeightMeters: 0.4064,
  targetHeightMeters: 2.13,
  goalRadiusMeters: 0.28,
  fuelRadiusMeters: (5.91 * 0.0254) / 2.0,
  hubOpeningDepthMeters: 41.7 * 0.0254,
  hubFrontLipHeightMeters: 72.0 * 0.0254,
  hubLipClearanceMeters: 0.02,
  minEntryAngleDeg: 18.0,
  requiresHubOpeningClearance: true,
  minDistanceMeters: 1.5,
  maxDistanceMeters: 8.0,
  minRadialVelocityMps: -2.5,
  maxRadialVelocityMps: 2.5,
  minHoodRot: -2.266942,
  maxHoodRot: -0.631116,
  minExitSpeedMps: 4.0,
  maxExitSpeedMps: 11.120291,
  hoodSlope: 0.0857913,
  hoodIntercept: -7.49442,
  rollerDirectionSign: -1.0,
  rollerSlope: 8.01922,
  rollerIntercept: 10.82394,
  maxRollerRps: 100.0,
};

export const closeRangeHubConfig: ShotCalculatorConfig = {
  ...defaultConfig,
  minDistanceMeters: 0.85,
  maxDistanceMeters: defaultConfig.minDistanceMeters,
};

export function launchAngleDegForHood(config: ShotCalculatorConfig, hoodRot: number): number {
  return (hoodRot - config.hoodIntercept) / config.hoodSlope;
}

export function rollerRpsForExitSpeed(config: ShotCalculatorConfig, exitSpeedMps: number): number {
  return config.rollerDirectionSign * ((config.rollerSlope * exitSpeedMps) + config.rollerIntercept);
}

export function simulateCandidate(
  config: ShotCalculatorConfig,
  input: ShotInput,
  hoodRot: number,
  exitSpeedMps: number,
): TrajectoryCandidate | null {
  const launchAngleDeg = launchAngleDegForHood(config, hoodRot);
  const launchAngleRad = (launchAngleDeg * Math.PI) / 180.0;
  const horizontalLaunchSpeed = exitSpeedMps * Math.cos(launchAngleRad);
  const verticalLaunchSpeed = exitSpeedMps * Math.sin(launchAngleRad);
  const targetLineSpeed = horizontalLaunchSpeed + input.radialVelocityMps;
  if (targetLineSpeed <= 0.05) return null;

  const flightTimeSec = input.distanceMeters / targetLineSpeed;
  if (!Number.isFinite(flightTimeSec) || flightTimeSec <= 0 || flightTimeSec > 3.0) return null;

  const heightDelta = (verticalLaunchSpeed * flightTimeSec)
    - (0.5 * config.gravityMps2 * flightTimeSec * flightTimeSec);
  const impactHeightMeters = config.shooterExitHeightMeters + heightDelta;
  if (!clearsHubOpening(config, input.distanceMeters, targetLineSpeed, verticalLaunchSpeed)) return null;

  const rollerRps = rollerRpsForExitSpeed(config, exitSpeedMps);
  if (Math.abs(rollerRps) > config.maxRollerRps) return null;

  const heightError = Math.abs(impactHeightMeters - config.targetHeightMeters);
  const heightMarginMeters = config.goalRadiusMeters - heightError;
  if (heightMarginMeters < 0) return null;

  const timeToPeakSec = verticalLaunchSpeed / config.gravityMps2;
  const peakHeightMeters = config.shooterExitHeightMeters
    + (verticalLaunchSpeed * timeToPeakSec)
    - (0.5 * config.gravityMps2 * timeToPeakSec * timeToPeakSec);
  const robustnessMargin = scoreRobustness(config, input, hoodRot, exitSpeedMps, heightMarginMeters, flightTimeSec);

  return {
    input,
    hoodRot,
    launchAngleDeg,
    exitSpeedMps,
    rollerRps,
    flightTimeSec,
    peakHeightMeters,
    impactHeightMeters,
    heightMarginMeters,
    robustnessMargin,
  };
}

export function findBestCandidate(
  config: ShotCalculatorConfig,
  input: ShotInput,
  selection: CandidateSelection = 'robust',
): TrajectoryCandidate | null {
  if (!isInsideShotMapDomain(config, input)) return null;

  let best: TrajectoryCandidate | null = null;
  for (let hoodRot = config.minHoodRot; hoodRot <= config.maxHoodRot + 1e-9; hoodRot += 0.025) {
    for (let exitSpeedMps = config.minExitSpeedMps; exitSpeedMps <= config.maxExitSpeedMps + 1e-9; exitSpeedMps += 0.05) {
      const candidate = simulateCandidate(config, input, hoodRot, exitSpeedMps);
      if (candidate && isBetterCandidate(selection, candidate, best)) {
        best = candidate;
      }
    }
  }
  return best;
}

export function isInsideShotMapDomain(config: ShotCalculatorConfig, input: ShotInput): boolean {
  return input.distanceMeters >= config.minDistanceMeters
    && input.distanceMeters <= config.maxDistanceMeters
    && input.radialVelocityMps >= config.minRadialVelocityMps
    && input.radialVelocityMps <= config.maxRadialVelocityMps;
}

export function generateShotSamples(config: ShotCalculatorConfig): TrajectoryCandidate[] {
  const samples: TrajectoryCandidate[] = [];
  for (let distance = config.minDistanceMeters; distance <= config.maxDistanceMeters + 1e-9; distance += 0.25) {
    for (let radialVelocity = config.minRadialVelocityMps; radialVelocity <= config.maxRadialVelocityMps + 1e-9; radialVelocity += 0.25) {
      const candidate = findBestCandidate(config, { distanceMeters: distance, radialVelocityMps: radialVelocity });
      if (candidate) samples.push(candidate);
    }
  }
  return samples;
}

function clearsHubOpening(
  config: ShotCalculatorConfig,
  distanceMeters: number,
  targetLineSpeed: number,
  verticalLaunchSpeed: number,
): boolean {
  if (!config.requiresHubOpeningClearance) return true;

  const lipDistanceMeters = Math.max(0.0, distanceMeters - (config.hubOpeningDepthMeters / 2.0));
  const lipTimeSec = lipDistanceMeters / targetLineSpeed;
  const lipHeightMeters = config.shooterExitHeightMeters
    + (verticalLaunchSpeed * lipTimeSec)
    - (0.5 * config.gravityMps2 * lipTimeSec * lipTimeSec);
  const minimumBallCenterHeight = config.hubFrontLipHeightMeters
    + config.fuelRadiusMeters
    + config.hubLipClearanceMeters;
  const targetTimeSec = distanceMeters / targetLineSpeed;
  const verticalVelocityAtTarget = verticalLaunchSpeed - (config.gravityMps2 * targetTimeSec);
  const entryAngleDeg = Math.atan2(-verticalVelocityAtTarget, targetLineSpeed) * 180.0 / Math.PI;

  return lipHeightMeters >= minimumBallCenterHeight
    && entryAngleDeg >= config.minEntryAngleDeg;
}

function isBetterCandidate(
  selection: CandidateSelection,
  candidate: TrajectoryCandidate,
  best: TrajectoryCandidate | null,
): boolean {
  if (!best) return true;

  if (selection === 'flat-fast') {
    if (candidate.flightTimeSec + 1e-9 < best.flightTimeSec) return true;
    if (Math.abs(candidate.flightTimeSec - best.flightTimeSec) <= 1e-9
      && candidate.launchAngleDeg + 1e-9 < best.launchAngleDeg) {
      return true;
    }
    return Math.abs(candidate.flightTimeSec - best.flightTimeSec) <= 1e-9
      && Math.abs(candidate.launchAngleDeg - best.launchAngleDeg) <= 1e-9
      && candidate.exitSpeedMps > best.exitSpeedMps + 1e-9;
  }

  if (selection === 'flattest') {
    if (candidate.launchAngleDeg + 1e-9 < best.launchAngleDeg) return true;
    if (Math.abs(candidate.launchAngleDeg - best.launchAngleDeg) <= 1e-9
      && candidate.flightTimeSec + 1e-9 < best.flightTimeSec) {
      return true;
    }
    return Math.abs(candidate.launchAngleDeg - best.launchAngleDeg) <= 1e-9
      && Math.abs(candidate.flightTimeSec - best.flightTimeSec) <= 1e-9
      && candidate.exitSpeedMps + 1e-9 < best.exitSpeedMps;
  }

  return candidate.robustnessMargin > best.robustnessMargin;
}

function scoreRobustness(
  config: ShotCalculatorConfig,
  input: ShotInput,
  hoodRot: number,
  exitSpeedMps: number,
  heightMarginMeters: number,
  flightTimeSec: number,
): number {
  const hoodCenter = (config.minHoodRot + config.maxHoodRot) / 2.0;
  const hoodSpan = (config.maxHoodRot - config.minHoodRot) / 2.0;
  const hoodEdgePenalty = Math.abs(hoodRot - hoodCenter) / hoodSpan;
  const speedHeadroom = (config.maxExitSpeedMps - exitSpeedMps) / (config.maxExitSpeedMps - config.minExitSpeedMps);
  const distancePenalty = (input.distanceMeters - config.minDistanceMeters)
    / (config.maxDistanceMeters - config.minDistanceMeters);
  return heightMarginMeters
    + (0.08 * speedHeadroom)
    - (0.08 * flightTimeSec)
    - (0.05 * hoodEdgePenalty)
    - (0.04 * distancePenalty);
}
