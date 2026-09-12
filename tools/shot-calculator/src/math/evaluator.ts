import { generatedCoefficients } from './generatedCoefficients';
import { evaluatePolynomial } from './polynomial';
import {
  closeRangeHubConfig,
  findBestCandidate,
  isInsideShotMapDomain,
  launchAngleDegForHood,
} from './trajectory';
import { solveTurretShot, type TurretSolution } from './turret';
import type { CandidateSelection } from './trajectory';
import type { Point, Pose2d } from './field';
import type { ShotCalculatorConfig, TrajectoryCandidate } from './types';

export interface ReferenceShotInput {
  pose: Pose2d;
  velocity: Point;
  acceleration: Point;
  omegaRadPerSec: number;
  angularAccelRadPerSec2: number;
  releaseLatencySec: number;
  shooterOffset: Point;
  targetPoint: Point;
  config: ShotCalculatorConfig;
  usesPolynomial: boolean;
  candidateSelection: CandidateSelection;
  robotPitchDeg: number;
  robotRollDeg: number;
  turretZeroOffsetDeg: number;
  currentTurretAngleDeg: number;
}

export interface ReferenceShotResult {
  feasible: boolean;
  reason: string;
  commandSource: 'generated-polynomial' | 'direct-candidate' | 'none';
  selectionMode: CandidateSelection;
  predictedPose: Pose2d;
  shooterPoint: Point;
  shooterVelocity: Point;
  distanceMeters: number;
  radialVelocityMps: number;
  tangentialVelocityMps: number;
  hoodPositionRot: number | null;
  rollerRps: number | null;
  targetPlaneCrossingTimeSec: number | null;
  impactHeightMeters: number | null;
  entryAngleDeg: number | null;
  turret: TurretSolution | null;
}

export interface FinalTrajectoryResult {
  feasible: boolean;
  reason: string;
  targetPlaneCrossingTimeSec: number | null;
  impactHeightMeters: number | null;
  peakHeightMeters: number | null;
  crossTrackAtTargetMeters: number | null;
  entryAngleDeg: number | null;
  frontLipCleared: boolean;
  openingIntersectionValid: boolean;
  openingSamples: Array<{ distanceMeters: number; heightMeters: number; crossTrackMeters: number }>;
}

/** Robot-relevant calculator evaluator. It has no DOM, rendering, or mutable UI dependencies. */
export function evaluateReferenceShot(input: ReferenceShotInput): ReferenceShotResult {
  if (!validInput(input)) return invalid(input, 'nonfinite-input');
  const latency = Math.max(0, input.releaseLatencySec);
  const predictedPose = {
    x: input.pose.x + input.velocity.x * latency + 0.5 * input.acceleration.x * latency * latency,
    y: input.pose.y + input.velocity.y * latency + 0.5 * input.acceleration.y * latency * latency,
    headingRad: wrapRadians(input.pose.headingRad + input.omegaRadPerSec * latency
      + 0.5 * input.angularAccelRadPerSec2 * latency * latency),
  };
  const releaseOmega = input.omegaRadPerSec + input.angularAccelRadPerSec2 * latency;
  const centerVelocity = {
    x: input.velocity.x + input.acceleration.x * latency,
    y: input.velocity.y + input.acceleration.y * latency,
  };
  const shooterPoint = offsetPoint(predictedPose, input.shooterOffset);
  const radius = { x: shooterPoint.x - predictedPose.x, y: shooterPoint.y - predictedPose.y };
  const shooterVelocity = {
    x: centerVelocity.x - releaseOmega * radius.y,
    y: centerVelocity.y + releaseOmega * radius.x,
  };
  const delta = { x: input.targetPoint.x - shooterPoint.x, y: input.targetPoint.y - shooterPoint.y };
  const distanceMeters = Math.hypot(delta.x, delta.y);
  if (distanceMeters <= 1e-9) return invalid(input, 'target-too-close', predictedPose, shooterPoint, shooterVelocity);
  const unit = { x: delta.x / distanceMeters, y: delta.y / distanceMeters };
  const tangent = { x: -unit.y, y: unit.x };
  const radialVelocityMps = dot(shooterVelocity, unit);
  const tangentialVelocityMps = dot(shooterVelocity, tangent);
  const quantized = {
    distanceMeters: roundStep(distanceMeters, 0.05),
    radialVelocityMps: roundStep(radialVelocityMps, 0.05),
  };

  let candidate: TrajectoryCandidate | null = null;
  let hoodPositionRot: number | null = null;
  let rollerRps: number | null = null;
  let commandSource: ReferenceShotResult['commandSource'] = 'none';
  if (input.usesPolynomial && isInsideShotMapDomain(input.config, { distanceMeters, radialVelocityMps })) {
    const hood = evaluatePolynomial(generatedCoefficients.hood, { distanceMeters, radialVelocityMps });
    const roller = evaluatePolynomial(generatedCoefficients.roller, { distanceMeters, radialVelocityMps });
    if (hood !== null && roller !== null && hood >= input.config.minHoodRot
      && hood <= input.config.maxHoodRot && Math.abs(roller) <= input.config.maxRollerRps) {
      hoodPositionRot = hood;
      rollerRps = roller;
      commandSource = 'generated-polynomial';
    }
    candidate = findBestCandidate(input.config, quantized, input.candidateSelection);
  } else {
    const candidateConfig = input.usesPolynomial
      ? { ...closeRangeHubConfig, targetHeightMeters: input.config.targetHeightMeters }
      : input.config;
    candidate = findBestCandidate(candidateConfig, quantized, input.candidateSelection);
    if (candidate) {
      hoodPositionRot = candidate.hoodRot;
      rollerRps = candidate.rollerRps;
      commandSource = 'direct-candidate';
    }
  }
  if (hoodPositionRot === null || rollerRps === null) {
    return invalid(input, candidate ? 'invalid-shot-command' : 'no-physics-valid-trajectory', predictedPose,
      shooterPoint, shooterVelocity, distanceMeters, radialVelocityMps, tangentialVelocityMps);
  }

  const exitSpeedMps = (Math.abs(rollerRps) - input.config.rollerIntercept) / input.config.rollerSlope;
  const turret = solveTurretShot({
    shooterPoint,
    targetPoint: input.targetPoint,
    predictedRobotPose: predictedPose,
    tangentialVelocityMps,
    launchAngleDeg: launchAngleDegForHood(input.config, hoodPositionRot),
    exitSpeedMps,
    pitchDeg: input.robotPitchDeg,
    rollDeg: input.robotRollDeg,
    turretZeroOffsetDeg: input.turretZeroOffsetDeg,
    currentTurretAngleDeg: input.currentTurretAngleDeg,
  });
  if (!turret.valid) {
    return { ...base(input, predictedPose, shooterPoint, shooterVelocity, distanceMeters,
      radialVelocityMps, tangentialVelocityMps), feasible: false, reason: turret.reason,
      commandSource, hoodPositionRot, rollerRps, targetPlaneCrossingTimeSec: null,
      impactHeightMeters: null, entryAngleDeg: null, turret };
  }

  const radialFieldSpeed = radialVelocityMps + dot(turret.launchVector, unit);
  const tangentialFieldSpeed = tangentialVelocityMps + dot(turret.launchVector, tangent);
  const finalTrajectory = evaluateFinalTrajectory(
    input.config,
    distanceMeters,
    radialFieldSpeed,
    tangentialFieldSpeed,
    turret.launchVector.z,
  );
  return result(
    finalTrajectory.feasible,
    finalTrajectory.reason,
    finalTrajectory.targetPlaneCrossingTimeSec,
    finalTrajectory.impactHeightMeters,
    finalTrajectory.entryAngleDeg,
  );

  function result(feasible: boolean, reason: string, time: number | null, height: number | null, entry: number | null): ReferenceShotResult {
    return { ...base(input, predictedPose, shooterPoint, shooterVelocity, distanceMeters,
      radialVelocityMps, tangentialVelocityMps), feasible, reason, commandSource,
      hoodPositionRot, rollerRps, targetPlaneCrossingTimeSec: time,
      impactHeightMeters: height, entryAngleDeg: entry, turret };
  }
}

/** Continuous final-command check; candidate input quantization is deliberately outside this function. */
export function evaluateFinalTrajectory(
  config: ShotCalculatorConfig,
  targetDistanceMeters: number,
  radialFieldSpeedMps: number,
  tangentialFieldSpeedMps: number,
  verticalLaunchSpeedMps: number,
): FinalTrajectoryResult {
  const empty = (reason: string): FinalTrajectoryResult => ({
    feasible: false, reason, targetPlaneCrossingTimeSec: null, impactHeightMeters: null,
    peakHeightMeters: null, crossTrackAtTargetMeters: null, entryAngleDeg: null,
    frontLipCleared: false, openingIntersectionValid: false, openingSamples: [],
  });
  if (!Number.isFinite(radialFieldSpeedMps) || radialFieldSpeedMps <= 0.05) {
    return empty('target-plane-crossing-unavailable');
  }
  const crossingTime = targetDistanceMeters / radialFieldSpeedMps;
  const impactHeight = heightAt(config, verticalLaunchSpeedMps, crossingTime);
  const peakTime = Math.max(0, verticalLaunchSpeedMps / config.gravityMps2);
  const peakHeight = heightAt(config, verticalLaunchSpeedMps, peakTime);
  const crossTrack = tangentialFieldSpeedMps * crossingTime;
  const halfDepth = config.requiresHubOpeningClearance ? config.hubOpeningDepthMeters / 2 : 0.18;
  const frontDistance = Math.max(0, targetDistanceMeters - halfDepth);
  const backDistance = targetDistanceMeters + halfDepth;
  const openingSamples = Array.from({ length: 9 }, (_, index) => {
    const distanceMeters = frontDistance + (backDistance - frontDistance) * index / 8;
    const time = distanceMeters / radialFieldSpeedMps;
    return { distanceMeters, heightMeters: heightAt(config, verticalLaunchSpeedMps, time),
      crossTrackMeters: tangentialFieldSpeedMps * time };
  });
  const openingIntersectionValid = openingSamples.some((sample) =>
    sample.heightMeters >= config.targetHeightMeters - config.goalRadiusMeters
      && sample.heightMeters <= config.targetHeightMeters + config.goalRadiusMeters
      && Math.abs(sample.crossTrackMeters) <= config.goalRadiusMeters);
  const frontHeight = heightAt(config, verticalLaunchSpeedMps, frontDistance / radialFieldSpeedMps);
  const frontLipCleared = !config.requiresHubOpeningClearance || frontHeight >= config.hubFrontLipHeightMeters
    + config.fuelRadiusMeters + config.hubLipClearanceMeters;
  const verticalAtTarget = verticalLaunchSpeedMps - config.gravityMps2 * crossingTime;
  const entryAngleDeg = Math.atan2(-verticalAtTarget, radialFieldSpeedMps) * 180 / Math.PI;
  const entryAngleValid = !config.requiresHubOpeningClearance || entryAngleDeg >= config.minEntryAngleDeg;
  const feasible = openingIntersectionValid && frontLipCleared && entryAngleValid;
  const reason = !openingIntersectionValid ? 'target-opening-miss'
    : (!frontLipCleared ? 'hub-front-lip-clearance' : (!entryAngleValid ? 'hub-entry-angle' : 'ready'));
  return { feasible, reason, targetPlaneCrossingTimeSec: crossingTime, impactHeightMeters: impactHeight,
    peakHeightMeters: peakHeight, crossTrackAtTargetMeters: crossTrack, entryAngleDeg,
    frontLipCleared, openingIntersectionValid, openingSamples };
}

function base(input: ReferenceShotInput, predictedPose: Pose2d = input.pose, shooterPoint: Point = input.pose,
  shooterVelocity: Point = input.velocity, distanceMeters = Number.NaN, radialVelocityMps = Number.NaN,
  tangentialVelocityMps = Number.NaN) {
  return { selectionMode: input.candidateSelection, predictedPose, shooterPoint, shooterVelocity,
    distanceMeters, radialVelocityMps, tangentialVelocityMps };
}
function invalid(input: ReferenceShotInput, reason: string, predictedPose: Pose2d = input.pose, shooterPoint: Point = input.pose,
  shooterVelocity: Point = input.velocity, distanceMeters = Number.NaN, radialVelocityMps = Number.NaN,
  tangentialVelocityMps = Number.NaN): ReferenceShotResult {
  return { ...base(input, predictedPose, shooterPoint, shooterVelocity, distanceMeters, radialVelocityMps,
    tangentialVelocityMps), feasible: false, reason, commandSource: 'none', hoodPositionRot: null,
    rollerRps: null, targetPlaneCrossingTimeSec: null, impactHeightMeters: null, entryAngleDeg: null, turret: null };
}
function validInput(input: ReferenceShotInput) {
  return input !== null && Object.values(input.pose).every(Number.isFinite)
    && Object.values(input.velocity).every(Number.isFinite) && Object.values(input.acceleration).every(Number.isFinite)
    && Object.values(input.shooterOffset).every(Number.isFinite) && Object.values(input.targetPoint).every(Number.isFinite)
    && [input.omegaRadPerSec, input.angularAccelRadPerSec2, input.releaseLatencySec, input.robotPitchDeg,
      input.robotRollDeg, input.turretZeroOffsetDeg, input.currentTurretAngleDeg].every(Number.isFinite);
}
function offsetPoint(pose: Pose2d, offset: Point): Point { const c = Math.cos(pose.headingRad); const s = Math.sin(pose.headingRad); return { x: pose.x + offset.x * c - offset.y * s, y: pose.y + offset.x * s + offset.y * c }; }
function heightAt(config: ShotCalculatorConfig, vz: number, time: number) { return config.shooterExitHeightMeters + vz * time - 0.5 * config.gravityMps2 * time * time; }
function dot(a: Point, b: Point) { return a.x * b.x + a.y * b.y; }
function roundStep(value: number, step: number) { return Math.round(value / step) * step; }
function wrapRadians(value: number) { return Math.atan2(Math.sin(value), Math.cos(value)); }
