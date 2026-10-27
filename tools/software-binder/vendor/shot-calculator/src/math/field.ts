import { defaultConfig } from './trajectory';
import type { ShotCalculatorConfig } from './types';

export type Alliance = 'blue' | 'red';
export type PageMode = 'drag' | 'sim' | 'path';
export type TargetMode = 'auto' | 'hub' | 'neutral-back' | 'opponent-passback';
export type ResolvedTargetMode = Exclude<TargetMode, 'auto'>;
export type FieldElementKind = 'blocked' | 'ramp' | 'low-clearance' | 'field-marker';

export interface Point {
  x: number;
  y: number;
}

export interface Pose2d extends Point {
  headingRad: number;
}

export interface FieldRect {
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface FieldBounds extends FieldRect {
  scale: number;
}

export interface FieldElement extends FieldRect {
  name: string;
  kind: FieldElementKind;
  alliance?: Alliance;
}

export interface TargetSelection {
  mode: ResolvedTargetMode;
  point: Point;
  label: string;
  shortLabel: string;
  config: ShotCalculatorConfig;
  usesPolynomial: boolean;
  candidateSelection: 'robust' | 'flattest' | 'flat-fast';
  color: string;
}

export const inchesToMeters = (inches: number): number => inches * 0.0254;

export const field = {
  lengthMeters: inchesToMeters(651.2),
  widthMeters: inchesToMeters(317.7),
  allianceZoneDepthMeters: inchesToMeters(158.6),
  centerLineX: inchesToMeters(651.2) / 2.0,
  // Opposing centerline-tag midpoints from WPILib 2026.2.1's welded layout, matching Java.
  blueHubCenter: { x: 4.6255178, y: 4.0346376 },
  redHubCenter: { x: 11.915521, y: 4.0346376 },
  blueHubAim: { x: 4.6255178, y: 4.0346376 },
  redHubAim: { x: 11.915521, y: 4.0346376 },
  blueBackUpper: { x: 0.48, y: 6.00 },
  blueBackLower: { x: 0.48, y: 2.50 },
  redBackUpper: { x: 16.10, y: 6.00 },
  redBackLower: { x: 16.10, y: 2.50 },
  opponentPassBackUpper: { x: 8.295, y: 7.00 },
  opponentPassBackLower: { x: 8.295, y: 1.00 },
};

export const robotGeometry = {
  lengthMeters: inchesToMeters(27.5),
  widthMeters: inchesToMeters(27.0),
  shooterOffset: { x: -0.1651, y: -0.2159 },
};

export const driveLimits = {
  maxFreeSpeedMps: 4.94,
  maxAngularRateRadPerSec: 0.90 * Math.PI * 2.0,
  maxLinearAccelMps2: 4.5,
  maxAngularAccelRadPerSec2: (720.0 * Math.PI) / 180.0,
  passiveLinearDecelMps2: 3.0,
  passiveAngularDecelRadPerSec2: 8.0,
};

export const passbackConfig: ShotCalculatorConfig = {
  ...defaultConfig,
  targetHeightMeters: 0.5,
  goalRadiusMeters: 0.24,
  minDistanceMeters: 0.4,
  maxDistanceMeters: 12.0,
  minEntryAngleDeg: -90.0,
  requiresHubOpeningClearance: false,
};

const hubSize = inchesToMeters(47.0);
const bumpXDepth = inchesToMeters(44.4);
const bumpYWidth = inchesToMeters(73.0);
const trenchXWidth = inchesToMeters(65.65);
const towerXDepth = inchesToMeters(45.0);
const towerYWidth = inchesToMeters(49.25);
const depotXDepth = inchesToMeters(27.0);
const depotYWidth = inchesToMeters(42.0);
const depotGapFromTower = inchesToMeters(46.625);
const trenchNearCenterline = inchesToMeters(119.0);

export const fieldElements: FieldElement[] = [
  ...hubAndBumpElements('blue', field.blueHubCenter),
  ...hubAndBumpElements('red', field.redHubCenter),
  ...trenchElements('blue'),
  ...trenchElements('red'),
  towerElement('blue'),
  towerElement('red'),
  depotElement('blue'),
  depotElement('red'),
  outpostElement('blue'),
  outpostElement('red'),
];

export function selectedTarget(
  alliance: Alliance,
  targetMode: TargetMode,
  robotPose: Pose2d,
): TargetSelection {
  const mode = resolvedTargetMode(alliance, targetMode, robotPose);
  if (mode === 'hub') {
    const point = alliance === 'blue' ? field.blueHubAim : field.redHubAim;
    return {
      mode,
      point,
      label: `${allianceLabel(alliance)} HUB aim point`,
      shortLabel: 'HUB',
      config: defaultConfig,
      usesPolynomial: true,
      candidateSelection: 'robust',
      color: '#f8c537',
    };
  }

  if (mode === 'neutral-back') {
    const point = preferredByRobotY(robotPose.y, alliance === 'blue'
      ? [field.blueBackUpper, field.blueBackLower]
      : [field.redBackUpper, field.redBackLower]);
    return {
      mode,
      point,
      label: `${allianceLabel(alliance)} neutral back ${point.y > field.widthMeters / 2.0 ? 'upper' : 'lower'}`,
      shortLabel: 'Neutral Back',
      config: passbackConfig,
      usesPolynomial: false,
      candidateSelection: 'flat-fast',
      color: '#38bdf8',
    };
  }

  const point = preferredByRobotY(robotPose.y, [field.opponentPassBackUpper, field.opponentPassBackLower]);
  return {
    mode,
    point,
    label: `Opponent passback ${point.y > field.widthMeters / 2.0 ? 'upper' : 'lower'}`,
    shortLabel: 'Opponent Passback',
    config: passbackConfig,
    usesPolynomial: false,
    candidateSelection: 'flat-fast',
    color: '#34d399',
  };
}

export function resolvedTargetMode(
  alliance: Alliance,
  targetMode: TargetMode,
  robotPose: Point,
): ResolvedTargetMode {
  if (targetMode !== 'auto') return targetMode;
  const opponentAlliance = alliance === 'blue' ? 'red' : 'blue';
  if (isInAllianceZone(alliance, robotPose.x)) return 'hub';
  if (!isInAllianceZone(opponentAlliance, robotPose.x)) return 'neutral-back';
  return 'opponent-passback';
}

export function isInAllianceZone(alliance: Alliance, xMeters: number): boolean {
  return alliance === 'blue'
    ? xMeters <= field.allianceZoneDepthMeters
    : xMeters >= field.lengthMeters - field.allianceZoneDepthMeters;
}

export function shooterFieldPosition(pose: Pose2d): Point {
  return transformRobotToField(pose, robotGeometry.shooterOffset);
}

export function transformRobotToField(pose: Pose2d, robotPoint: Point): Point {
  const cos = Math.cos(pose.headingRad);
  const sin = Math.sin(pose.headingRad);
  return {
    x: pose.x + (robotPoint.x * cos) - (robotPoint.y * sin),
    y: pose.y + (robotPoint.x * sin) + (robotPoint.y * cos),
  };
}

export function robotFootprint(pose: Pose2d): Point[] {
  const halfLength = robotGeometry.lengthMeters / 2.0;
  const halfWidth = robotGeometry.widthMeters / 2.0;
  return [
    transformRobotToField(pose, { x: halfLength, y: halfWidth }),
    transformRobotToField(pose, { x: halfLength, y: -halfWidth }),
    transformRobotToField(pose, { x: -halfLength, y: -halfWidth }),
    transformRobotToField(pose, { x: -halfLength, y: halfWidth }),
  ];
}

export function clampPoseInsideField(pose: Pose2d): Pose2d {
  const halfDiagonal = Math.hypot(robotGeometry.lengthMeters, robotGeometry.widthMeters) / 2.0;
  return {
    ...pose,
    x: clamp(pose.x, halfDiagonal, field.lengthMeters - halfDiagonal),
    y: clamp(pose.y, halfDiagonal, field.widthMeters - halfDiagonal),
  };
}

export function isReachablePose(pose: Pose2d): boolean {
  const footprint = robotFootprint(pose);
  if (!footprint.every((point) => isInsideField(point))) return false;

  return !fieldElements
    .filter((element) => isBlockingElement(element))
    .some((element) => polygonsIntersect(footprint, rectPolygon(element)));
}

export function radialTangentialVelocity(
  shooterPoint: Point,
  targetPoint: Point,
  fieldVelocity: Point,
): { radialVelocityMps: number; tangentialVelocityMps: number } {
  const dx = targetPoint.x - shooterPoint.x;
  const dy = targetPoint.y - shooterPoint.y;
  const distance = Math.hypot(dx, dy);
  if (distance <= 1e-9) return { radialVelocityMps: 0, tangentialVelocityMps: 0 };
  const ux = dx / distance;
  const uy = dy / distance;
  return {
    radialVelocityMps: (fieldVelocity.x * ux) + (fieldVelocity.y * uy),
    tangentialVelocityMps: (fieldVelocity.x * -uy) + (fieldVelocity.y * ux),
  };
}

export function targetModeLabel(targetMode: TargetMode): string {
  switch (targetMode) {
    case 'auto':
      return 'Auto';
    case 'hub':
      return 'Hub';
    case 'neutral-back':
      return 'Neutral Back';
    case 'opponent-passback':
      return 'Opponent Pass';
  }
}

export function allianceLabel(alliance: Alliance): string {
  return alliance === 'blue' ? 'Blue' : 'Red';
}

export function isBlockingElement(element: FieldElement): boolean {
  return element.kind === 'blocked';
}

function preferredByRobotY(robotY: number, points: Point[]): Point {
  return points.reduce((best, candidate) => {
    const candidateDistance = Math.abs(candidate.y - robotY);
    const bestDistance = Math.abs(best.y - robotY);
    if (candidateDistance < bestDistance) return candidate;
    if (Math.abs(candidateDistance - bestDistance) <= 1e-9 && candidate.y < best.y) return candidate;
    return best;
  });
}

function hubAndBumpElements(alliance: Alliance, hubCenter: Point): FieldElement[] {
  const colorName = alliance === 'blue' ? 'Blue' : 'Red';
  const lowerBumpY = hubCenter.y - (hubSize / 2.0) - (bumpYWidth / 2.0);
  const upperBumpY = hubCenter.y + (hubSize / 2.0) + (bumpYWidth / 2.0);
  return [
    centeredRect(`${colorName} HUB structure`, 'blocked', hubCenter, hubSize, hubSize, alliance),
    centeredRect(`${colorName} lower BUMP`, 'ramp', { x: hubCenter.x, y: lowerBumpY }, bumpXDepth, bumpYWidth, alliance),
    centeredRect(`${colorName} upper BUMP`, 'ramp', { x: hubCenter.x, y: upperBumpY }, bumpXDepth, bumpYWidth, alliance),
  ];
}

function trenchElements(alliance: Alliance): FieldElement[] {
  const hubCenter = alliance === 'blue' ? field.blueHubCenter : field.redHubCenter;
  const lowerBumpOuterY = hubCenter.y - (hubSize / 2.0) - bumpYWidth;
  const upperBumpOuterY = hubCenter.y + (hubSize / 2.0) + bumpYWidth;
  const x = alliance === 'blue'
    ? field.centerLineX - trenchNearCenterline - trenchXWidth
    : field.centerLineX + trenchNearCenterline;

  return [
    {
      name: `${allianceLabel(alliance)} lower TRENCH low-clearance arm`,
      kind: 'low-clearance',
      alliance,
      x,
      y: 0,
      width: trenchXWidth,
      height: lowerBumpOuterY,
    },
    {
      name: `${allianceLabel(alliance)} upper TRENCH low-clearance arm`,
      kind: 'low-clearance',
      alliance,
      x,
      y: upperBumpOuterY,
      width: trenchXWidth,
      height: field.widthMeters - upperBumpOuterY,
    },
  ];
}

function towerElement(alliance: Alliance): FieldElement {
  const x = alliance === 'blue' ? 0 : field.lengthMeters - towerXDepth;
  return {
    name: `${allianceLabel(alliance)} TOWER base`,
    kind: 'blocked',
    alliance,
    x,
    y: (field.widthMeters - towerYWidth) / 2.0,
    width: towerXDepth,
    height: towerYWidth,
  };
}

function depotElement(alliance: Alliance): FieldElement {
  const towerLowerY = (field.widthMeters - towerYWidth) / 2.0;
  const towerUpperY = (field.widthMeters + towerYWidth) / 2.0;
  const lowerDepotY = towerLowerY - depotGapFromTower - depotYWidth;
  const upperDepotY = towerUpperY + depotGapFromTower;
  return {
    name: `${allianceLabel(alliance)} DEPOT`,
    kind: 'blocked',
    alliance,
    x: alliance === 'blue' ? 0 : field.lengthMeters - depotXDepth,
    y: alliance === 'blue' ? lowerDepotY : upperDepotY,
    width: depotXDepth,
    height: depotYWidth,
  };
}

function outpostElement(alliance: Alliance): FieldElement {
  return {
    name: `${allianceLabel(alliance)} OUTPOST wall`,
    kind: 'field-marker',
    alliance,
    x: alliance === 'blue' ? 0 : field.lengthMeters - inchesToMeters(10.0),
    y: field.widthMeters - inchesToMeters(34.0),
    width: inchesToMeters(10.0),
    height: inchesToMeters(34.0),
  };
}

function centeredRect(
  name: string,
  kind: FieldElementKind,
  center: Point,
  width: number,
  height: number,
  alliance?: Alliance,
): FieldElement {
  return {
    name,
    kind,
    alliance,
    x: center.x - (width / 2.0),
    y: center.y - (height / 2.0),
    width,
    height,
  };
}

function isInsideField(point: Point): boolean {
  return point.x >= 0 && point.x <= field.lengthMeters && point.y >= 0 && point.y <= field.widthMeters;
}

function rectPolygon(rect: FieldRect): Point[] {
  return [
    { x: rect.x, y: rect.y },
    { x: rect.x + rect.width, y: rect.y },
    { x: rect.x + rect.width, y: rect.y + rect.height },
    { x: rect.x, y: rect.y + rect.height },
  ];
}

function polygonsIntersect(a: Point[], b: Point[]): boolean {
  const axes = [...polygonAxes(a), ...polygonAxes(b)];
  return axes.every((axis) => projectionsOverlap(project(a, axis), project(b, axis)));
}

function polygonAxes(points: Point[]): Point[] {
  const axes: Point[] = [];
  for (let i = 0; i < points.length; i += 1) {
    const current = points[i];
    const next = points[(i + 1) % points.length];
    const edge = { x: next.x - current.x, y: next.y - current.y };
    const length = Math.hypot(edge.x, edge.y);
    if (length > 1e-9) axes.push({ x: -edge.y / length, y: edge.x / length });
  }
  return axes;
}

function project(points: Point[], axis: Point): { min: number; max: number } {
  let min = Number.POSITIVE_INFINITY;
  let max = Number.NEGATIVE_INFINITY;
  for (const point of points) {
    const value = (point.x * axis.x) + (point.y * axis.y);
    min = Math.min(min, value);
    max = Math.max(max, value);
  }
  return { min, max };
}

function projectionsOverlap(a: { min: number; max: number }, b: { min: number; max: number }): boolean {
  return a.max >= b.min && b.max >= a.min;
}

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}
