export type Polynomial2 = [number, number, number, number, number, number];

export interface ShotCalculatorConfig {
  gravityMps2: number;
  shooterExitHeightMeters: number;
  targetHeightMeters: number;
  goalRadiusMeters: number;
  fuelRadiusMeters: number;
  hubOpeningDepthMeters: number;
  hubFrontLipHeightMeters: number;
  hubLipClearanceMeters: number;
  minEntryAngleDeg: number;
  requiresHubOpeningClearance: boolean;
  minDistanceMeters: number;
  maxDistanceMeters: number;
  minRadialVelocityMps: number;
  maxRadialVelocityMps: number;
  minHoodRot: number;
  maxHoodRot: number;
  minExitSpeedMps: number;
  maxExitSpeedMps: number;
  hoodSlope: number;
  hoodIntercept: number;
  rollerDirectionSign: number;
  rollerSlope: number;
  rollerIntercept: number;
}

export interface ShotInput {
  distanceMeters: number;
  radialVelocityMps: number;
}

export interface TrajectoryCandidate {
  input: ShotInput;
  hoodRot: number;
  launchAngleDeg: number;
  exitSpeedMps: number;
  rollerRps: number;
  flightTimeSec: number;
  peakHeightMeters: number;
  impactHeightMeters: number;
  heightMarginMeters: number;
  robustnessMargin: number;
}

export interface ShotMapFit {
  hoodCoefficients: Polynomial2;
  rollerCoefficients: Polynomial2;
  maxHoodResidualRot: number;
  maxRollerResidualRps: number;
  samples: TrajectoryCandidate[];
}
