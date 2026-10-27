import {
  allianceLabel,
  clampPoseInsideField,
  driveLimits,
  field,
  fieldElements,
  isBlockingElement,
  isReachablePose,
  radialTangentialVelocity,
  robotFootprint,
  robotGeometry,
  selectedTarget,
  shooterFieldPosition,
  targetModeLabel,
  type Alliance,
  type FieldBounds,
  type FieldElement,
  type PageMode,
  type Point,
  type Pose2d,
  type TargetMode,
  type TargetSelection,
} from './math/field';
import { generatedCoefficients } from './math/generatedCoefficients';
import { evaluateFinalTrajectory } from './math/evaluator';
import { parsePathFile, sampleLoadedPath, type LoadedPath, type PathSample } from './math/pathPlayback';
import { evaluatePolynomial } from './math/polynomial';
import {
  createDriveSimState,
  stepDriveSimulator,
  type DriveSimState,
  type KeyboardState,
} from './math/simulator';
import {
  closeRangeHubConfig,
  findBestCandidate,
  isInsideShotMapDomain,
  launchAngleDegForHood,
} from './math/trajectory';
import {
  createTurretSimState,
  defaultTurretConstants,
  defaultTurretControllerConfig,
  solveTurretShot,
  stepTurretSimulator,
  type TurretControllerConfig,
  type TurretSimState,
  type TurretSolution,
} from './math/turret';
import type { ShotCalculatorConfig, TrajectoryCandidate } from './math/types';
import './ui/styles.css';

const app = document.querySelector<HTMLElement>('#app');
if (!app) throw new Error('Missing #app root');
const appRoot = app;

const appState = {
  pageMode: pageModeFromHash(),
  alliance: 'blue' as Alliance,
  targetMode: 'auto' as TargetMode,
  loadedPath: null as LoadedPath | null,
  pathTimeSec: 0,
  pathPlaying: false,
  pathMessage: 'Load a PathPlanner .path or Choreo JSON file to replay robot motion.',
  optimizeMessage: 'Score the current state or loaded path, then apply the highest ranked caps.',
  turretOptimizeMessage: 'Optimize PID against current/path tracking demand. Defaults model Java current limits plus 45% effective turret speed.',
};

let robotPose: Pose2d = {
  x: 2.0,
  y: field.widthMeters / 2.0,
  headingRad: 0,
};
let driveSim: DriveSimState = createDriveSimState(robotPose);
let turretSim: TurretSimState = createTurretSimState(0);
const keyboard: KeyboardState = { keys: new Set() };
let draggingRobot = false;
let lastFrameTime = performance.now();
let cachedThresholdRows: { key: string; rows: ThresholdRecommendation[] } | null = null;
let cachedPathOverlay: { key: string; samples: PathSample[]; ready: boolean[] } | null = null;
let cachedTurretPidOptimization: { key: string; score: TurretPidScore } | null = null;
const candidateCache = new Map<string, TrajectoryCandidate | null>();

appRoot.innerHTML = `
  <section class="panel">
    <div>
      <p class="eyebrow">Ultimate SOTM</p>
      <h1>Shot Calculator</h1>
      <p class="summary">Inspect HUB scoring, passback shots, robot reachability, release-state prediction, motion limits, and imported autonomous paths on the 2026 REBUILT field.</p>
    </div>
    <div class="controls">
      <div class="control-group">
        <span class="control-label">Visualizer Page</span>
        <div class="segmented page-mode" aria-label="Visualizer page selector">
          <button id="dragPage" class="active" type="button">Drag Inspector</button>
          <button id="simPage" type="button">Keyboard Sim</button>
          <button id="pathPage" type="button">Path Playback</button>
        </div>
      </div>
      <div class="control-group">
        <span class="control-label">Alliance</span>
        <div class="segmented alliance-mode" aria-label="Alliance selector">
          <button id="blueAlliance" class="active" type="button">Blue</button>
          <button id="redAlliance" type="button">Red</button>
        </div>
      </div>
      <div class="control-group">
        <span class="control-label">Target mode</span>
        <div class="segmented target-mode" aria-label="Target selector">
          <button id="autoTarget" class="active" type="button">Auto</button>
          <button id="hubTarget" type="button">Hub</button>
          <button id="neutralBackTarget" type="button">Neutral Back</button>
          <button id="opponentPassTarget" type="button">Opponent Pass</button>
        </div>
      </div>
      <div class="drag-only section-card">
        ${slider('heading', 'Robot heading (deg)', -180, 180, 0, 1)}
        ${slider('radial', 'Radial velocity (m/s)', -2.5, 2.5, 0, 0.05)}
        ${slider('tangential', 'Tangential velocity (m/s)', -2.5, 2.5, 0, 0.05)}
      </div>
      <div class="sim-card sim-only">
        <b>Keyboard simulator</b>
        <span>WASD = field-centric translation</span>
        <span>IJKL = robot-centric translation</span>
        <span>Q/E = rotate, R = reset velocity</span>
        <span>Hard limits: ${driveLimits.maxFreeSpeedMps.toFixed(2)} m/s, ${driveLimits.maxLinearAccelMps2.toFixed(1)} m/s², ${(driveLimits.maxAngularRateRadPerSec * 180 / Math.PI).toFixed(0)} deg/s</span>
      </div>
      <div class="path-card path-only">
        <label class="file-label">
          <span>PathPlanner/Choreo file</span>
          <input id="pathFile" type="file" accept=".path,.traj,.chor,.choreo,.json,application/json" />
        </label>
        <div class="path-controls">
          <button id="playPath" type="button">Play</button>
          <span id="pathStatus" class="path-status">${appState.pathMessage}</span>
        </div>
        ${slider('pathTime', 'Path time (s)', 0, 1, 0, 0.01)}
      </div>
      <div class="section-card">
        ${slider('pitch', 'Robot pitch (deg)', -12, 12, 0, 0.25)}
        ${slider('roll', 'Robot roll (deg)', -12, 12, 0, 0.25)}
      </div>
    </div>
  </section>
  <section class="tuning-workspace">
    <section class="field-panel">
      <canvas id="field" width="1180" height="600"></canvas>
    </section>
    <aside class="threshold-sidebar">
      <div class="threshold-card">
        <span class="control-label">Shoot-on-move thresholds</span>
        ${slider('releaseLatency', 'Release latency prediction (s)', 0, 0.35, 0.14, 0.01)}
        ${slider('desiredAccuracy', 'Required estimated accuracy (%)', 50, 99, 90, 1)}
        ${slider('shotSpeedCap', 'Shootable speed cap (m/s)', 0, driveLimits.maxFreeSpeedMps, 3.0, 0.01)}
        ${slider('shotAccelCap', 'Shootable accel cap (m/s²)', 0, 8.0, 4.5, 0.1)}
        ${slider('shotOmegaCap', 'Shootable omega cap (deg/s)', 0, 540, 324, 5)}
        ${slider('shotAlphaCap', 'Shootable angular accel cap (deg/s²)', 0, 1080, 720, 10)}
        <label class="checkbox-label">
          <input id="applyVisualLimiter" type="checkbox" checked />
          <span>Apply caps to visualizer motion</span>
        </label>
        <button id="optimizeThresholds" type="button">Score + optimize caps</button>
        <span id="optimizeStatus" class="optimize-status">${appState.optimizeMessage}</span>
        ${slider('linearFriction', 'Simulator coast friction (m/s²)', 0.5, 10, driveLimits.passiveLinearDecelMps2, 0.1)}
        ${slider('angularFriction', 'Simulator angular friction (rad/s²)', 1, 20, driveLimits.passiveAngularDecelRadPerSec2, 0.25)}
      </div>
      <div class="threshold-card turret-card">
        <span class="control-label">Turret tracker</span>
        <label class="checkbox-label">
          <input id="enableTurretSim" type="checkbox" checked />
          <span>Gate readiness on simulated turret tracking</span>
        </label>
        ${slider('turretCurrentAngle', 'Reset current turret angle (deg)', -390, 180, 0, 1)}
        <button id="resetTurretSim" type="button">Reset turret sim to angle</button>
        ${slider('turretZeroOffset', 'Turret zero offset (deg)', -30, 30, 0, 0.25)}
        ${slider('turretAimTolerance', 'Aim tolerance (deg)', 0.1, 5, defaultTurretControllerConfig.aimToleranceDeg, 0.1)}
        ${slider('turretVoltageLimit', 'Voltage limit (V)', 1, 12, defaultTurretControllerConfig.voltageLimit, 0.1)}
        ${slider('turretSupplyCurrentLimit', 'Supply current limit (A)', 5, 60, defaultTurretControllerConfig.supplyCurrentLimitA, 1)}
        ${slider('turretStatorCurrentLimit', 'Stator current limit (A)', 5, 100, defaultTurretControllerConfig.statorCurrentLimitA, 1)}
        ${slider('turretEffectiveMotionScale', 'Effective motion scale (%)', 10, 100, defaultTurretControllerConfig.effectiveMotionScale * 100.0, 1)}
        ${slider('turretKp', 'kP', 0, 20, defaultTurretControllerConfig.kP, 0.05)}
        ${slider('turretKi', 'kI', 0, 5, defaultTurretControllerConfig.kI, 0.025)}
        ${slider('turretKd', 'kD', 0, 5, defaultTurretControllerConfig.kD, 0.025)}
        ${slider('turretKs', 'kS', 0, 2, defaultTurretControllerConfig.kS, 0.01)}
        ${slider('turretKv', 'kV', 0, 1, defaultTurretControllerConfig.kV, 0.005)}
        ${slider('turretKa', 'kA', 0, 0.1, defaultTurretControllerConfig.kA, 0.001)}
        ${slider('turretMaxVelocity', 'Max turret velocity (rps)', 0.25, 25, defaultTurretControllerConfig.maxVelocityDegPerSec / 360.0, 0.25)}
        ${slider('turretMaxAcceleration', 'Max turret accel (rps²)', 1, 250, defaultTurretControllerConfig.maxAccelerationDegPerSec2 / 360.0, 1)}
        ${slider('turretMaxJerk', 'Max turret jerk (rps³)', 10, 1500, defaultTurretControllerConfig.maxJerkDegPerSec3 / 360.0, 10)}
        ${slider('turretOptimizeTurnFraction', 'Optimizer robot turn speed (%)', 10, 80, 45, 1)}
        <label class="checkbox-label">
          <input id="capsUseOptimizedPid" type="checkbox" checked />
          <span>Use best PID when scoring movement caps</span>
        </label>
        <button id="optimizeTurretPid" type="button">Optimize turret PID</button>
        <span id="turretOptimizeStatus" class="optimize-status">${appState.turretOptimizeMessage}</span>
      </div>
      <div id="metrics"></div>
    </aside>
    <section class="plot-panel">
      <canvas id="plot" width="920" height="460"></canvas>
    </section>
  </section>
`;

const fieldCanvas = document.querySelector<HTMLCanvasElement>('#field')!;
const plot = document.querySelector<HTMLCanvasElement>('#plot')!;
const metrics = document.querySelector<HTMLDivElement>('#metrics')!;
if (!fieldCanvas || !plot || !metrics) throw new Error('Missing visualizer nodes');
const metricsPanel = metrics;

for (const input of appRoot.querySelectorAll('input')) {
  input.addEventListener('input', () => {
    if (input.id === 'heading' && appState.pageMode === 'drag') {
      robotPose = { ...robotPose, headingRad: degreesToRadians(value('heading')) };
      driveSim = createDriveSimState(robotPose);
    }
    if (input.id === 'pathTime') {
      appState.pathTimeSec = value('pathTime');
      appState.pathPlaying = false;
      syncPoseToPathTime();
    }
    render();
  });
}

document.querySelector<HTMLButtonElement>('#dragPage')?.addEventListener('click', () => setPageMode('drag'));
document.querySelector<HTMLButtonElement>('#simPage')?.addEventListener('click', () => setPageMode('sim'));
document.querySelector<HTMLButtonElement>('#pathPage')?.addEventListener('click', () => setPageMode('path'));
document.querySelector<HTMLButtonElement>('#blueAlliance')?.addEventListener('click', () => setAlliance('blue'));
document.querySelector<HTMLButtonElement>('#redAlliance')?.addEventListener('click', () => setAlliance('red'));
document.querySelector<HTMLButtonElement>('#autoTarget')?.addEventListener('click', () => setTargetMode('auto'));
document.querySelector<HTMLButtonElement>('#hubTarget')?.addEventListener('click', () => setTargetMode('hub'));
document.querySelector<HTMLButtonElement>('#neutralBackTarget')?.addEventListener('click', () => setTargetMode('neutral-back'));
document.querySelector<HTMLButtonElement>('#opponentPassTarget')?.addEventListener('click', () => setTargetMode('opponent-passback'));
document.querySelector<HTMLButtonElement>('#optimizeThresholds')?.addEventListener('click', () => applyOptimizedThresholds());
document.querySelector<HTMLButtonElement>('#resetTurretSim')?.addEventListener('click', () => {
  turretSim = createTurretSimState(value('turretCurrentAngle'));
  render();
});
document.querySelector<HTMLButtonElement>('#optimizeTurretPid')?.addEventListener('click', () => applyOptimizedTurretPid());
document.querySelector<HTMLButtonElement>('#playPath')?.addEventListener('click', () => {
  if (!appState.loadedPath) return;
  appState.pathPlaying = !appState.pathPlaying;
  render();
});
document.querySelector<HTMLInputElement>('#pathFile')?.addEventListener('change', (event) => {
  const input = event.target as HTMLInputElement;
  const file = input.files?.[0];
  if (!file) return;
  void loadPathFile(file);
});

fieldCanvas.addEventListener('pointerdown', (event) => {
  if (appState.pageMode !== 'drag') return;
  draggingRobot = true;
  fieldCanvas.setPointerCapture(event.pointerId);
  updateRobotFromPointer(event);
});
fieldCanvas.addEventListener('pointermove', (event) => {
  if (draggingRobot && appState.pageMode === 'drag') updateRobotFromPointer(event);
});
fieldCanvas.addEventListener('pointerup', (event) => {
  draggingRobot = false;
  fieldCanvas.releasePointerCapture(event.pointerId);
});
fieldCanvas.addEventListener('pointercancel', () => {
  draggingRobot = false;
});

window.addEventListener('hashchange', () => setPageMode(pageModeFromHash(), false));
window.addEventListener('keydown', (event) => {
  const key = event.key.toLowerCase();
  if (!isDriveKey(key)) return;
  event.preventDefault();
  if (key === 'r') {
    driveSim = createDriveSimState(currentPose());
    return;
  }
  keyboard.keys.add(key);
});
window.addEventListener('keyup', (event) => {
  const key = event.key.toLowerCase();
  if (!isDriveKey(key)) return;
  event.preventDefault();
  keyboard.keys.delete(key);
});

syncUiState();
render();
requestAnimationFrame(animationFrame);

function animationFrame(timestamp: number): void {
  const dtSec = Math.max(0, Math.min((timestamp - lastFrameTime) / 1000.0, 0.05));
  lastFrameTime = timestamp;
  let shouldRender = false;

  if (appState.pageMode === 'sim') {
    const useLimiter = checked('applyVisualLimiter');
    driveSim = stepDriveSimulator(driveSim, keyboard, dtSec, {
      passiveLinearDecelMps2: value('linearFriction'),
      passiveAngularDecelRadPerSec2: value('angularFriction'),
      maxFreeSpeedMps: useLimiter
        ? Math.min(value('shotSpeedCap'), driveLimits.maxFreeSpeedMps)
        : driveLimits.maxFreeSpeedMps,
      maxLinearAccelMps2: useLimiter
        ? Math.min(value('shotAccelCap'), driveLimits.maxLinearAccelMps2)
        : driveLimits.maxLinearAccelMps2,
      maxAngularRateRadPerSec: useLimiter
        ? Math.min(degreesToRadians(value('shotOmegaCap')), driveLimits.maxAngularRateRadPerSec)
        : driveLimits.maxAngularRateRadPerSec,
      maxAngularAccelRadPerSec2: useLimiter
        ? Math.min(degreesToRadians(value('shotAlphaCap')), driveLimits.maxAngularAccelRadPerSec2)
        : driveLimits.maxAngularAccelRadPerSec2,
    });
    robotPose = driveSim.pose;
    shouldRender = true;
  }

  if (appState.pageMode === 'path' && appState.loadedPath && appState.pathPlaying) {
    const pathSample = sampleLoadedPath(appState.loadedPath, appState.pathTimeSec);
    appState.pathTimeSec = Math.min(
      appState.loadedPath.totalTimeSec,
      appState.pathTimeSec + (dtSec * visualPathTimeScale(pathSample)),
    );
    if (appState.pathTimeSec >= appState.loadedPath.totalTimeSec) appState.pathPlaying = false;
    syncPoseToPathTime();
    const timeInput = document.querySelector<HTMLInputElement>('#pathTime');
    if (timeInput) timeInput.value = appState.pathTimeSec.toFixed(2);
    shouldRender = true;
  }

  if (checked('enableTurretSim')) {
    const evaluation = evaluateCurrentShot();
    const previousTurretSim = turretSim;
    turretSim = stepTurretSimulator(
      turretSim,
      evaluation.turretSolution.legalGoalTurretYawDeg,
      dtSec,
      turretControllerConfig(),
    );
    shouldRender = shouldRender
      || !turretSim.atGoal
      || Math.abs(turretSim.angleDeg - previousTurretSim.angleDeg) > 0.01
      || Math.abs(turretSim.goalDeg - previousTurretSim.goalDeg) > 0.01
      || turretSim.trackingRiskReason !== previousTurretSim.trackingRiskReason;
  }

  if (shouldRender) render();
  requestAnimationFrame(animationFrame);
}

function render(): void {
  syncUiState();
  syncOutputs();

  const evaluation = evaluateCurrentShot();
  drawField(evaluation);
  drawPlot(evaluation);
  renderMetrics(evaluation);
}

function evaluateCurrentShot(): ShotEvaluation {
  return evaluateShotFromBaseMotion(currentBaseMotion());
}

function evaluateShotFromBaseMotion(base: BaseMotionState, turretAngleOverrideDeg = currentTurretAngleDeg()): ShotEvaluation {
  const release = predictedReleaseState(base);
  const target = selectedTarget(appState.alliance, appState.targetMode, release.predictedPose);
  const distanceMeters = distance(release.shooterPoint, target.point);
  const radialTangential = radialTangentialVelocity(release.shooterPoint, target.point, release.shooterVelocity);
  const candidate = findBestCandidateCached(
    target.config,
    { distanceMeters, radialVelocityMps: radialTangential.radialVelocityMps },
    target.candidateSelection,
    target.usesPolynomial,
  );
  const hoodPrediction = target.usesPolynomial
    ? evaluatePolynomial(generatedCoefficients.hood, {
      distanceMeters,
      radialVelocityMps: radialTangential.radialVelocityMps,
    })
    : null;
  const rollerPrediction = target.usesPolynomial
    ? evaluatePolynomial(generatedCoefficients.roller, {
      distanceMeters,
      radialVelocityMps: radialTangential.radialVelocityMps,
    })
    : null;
  const polynomialValid = !target.usesPolynomial || (
    hoodPrediction !== null
    && rollerPrediction !== null
    && isInsideShotMapDomain(target.config, { distanceMeters, radialVelocityMps: radialTangential.radialVelocityMps })
    && hoodPrediction >= target.config.minHoodRot
    && hoodPrediction <= target.config.maxHoodRot
    && Math.abs(rollerPrediction) <= target.config.maxRollerRps
  );
  const closeRangeCandidateValid = target.usesPolynomial
    && !polynomialValid
    && isCloseRangeHubInput(target.config, { distanceMeters, radialVelocityMps: radialTangential.radialVelocityMps })
    && candidate !== null;
  const shotCommandValid = polynomialValid || closeRangeCandidateValid;
  const launchCommand = launchCommandForTurret(target.config, target.usesPolynomial, polynomialValid, candidate, hoodPrediction, rollerPrediction);
  const turretSolution = solveTurretShot({
    shooterPoint: release.shooterPoint,
    targetPoint: target.point,
    predictedRobotPose: release.predictedPose,
    tangentialVelocityMps: radialTangential.tangentialVelocityMps,
    launchAngleDeg: launchCommand?.launchAngleDeg ?? null,
    exitSpeedMps: launchCommand?.exitSpeedMps ?? null,
    pitchDeg: value('pitch'),
    rollDeg: value('roll'),
    turretZeroOffsetDeg: value('turretZeroOffset'),
    currentTurretAngleDeg: turretAngleOverrideDeg,
  });
  const turretTracking = evaluateTurretTracking(turretSolution, turretAngleOverrideDeg);
  const turretSideProfile = sideProfileForEvaluation(target.config, release, target.point, turretSolution);
  const threshold = evaluateThresholds(release);
  const reachable = isReachablePose(base.pose) && isReachablePose(release.predictedPose);
  const invalidReasons = [
    reachable ? null : 'robot pose blocked',
    candidate ? null : 'no physics-valid trajectory',
    shotCommandValid ? null : 'generated polynomial/cap invalid',
    threshold.ready ? null : threshold.reason,
    turretTracking.ready ? null : turretTracking.reason,
    turretSideProfile?.targetOpeningValid
      ? null
      : 'missing or invalid turret-compensated trajectory',
  ].filter((reason): reason is string => reason !== null);

  return {
    base,
    release,
    target,
    candidate,
    hoodPrediction,
    rollerPrediction,
    polynomialValid,
    shotCommandValid,
    turretSolution,
    turretTracking,
    turretSideProfile,
    threshold,
    reachable,
    distanceMeters,
    radialVelocityMps: radialTangential.radialVelocityMps,
    tangentialVelocityMps: radialTangential.tangentialVelocityMps,
    shotReady: invalidReasons.length === 0,
    invalidReason: invalidReasons.join(', '),
  };
}

function currentBaseMotion(): BaseMotionState {
  const pose = currentPose();
  if (appState.pageMode === 'sim') {
    return {
      pose,
      velocity: driveSim.velocity,
      acceleration: driveSim.acceleration,
      omegaRadPerSec: driveSim.omegaRadPerSec,
      angularAccelRadPerSec2: driveSim.angularAccelRadPerSec2,
      source: 'sim',
    };
  }

  if (appState.pageMode === 'path' && appState.loadedPath) {
    const sample = sampleLoadedPath(appState.loadedPath, appState.pathTimeSec);
    return baseMotionFromPathSample(sample, checked('applyVisualLimiter'));
  }

  const target = selectedTarget(appState.alliance, appState.targetMode, pose);
  const shooterPoint = shooterFieldPosition(pose);
  const fieldVelocity = fieldVelocityFromRadialTangential(
    shooterPoint,
    target.point,
    value('radial'),
    value('tangential'),
  );
  return {
    pose,
    velocity: fieldVelocity,
    acceleration: { x: 0, y: 0 },
    omegaRadPerSec: 0,
    angularAccelRadPerSec2: 0,
    source: 'drag',
  };
}

function predictedReleaseState(base: BaseMotionState): ReleaseState {
  const latencySec = value('releaseLatency');
  const predictedPose = clampPoseInsideField({
    x: base.pose.x + (base.velocity.x * latencySec) + (0.5 * base.acceleration.x * latencySec * latencySec),
    y: base.pose.y + (base.velocity.y * latencySec) + (0.5 * base.acceleration.y * latencySec * latencySec),
    headingRad: wrapAngle(
      base.pose.headingRad
      + (base.omegaRadPerSec * latencySec)
      + (0.5 * base.angularAccelRadPerSec2 * latencySec * latencySec),
    ),
  });
  const releaseVelocity = {
    x: base.velocity.x + (base.acceleration.x * latencySec),
    y: base.velocity.y + (base.acceleration.y * latencySec),
  };
  const releaseOmega = base.omegaRadPerSec + (base.angularAccelRadPerSec2 * latencySec);
  const shooterPoint = shooterFieldPosition(predictedPose);
  const shooterRadius = {
    x: shooterPoint.x - predictedPose.x,
    y: shooterPoint.y - predictedPose.y,
  };
  const shooterVelocity = {
    x: releaseVelocity.x - (releaseOmega * shooterRadius.y),
    y: releaseVelocity.y + (releaseOmega * shooterRadius.x),
  };
  const shooterAcceleration = {
    x: base.acceleration.x - (base.angularAccelRadPerSec2 * shooterRadius.y) - (releaseOmega * releaseOmega * shooterRadius.x),
    y: base.acceleration.y + (base.angularAccelRadPerSec2 * shooterRadius.x) - (releaseOmega * releaseOmega * shooterRadius.y),
  };

  return {
    latencySec,
    predictedPose,
    currentShooterPoint: shooterFieldPosition(base.pose),
    shooterPoint,
    shooterVelocity,
    shooterAcceleration,
    omegaRadPerSec: releaseOmega,
    angularAccelRadPerSec2: base.angularAccelRadPerSec2,
  };
}

function evaluateThresholds(release: ReleaseState): ThresholdEvaluation {
  const speed = vectorMagnitude(release.shooterVelocity);
  const acceleration = vectorMagnitude(release.shooterAcceleration);
  const omegaDegPerSec = Math.abs(radiansToDegrees(release.omegaRadPerSec));
  const alphaDegPerSec2 = Math.abs(radiansToDegrees(release.angularAccelRadPerSec2));
  const failed = [
    speed <= value('shotSpeedCap') ? null : 'speed cap',
    acceleration <= value('shotAccelCap') ? null : 'accel cap',
    omegaDegPerSec <= value('shotOmegaCap') ? null : 'omega cap',
    alphaDegPerSec2 <= value('shotAlphaCap') ? null : 'angular accel cap',
  ].filter((reason): reason is string => reason !== null);
  return {
    ready: failed.length === 0,
    reason: failed.join(', '),
    speedMps: speed,
    accelerationMps2: acceleration,
    omegaDegPerSec,
    alphaDegPerSec2,
  };
}

function launchCommandForTurret(
  config: ShotCalculatorConfig,
  usesPolynomial: boolean,
  polynomialValid: boolean,
  candidate: TrajectoryCandidate | null,
  hoodPrediction: number | null,
  rollerPrediction: number | null,
): LaunchCommand | null {
  if (usesPolynomial) {
    if (!polynomialValid || hoodPrediction === null || rollerPrediction === null) {
      if (!candidate) return null;
      return {
        launchAngleDeg: candidate.launchAngleDeg,
        exitSpeedMps: candidate.exitSpeedMps,
        source: 'candidate',
      };
    }
    return {
      launchAngleDeg: launchAngleDegForHood(config, hoodPrediction),
      exitSpeedMps: exitSpeedMpsForRollerRps(config, rollerPrediction),
      source: 'polynomial',
    };
  }
  if (!candidate) return null;
  return {
    launchAngleDeg: candidate.launchAngleDeg,
    exitSpeedMps: candidate.exitSpeedMps,
    source: 'candidate',
  };
}

function exitSpeedMpsForRollerRps(config: ShotCalculatorConfig, rollerRps: number): number {
  return (Math.abs(rollerRps) - config.rollerIntercept) / config.rollerSlope;
}

function currentTurretAngleDeg(): number {
  return checked('enableTurretSim') ? turretSim.angleDeg : value('turretCurrentAngle');
}

function evaluateTurretTracking(solution: TurretSolution, turretAngleDeg = currentTurretAngleDeg()): TurretTrackingEvaluation {
  const simEnabled = checked('enableTurretSim');
  const aimErrorDeg = solution.legalGoalTurretYawDeg - turretAngleDeg;
  const toleranceDeg = value('turretAimTolerance');
  const trackingReady = !simEnabled || Math.abs(aimErrorDeg) <= toleranceDeg;
  const ready = solution.valid && trackingReady;
  let reason = '';
  if (!solution.valid) {
    reason = solution.reason;
  } else if (!trackingReady) {
    reason = `turret aim error ${aimErrorDeg.toFixed(1)} deg`;
  }
  return {
    ready,
    reason,
    simEnabled,
    currentAngleDeg: turretAngleDeg,
    aimErrorDeg,
    toleranceDeg,
  };
}

function sideProfileForEvaluation(
  config: ShotCalculatorConfig,
  release: ReleaseState,
  targetPoint: Point,
  turretSolution: TurretSolution,
): TurretSideProfile | null {
  if (!turretSolution.valid) return null;
  const dx = targetPoint.x - release.shooterPoint.x;
  const dy = targetPoint.y - release.shooterPoint.y;
  const targetDistanceMeters = Math.hypot(dx, dy);
  if (targetDistanceMeters <= 1e-9) return null;

  const unitToTarget = { x: dx / targetDistanceMeters, y: dy / targetDistanceMeters };
  const tangentUnit = { x: -unitToTarget.y, y: unitToTarget.x };
  const launchVector = turretSolution.launchVector;
  const radialLaunchSpeedMps = dot2d(launchVector, unitToTarget);
  const tangentialLaunchSpeedMps = dot2d(launchVector, tangentUnit);
  const radialFieldSpeedMps = dot2d(release.shooterVelocity, unitToTarget) + radialLaunchSpeedMps;
  const tangentialFieldSpeedMps = dot2d(release.shooterVelocity, tangentUnit) + tangentialLaunchSpeedMps;
  const verticalLaunchSpeedMps = launchVector.z;

  if (radialFieldSpeedMps <= 0.05 || !Number.isFinite(radialFieldSpeedMps)) {
    return {
      radialLaunchSpeedMps,
      tangentialLaunchSpeedMps,
      radialFieldSpeedMps,
      tangentialFieldSpeedMps,
      verticalLaunchSpeedMps,
      targetDistanceMeters,
      flightTimeToTargetSec: Number.NaN,
      impactHeightMeters: Number.NaN,
      peakHeightMeters: config.shooterExitHeightMeters,
      crossTrackAtTargetMeters: Number.NaN,
      targetOpeningValid: false,
      frontLipCleared: false,
      openingIntersectionValid: false,
      entryAngleDeg: Number.NaN,
      entryAngleValid: false,
      openingSamples: [],
    };
  }

  const finalTrajectory = evaluateFinalTrajectory(
    config,
    targetDistanceMeters,
    radialFieldSpeedMps,
    tangentialFieldSpeedMps,
    verticalLaunchSpeedMps,
  );

  return {
    radialLaunchSpeedMps,
    tangentialLaunchSpeedMps,
    radialFieldSpeedMps,
    tangentialFieldSpeedMps,
    verticalLaunchSpeedMps,
    targetDistanceMeters,
    flightTimeToTargetSec: finalTrajectory.targetPlaneCrossingTimeSec ?? Number.NaN,
    impactHeightMeters: finalTrajectory.impactHeightMeters ?? Number.NaN,
    peakHeightMeters: finalTrajectory.peakHeightMeters ?? Number.NaN,
    crossTrackAtTargetMeters: finalTrajectory.crossTrackAtTargetMeters ?? Number.NaN,
    targetOpeningValid: finalTrajectory.feasible,
    frontLipCleared: finalTrajectory.frontLipCleared,
    openingIntersectionValid: finalTrajectory.openingIntersectionValid,
    entryAngleDeg: finalTrajectory.entryAngleDeg ?? Number.NaN,
    entryAngleValid: !config.requiresHubOpeningClearance
      || (finalTrajectory.entryAngleDeg ?? Number.NEGATIVE_INFINITY) >= config.minEntryAngleDeg,
    openingSamples: finalTrajectory.openingSamples,
  };
}

function openingSampleDistances(config: ShotCalculatorConfig, targetDistanceMeters: number): number[] {
  const front = config.requiresHubOpeningClearance
    ? Math.max(0.0, targetDistanceMeters - (config.hubOpeningDepthMeters / 2.0))
    : Math.max(0.0, targetDistanceMeters - 0.18);
  const back = config.requiresHubOpeningClearance
    ? targetDistanceMeters + (config.hubOpeningDepthMeters / 2.0)
    : targetDistanceMeters + 0.18;
  return Array.from({ length: 9 }, (_, index) => front + ((back - front) * index) / 8.0);
}

function heightAtTime(config: ShotCalculatorConfig, verticalLaunchSpeedMps: number, timeSec: number): number {
  return config.shooterExitHeightMeters
    + (verticalLaunchSpeedMps * timeSec)
    - (0.5 * config.gravityMps2 * timeSec * timeSec);
}

function turretControllerConfig(): TurretControllerConfig {
  return {
    kP: value('turretKp'),
    kI: value('turretKi'),
    kD: value('turretKd'),
    kS: value('turretKs'),
    kV: value('turretKv'),
    kA: value('turretKa'),
    motorRotationsPerTurretRotation: defaultTurretConstants.motorRotationsPerTurretRotation,
    maxVelocityDegPerSec: value('turretMaxVelocity') * 360.0,
    maxAccelerationDegPerSec2: value('turretMaxAcceleration') * 360.0,
    maxJerkDegPerSec3: value('turretMaxJerk') * 360.0,
    effectiveMotionScale: value('turretEffectiveMotionScale') / 100.0,
    aimToleranceDeg: value('turretAimTolerance'),
    voltageLimit: value('turretVoltageLimit'),
    supplyCurrentLimitA: value('turretSupplyCurrentLimit'),
    statorCurrentLimitA: value('turretStatorCurrentLimit'),
  };
}

function capsTurretControllerConfig(): TurretControllerConfig {
  if (checked('capsUseOptimizedPid') && cachedTurretPidOptimization?.key === turretPidOptimizationKey()) {
    return cachedTurretPidOptimization.score.config;
  }
  return turretControllerConfig();
}

function turretPidOptimizationKey(): string {
  const pose = currentPose();
  return [
    appState.pageMode,
    appState.alliance,
    appState.targetMode,
    appState.loadedPath?.name ?? 'no-path',
    appState.loadedPath?.samples.length ?? 0,
    pose.x.toFixed(2),
    pose.y.toFixed(2),
    pose.headingRad.toFixed(2),
    value('shotSpeedCap').toFixed(2),
    value('shotAccelCap').toFixed(2),
    value('shotOmegaCap').toFixed(0),
    value('shotAlphaCap').toFixed(0),
    value('turretOptimizeTurnFraction').toFixed(0),
    value('turretEffectiveMotionScale').toFixed(0),
    value('turretSupplyCurrentLimit').toFixed(0),
    value('turretStatorCurrentLimit').toFixed(0),
    value('turretMaxVelocity').toFixed(2),
    value('turretMaxAcceleration').toFixed(1),
    value('turretMaxJerk').toFixed(0),
    value('turretAimTolerance').toFixed(2),
    value('turretKp').toFixed(2),
    value('turretKi').toFixed(3),
    value('turretKd').toFixed(3),
    value('turretKv').toFixed(3),
    value('turretKa').toFixed(3),
    value('releaseLatency').toFixed(2),
  ].join('|');
}

function renderMetrics(evaluation: ShotEvaluation): void {
  const target = evaluation.target;
  const candidate = evaluation.candidate;
  const thresholdRows = thresholdRecommendationRows(false);
  const pathMessage = appState.loadedPath
    ? `${escapeHtml(appState.loadedPath.name)} (${appState.loadedPath.source}, ${appState.loadedPath.samples.length} samples, ${appState.loadedPath.totalTimeSec.toFixed(2)} s)`
    : escapeHtml(appState.pathMessage);
  const statusClass = evaluation.shotReady ? 'good' : 'bad';

  metricsPanel.innerHTML = `
    <h2>${evaluation.shotReady ? 'Shot Ready' : 'Shot Not Ready'} - ${target.shortLabel}</h2>
    <p><b>Status:</b> <span class="${statusClass}">${evaluation.shotReady ? 'ready' : escapeHtml(evaluation.invalidReason || 'not ready')}</span></p>
    <p><b>Page:</b> ${pageModeLabel(appState.pageMode)}</p>
    <p><b>Visualizer drive limiter:</b> ${checked('applyVisualLimiter') ? 'enabled for sim/path playback' : 'disabled; thresholds only gate readiness'}</p>
    <p><b>Requested mode:</b> ${targetModeLabel(appState.targetMode)}${appState.targetMode === 'auto' ? ` -> ${target.shortLabel}` : ''}</p>
    <p><b>Model:</b> ${target.usesPolynomial ? 'HUB robust map + generated polynomial cap checks' : 'Passback direct sweep with flat-fast selector'}</p>
    <p><b>Path:</b> ${pathMessage}</p>
    <p><b>Robot center now:</b> x ${evaluation.base.pose.x.toFixed(2)} m, y ${evaluation.base.pose.y.toFixed(2)} m, heading ${radiansToDegrees(evaluation.base.pose.headingRad).toFixed(1)} deg</p>
    <p><b>Predicted release:</b> x ${evaluation.release.predictedPose.x.toFixed(2)} m, y ${evaluation.release.predictedPose.y.toFixed(2)} m, latency ${evaluation.release.latencySec.toFixed(2)} s</p>
    <p><b>Shooter pivot:</b> x ${evaluation.release.shooterPoint.x.toFixed(2)} m, y ${evaluation.release.shooterPoint.y.toFixed(2)} m; offset ${robotGeometry.shooterOffset.x.toFixed(3)}, ${robotGeometry.shooterOffset.y.toFixed(3)} m</p>
    <p><b>Target:</b> ${target.label} at x ${target.point.x.toFixed(2)} m, y ${target.point.y.toFixed(2)} m, z ${target.config.targetHeightMeters.toFixed(2)} m</p>
    <p><b>Distance:</b> ${evaluation.distanceMeters.toFixed(2)} m from predicted shooter pivot</p>
    <p><b>Radial/tangential:</b> ${evaluation.radialVelocityMps.toFixed(2)} / ${evaluation.tangentialVelocityMps.toFixed(2)} m/s</p>
    <p><b>Release velocity:</b> vx ${evaluation.release.shooterVelocity.x.toFixed(2)} m/s, vy ${evaluation.release.shooterVelocity.y.toFixed(2)} m/s, speed ${evaluation.threshold.speedMps.toFixed(2)} m/s</p>
    <p><b>Release accel:</b> ax ${evaluation.release.shooterAcceleration.x.toFixed(2)} m/s², ay ${evaluation.release.shooterAcceleration.y.toFixed(2)} m/s², norm ${evaluation.threshold.accelerationMps2.toFixed(2)} m/s²</p>
    <p><b>Angular motion:</b> omega ${evaluation.threshold.omegaDegPerSec.toFixed(0)} deg/s, alpha ${evaluation.threshold.alphaDegPerSec2.toFixed(0)} deg/s²</p>
    <p><b>Hood:</b> ${candidate ? `${candidate.hoodRot.toFixed(3)} rot (${candidate.launchAngleDeg.toFixed(1)} deg)` : 'N/A'}</p>
    <p><b>Polynomial hood:</b> ${evaluation.hoodPrediction === null ? 'N/A for passback' : `${evaluation.hoodPrediction.toFixed(3)} rot`}</p>
    <p><b>Roller:</b> ${candidate ? `${candidate.rollerRps.toFixed(1)} rps` : 'N/A'} / cap ${target.config.maxRollerRps.toFixed(0)} rps</p>
    <p><b>Polynomial roller:</b> ${evaluation.rollerPrediction === null ? 'N/A for passback' : `${evaluation.rollerPrediction.toFixed(1)} rps`}</p>
    <p><b>Flight time:</b> ${candidate ? `${candidate.flightTimeSec.toFixed(2)} s` : 'N/A'}</p>
    <p><b>Target margin:</b> ${candidate ? `${candidate.heightMarginMeters.toFixed(3)} m` : 'N/A'}</p>
    <p><b>Turret-compensated path:</b> ${turretSideProfileSummary(evaluation.turretSideProfile)}</p>
    <p><b>Reachable pose:</b> ${evaluation.reachable ? 'yes' : 'blocked by field geometry'}</p>
    <p><b>Tilt input:</b> pitch ${value('pitch').toFixed(1)} deg, roll ${value('roll').toFixed(1)} deg</p>
    <h3>Turret</h3>
    <p><b>Desired field yaw:</b> ${evaluation.turretSolution.desiredFieldYawDeg.toFixed(1)} deg; stationary ${evaluation.turretSolution.stationaryFieldYawDeg.toFixed(1)} deg; lead ${evaluation.turretSolution.leadYawDeg.toFixed(1)} deg</p>
    <p><b>Desired turret:</b> raw ${evaluation.turretSolution.desiredRawTurretYawDeg.toFixed(1)} deg; legal goal ${evaluation.turretSolution.legalGoalTurretYawDeg.toFixed(1)} deg; field goal ${evaluation.turretSolution.legalGoalFieldYawDeg.toFixed(1)} deg</p>
    <p><b>Sim current:</b> ${evaluation.turretTracking.currentAngleDeg.toFixed(1)} deg; aim error ${evaluation.turretTracking.aimErrorDeg.toFixed(1)} deg; tolerance ${evaluation.turretTracking.toleranceDeg.toFixed(1)} deg</p>
    <p><b>Sim velocity:</b> ${(turretSim.velocityDegPerSec / 360.0).toFixed(2)} rps; voltage ${turretSim.voltage.toFixed(1)} V; current ${turretSim.estimatedCurrentA.toFixed(0)} A; time-to-ready ${turretSim.timeToReadySec.toFixed(2)} s</p>
    <p><b>Effective limits:</b> ${(effectiveTurretVelocityRps()).toFixed(2)} rps velocity, ${(effectiveTurretAccelerationRps2()).toFixed(1)} rps² accel after ${value('turretEffectiveMotionScale').toFixed(0)}% scale and current derate</p>
    <p><b>Turret status:</b> <span class="${evaluation.turretTracking.ready ? 'good' : 'bad'}">${evaluation.turretTracking.ready ? (evaluation.turretTracking.simEnabled ? 'tracking ready' : 'math valid; sim not gating') : escapeHtml(evaluation.turretTracking.reason)}</span></p>
    <p><b>Tracking risk:</b> ${turretSim.trackingRiskReason ? `<span class="bad">${escapeHtml(turretSim.trackingRiskReason)}</span>` : '<span class="good">none</span>'}</p>
    <p><b>Tilt-corrected local pitch:</b> ${evaluation.turretSolution.localLaunchPitchDeg.toFixed(1)} deg; zero offset ${value('turretZeroOffset').toFixed(1)} deg</p>
    <h3>Envelope Score</h3>
    <p><b>Goal:</b> 100% solvable with worst-case estimated accuracy >= ${value('desiredAccuracy').toFixed(0)}%.</p>
    <table>
      <thead><tr><th>Speed</th><th>Accel</th><th>Solve</th><th>Turret</th><th>Worst Acc</th><th>Score</th></tr></thead>
      <tbody>${thresholdRows.map((row) => `
        <tr class="${row.qualified ? 'qualified-row' : ''}">
          <td>${row.speedCapMps.toFixed(2)}</td>
          <td>${row.accelCapMps2.toFixed(1)}</td>
          <td>${(row.solveRate * 100).toFixed(0)}%</td>
          <td>${(row.turretReadyRate * 100).toFixed(0)}% / ${row.turretP95ErrorDeg.toFixed(1)}°</td>
          <td>${(row.minAccuracy * 100).toFixed(0)}%</td>
          <td>${row.score.toFixed(2)}</td>
        </tr>
      `).join('')}</tbody>
    </table>
  `;
}

function turretSideProfileSummary(profile: TurretSideProfile | null): string {
  if (!profile) return 'N/A';
  const status = profile.targetOpeningValid
    ? '<span class="good">intersects opening</span>'
    : '<span class="bad">misses opening</span>';
  return `${status}; radial field ${profile.radialFieldSpeedMps.toFixed(2)} m/s, cross-track ${profile.crossTrackAtTargetMeters.toFixed(3)} m, impact ${profile.impactHeightMeters.toFixed(2)} m, flight ${profile.flightTimeToTargetSec.toFixed(2)} s`;
}

function effectiveTurretVelocityRps(): number {
  return value('turretMaxVelocity') * clamp(value('turretEffectiveMotionScale') / 100.0, 0.05, 1.0);
}

function effectiveTurretAccelerationRps2(): number {
  const supplyCurrent = Math.max(0.1, value('turretSupplyCurrentLimit'));
  const statorCurrent = Math.max(supplyCurrent, value('turretStatorCurrentLimit'));
  const currentScale = clamp(supplyCurrent / statorCurrent, 0.12, 1.0);
  return value('turretMaxAcceleration') * clamp(value('turretEffectiveMotionScale') / 100.0, 0.05, 1.0) * currentScale;
}

function drawField(evaluation: ShotEvaluation): void {
  const context = fieldCanvas.getContext('2d');
  if (!context) return;
  context.clearRect(0, 0, fieldCanvas.width, fieldCanvas.height);
  context.fillStyle = '#0a1512';
  context.fillRect(0, 0, fieldCanvas.width, fieldCanvas.height);

  const bounds = fieldBounds();
  context.fillStyle = 'rgba(15, 23, 42, 0.85)';
  context.fillRect(0, 0, fieldCanvas.width, fieldCanvas.height);
  context.fillStyle = '#10251d';
  context.fillRect(bounds.x, bounds.y, bounds.width, bounds.height);

  drawZones(context, bounds);
  drawGrid(context, bounds);
  for (const element of fieldElements) drawFieldElement(context, element);
  drawImportedPath(context);

  drawAimPoint(context, field.blueHubAim, '#bfdbfe', 'Blue HUB center');
  drawAimPoint(context, field.redHubAim, '#fecdd3', 'Red HUB center');
  drawPassTarget(context, field.blueBackUpper, '#38bdf8', 'B back upper', evaluation.target.point === field.blueBackUpper, 'triangle');
  drawPassTarget(context, field.blueBackLower, '#38bdf8', 'B back lower', evaluation.target.point === field.blueBackLower, 'triangle');
  drawPassTarget(context, field.redBackUpper, '#fb7185', 'R back upper', evaluation.target.point === field.redBackUpper, 'triangle');
  drawPassTarget(context, field.redBackLower, '#fb7185', 'R back lower', evaluation.target.point === field.redBackLower, 'triangle');
  drawPassTarget(context, field.opponentPassBackUpper, '#34d399', 'opp pass upper', evaluation.target.point === field.opponentPassBackUpper, 'diamond');
  drawPassTarget(context, field.opponentPassBackLower, '#34d399', 'opp pass lower', evaluation.target.point === field.opponentPassBackLower, 'diamond');

  const shooterCanvas = fieldToCanvas(evaluation.release.shooterPoint);
  const targetCanvas = fieldToCanvas(evaluation.target.point);
  context.strokeStyle = evaluation.shotReady ? evaluation.target.color : '#ef4444';
  context.lineWidth = 4;
  line(context, shooterCanvas.x, shooterCanvas.y, targetCanvas.x, targetCanvas.y);
  drawTurretOverlay(context, evaluation);
  drawRobot(context, evaluation.base.pose, evaluation.release.currentShooterPoint, false);
  if (distance(evaluation.base.pose, evaluation.release.predictedPose) > 0.03
    || Math.abs(evaluation.base.pose.headingRad - evaluation.release.predictedPose.headingRad) > 0.02) {
    drawRobot(context, evaluation.release.predictedPose, evaluation.release.shooterPoint, true);
  }

  context.fillStyle = '#f7f2e8';
  context.font = '600 22px Space Grotesk, sans-serif';
  context.fillText('2026 REBUILT Field - reachability, path playback, and shooter offset', bounds.x, 34);
  context.font = '500 15px IBM Plex Mono, monospace';
  context.fillText(
    `${pageModeLabel(appState.pageMode)} | ${evaluation.target.shortLabel} | distance ${evaluation.distanceMeters.toFixed(2)}m | radial ${evaluation.radialVelocityMps.toFixed(2)}m/s | ${evaluation.shotReady ? 'ready' : 'not ready'}`,
    bounds.x,
    fieldCanvas.height - 24,
  );
  drawLegend(context, bounds);
}

function drawImportedPath(context: CanvasRenderingContext2D): void {
  if (!appState.loadedPath || appState.loadedPath.samples.length < 2) return;
  const overlay = pathOverlayStatus();
  const samples = overlay.samples;
  for (let i = 1; i < samples.length; i += 1) {
    const previous = fieldToCanvas(samples[i - 1].pose);
    const current = fieldToCanvas(samples[i].pose);
    const ready = overlay.ready[i];
    context.strokeStyle = ready ? 'rgba(34, 197, 94, 0.95)' : 'rgba(239, 68, 68, 0.88)';
    context.lineWidth = 4;
    line(context, previous.x, previous.y, current.x, current.y);
  }
  const sample = sampleLoadedPath(appState.loadedPath, appState.pathTimeSec);
  const marker = fieldToCanvas(sample.pose);
  context.fillStyle = '#f8c537';
  context.beginPath();
  context.arc(marker.x, marker.y, 8, 0, Math.PI * 2);
  context.fill();
}

function drawZones(context: CanvasRenderingContext2D, bounds: FieldBounds): void {
  context.fillStyle = 'rgba(59, 130, 246, 0.14)';
  context.fillRect(bounds.x, bounds.y, field.allianceZoneDepthMeters * bounds.scale, bounds.height);
  context.fillStyle = 'rgba(239, 68, 68, 0.14)';
  context.fillRect(
    bounds.x + (field.lengthMeters - field.allianceZoneDepthMeters) * bounds.scale,
    bounds.y,
    field.allianceZoneDepthMeters * bounds.scale,
    bounds.height,
  );
  context.strokeStyle = 'rgba(248, 197, 55, 0.7)';
  context.lineWidth = 2;
  line(context, bounds.x + field.centerLineX * bounds.scale, bounds.y, bounds.x + field.centerLineX * bounds.scale, bounds.y + bounds.height);
  context.strokeStyle = '#dbe7d4';
  context.lineWidth = 3;
  context.strokeRect(bounds.x, bounds.y, bounds.width, bounds.height);
}

function drawGrid(context: CanvasRenderingContext2D, bounds: FieldBounds): void {
  context.strokeStyle = 'rgba(219, 231, 212, 0.16)';
  context.lineWidth = 1;
  for (let x = 0; x <= field.lengthMeters; x += 1) {
    const px = bounds.x + x * bounds.scale;
    line(context, px, bounds.y, px, bounds.y + bounds.height);
  }
  for (let y = 0; y <= field.widthMeters; y += 1) {
    const py = bounds.y + bounds.height - y * bounds.scale;
    line(context, bounds.x, py, bounds.x + bounds.width, py);
  }
}

function drawFieldElement(context: CanvasRenderingContext2D, element: FieldElement): void {
  const topLeft = fieldToCanvas({ x: element.x, y: element.y + element.height });
  const bottomRight = fieldToCanvas({ x: element.x + element.width, y: element.y });
  const width = bottomRight.x - topLeft.x;
  const height = bottomRight.y - topLeft.y;

  if (element.kind === 'ramp') {
    context.fillStyle = element.alliance === 'red' ? 'rgba(248, 113, 113, 0.42)' : 'rgba(96, 165, 250, 0.42)';
  } else if (element.kind === 'low-clearance') {
    context.fillStyle = 'rgba(245, 158, 11, 0.28)';
  } else if (element.kind === 'field-marker') {
    context.fillStyle = 'rgba(148, 163, 184, 0.18)';
  } else {
    context.fillStyle = 'rgba(17, 24, 39, 0.82)';
  }

  context.strokeStyle = isBlockingElement(element) ? '#ef4444' : 'rgba(247, 242, 232, 0.6)';
  context.lineWidth = isBlockingElement(element) ? 2.5 : 1.5;
  context.fillRect(topLeft.x, topLeft.y, width, height);
  context.strokeRect(topLeft.x, topLeft.y, width, height);

  if (element.kind === 'low-clearance') {
    context.strokeStyle = 'rgba(15, 23, 42, 0.55)';
    context.lineWidth = 1;
    for (let i = -height; i < width; i += 10) {
      line(context, topLeft.x + i, topLeft.y + height, topLeft.x + i + height, topLeft.y);
    }
  }

  context.fillStyle = '#f7f2e8';
  context.font = '500 11px IBM Plex Mono, monospace';
  context.fillText(shortElementName(element.name), topLeft.x + 5, topLeft.y + 15);
}

function drawTurretOverlay(context: CanvasRenderingContext2D, evaluation: ShotEvaluation): void {
  const pivot = evaluation.release.shooterPoint;
  const predictedHeadingDeg = radiansToDegrees(evaluation.release.predictedPose.headingRad);
  const zeroOffsetDeg = value('turretZeroOffset');
  const currentFieldYawDeg = predictedHeadingDeg + evaluation.turretTracking.currentAngleDeg + zeroOffsetDeg;

  drawSoftLimitArc(context, pivot, predictedHeadingDeg + zeroOffsetDeg);
  drawFieldRay(context, pivot, evaluation.turretSolution.stationaryFieldYawDeg, 1.45, 'rgba(219, 231, 212, 0.52)', 2, [8, 8]);
  drawFieldRay(context, pivot, evaluation.turretSolution.desiredFieldYawDeg, 1.55, evaluation.turretSolution.valid ? '#f8c537' : '#fb7185', 4);
  drawFieldRay(context, pivot, evaluation.turretSolution.legalGoalFieldYawDeg, 1.02, '#fb923c', 5);
  drawFieldRay(context, pivot, currentFieldYawDeg, 0.82, evaluation.turretTracking.ready ? '#38bdf8' : '#ef4444', 4);

  const pivotCanvas = fieldToCanvas(pivot);
  context.fillStyle = '#101820';
  context.beginPath();
  context.arc(pivotCanvas.x, pivotCanvas.y, 6, 0, Math.PI * 2);
  context.fill();
  context.strokeStyle = '#f7f2e8';
  context.lineWidth = 1.5;
  context.stroke();

  context.fillStyle = '#dbe7d4';
  context.font = '500 11px IBM Plex Mono, monospace';
  context.fillText(
    `turret lead ${evaluation.turretSolution.leadYawDeg.toFixed(1)}° | error ${evaluation.turretTracking.aimErrorDeg.toFixed(1)}°`,
    pivotCanvas.x + 12,
    pivotCanvas.y + 18,
  );
}

function drawSoftLimitArc(context: CanvasRenderingContext2D, pivot: Point, robotZeroFieldYawDeg: number): void {
  const lowDeg = robotZeroFieldYawDeg + (defaultTurretConstants.softLimitLowRot * 360.0);
  const highDeg = robotZeroFieldYawDeg + (defaultTurretConstants.softLimitHighRot * 360.0);
  const spanDeg = highDeg - lowDeg;
  const radiusMeters = 0.72;
  context.save();
  context.strokeStyle = 'rgba(34, 197, 94, 0.44)';
  context.lineWidth = 2;
  context.setLineDash([5, 6]);
  if (Math.abs(spanDeg) >= 360.0) {
    const center = fieldToCanvas(pivot);
    const edge = fieldToCanvas({ x: pivot.x + radiusMeters, y: pivot.y });
    context.beginPath();
    context.arc(center.x, center.y, Math.abs(edge.x - center.x), 0, Math.PI * 2);
    context.stroke();
  } else {
    context.beginPath();
    for (let i = 0; i <= 96; i += 1) {
      const angleDeg = lowDeg + ((spanDeg * i) / 96);
      const point = pointFromAngle(pivot, angleDeg, radiusMeters);
      const canvasPoint = fieldToCanvas(point);
      if (i === 0) context.moveTo(canvasPoint.x, canvasPoint.y);
      else context.lineTo(canvasPoint.x, canvasPoint.y);
    }
    context.stroke();
  }
  context.restore();
  drawFieldRay(context, pivot, lowDeg, radiusMeters, 'rgba(34, 197, 94, 0.52)', 1.5, [2, 5]);
  drawFieldRay(context, pivot, highDeg, radiusMeters, 'rgba(34, 197, 94, 0.52)', 1.5, [2, 5]);
}

function drawFieldRay(
  context: CanvasRenderingContext2D,
  origin: Point,
  angleDeg: number,
  lengthMeters: number,
  color: string,
  width: number,
  dash: number[] = [],
): void {
  const start = fieldToCanvas(origin);
  const end = fieldToCanvas(pointFromAngle(origin, angleDeg, lengthMeters));
  context.save();
  context.strokeStyle = color;
  context.lineWidth = width;
  context.setLineDash(dash);
  line(context, start.x, start.y, end.x, end.y);
  context.restore();
}

function pointFromAngle(origin: Point, angleDeg: number, lengthMeters: number): Point {
  const angleRad = degreesToRadians(angleDeg);
  return {
    x: origin.x + (Math.cos(angleRad) * lengthMeters),
    y: origin.y + (Math.sin(angleRad) * lengthMeters),
  };
}

function drawRobot(context: CanvasRenderingContext2D, pose: Pose2d, shooterPoint: Point, ghost: boolean): void {
  context.save();
  context.globalAlpha = ghost ? 0.45 : 1.0;
  const footprint = robotFootprint(pose).map(fieldToCanvas);
  context.beginPath();
  footprint.forEach((point, index) => {
    if (index === 0) context.moveTo(point.x, point.y);
    else context.lineTo(point.x, point.y);
  });
  context.closePath();
  context.fillStyle = isReachablePose(pose) ? (ghost ? '#93c5fd' : '#f8c537') : '#ef4444';
  context.strokeStyle = '#101820';
  context.lineWidth = 3;
  context.fill();
  context.stroke();

  const center = fieldToCanvas(pose);
  const front = fieldToCanvas({
    x: pose.x + (Math.cos(pose.headingRad) * robotGeometry.lengthMeters / 2.0),
    y: pose.y + (Math.sin(pose.headingRad) * robotGeometry.lengthMeters / 2.0),
  });
  context.strokeStyle = '#101820';
  context.lineWidth = 4;
  line(context, center.x, center.y, front.x, front.y);

  const shooter = fieldToCanvas(shooterPoint);
  context.fillStyle = '#101820';
  context.beginPath();
  context.arc(shooter.x, shooter.y, 5, 0, Math.PI * 2);
  context.fill();
  context.strokeStyle = '#f7f2e8';
  context.lineWidth = 1.5;
  context.beginPath();
  context.arc(shooter.x, shooter.y, 8, 0, Math.PI * 2);
  context.stroke();
  if (ghost) {
    context.fillStyle = '#bfdbfe';
    context.font = '600 11px IBM Plex Mono, monospace';
    context.fillText('predicted release', center.x + 10, center.y - 10);
  }
  context.restore();
}

function drawAimPoint(context: CanvasRenderingContext2D, point: Point, color: string, label: string): void {
  const canvasPoint = fieldToCanvas(point);
  context.strokeStyle = color;
  context.lineWidth = 2;
  line(context, canvasPoint.x - 9, canvasPoint.y, canvasPoint.x + 9, canvasPoint.y);
  line(context, canvasPoint.x, canvasPoint.y - 9, canvasPoint.x, canvasPoint.y + 9);
  context.fillStyle = color;
  context.font = '500 11px IBM Plex Mono, monospace';
  context.fillText(label, canvasPoint.x + 10, canvasPoint.y - 8);
}

function drawPassTarget(
  context: CanvasRenderingContext2D,
  point: Point,
  color: string,
  label: string,
  selected: boolean,
  shape: 'triangle' | 'diamond',
): void {
  const canvasPoint = fieldToCanvas(point);
  context.save();
  context.translate(canvasPoint.x, canvasPoint.y);
  context.globalAlpha = selected ? 1.0 : 0.65;
  context.fillStyle = color;
  context.strokeStyle = selected ? '#f8c537' : 'rgba(247, 242, 232, 0.7)';
  context.lineWidth = selected ? 4 : 2;
  context.beginPath();
  if (shape === 'triangle') {
    context.moveTo(0, -13);
    context.lineTo(13, 11);
    context.lineTo(-13, 11);
  } else {
    context.moveTo(0, -14);
    context.lineTo(14, 0);
    context.lineTo(0, 14);
    context.lineTo(-14, 0);
  }
  context.closePath();
  context.fill();
  context.stroke();
  context.restore();
  context.fillStyle = '#dbe7d4';
  context.font = '500 11px IBM Plex Mono, monospace';
  context.fillText(label, canvasPoint.x + 14, canvasPoint.y + 5);
}

function drawLegend(context: CanvasRenderingContext2D, bounds: FieldBounds): void {
  const x = bounds.x + bounds.width - 342;
  const y = 24;
  context.fillStyle = 'rgba(16, 24, 32, 0.78)';
  context.fillRect(x - 12, y - 16, 356, 154);
  context.fillStyle = '#dbe7d4';
  context.font = '500 12px IBM Plex Mono, monospace';
  context.fillText('red outline = hard collision/no-reach', x, y);
  context.fillText('amber hatch = TRENCH low clearance, not blocking', x, y + 20);
  context.fillText('blue/red translucent = BUMP ramp', x, y + 40);
  context.fillText('crosshair = constants HUB center aim point', x, y + 60);
  context.fillText('path green/red = threshold + solver status', x, y + 80);
  context.fillText('blue ghost robot = predicted release pose', x, y + 100);
  context.fillText('yellow/orange/cyan = desired/goal/current turret', x, y + 120);
}

function drawPlot(evaluation: ShotEvaluation): void {
  const candidate = evaluation.candidate;
  const config = evaluation.target.config;
  const profile = evaluation.turretSideProfile;
  const context = plot.getContext('2d');
  if (!context) return;
  context.clearRect(0, 0, plot.width, plot.height);
  context.fillStyle = '#101820';
  context.fillRect(0, 0, plot.width, plot.height);

  const chart = {
    left: 34,
    top: 26,
    right: 22,
    bottom: 28,
  };
  const chartWidth = plot.width - chart.left - chart.right;
  const chartHeight = plot.height - chart.top - chart.bottom;
  const hubFrontDistance = Math.max(0.0, evaluation.distanceMeters - (config.hubOpeningDepthMeters / 2.0));
  const hubBackDistance = evaluation.distanceMeters + (config.hubOpeningDepthMeters / 2.0);
  const plotEndDistance = Math.max(8.0, Math.ceil(Math.max(evaluation.distanceMeters, hubBackDistance) + 1.0));
  const maxHeightMeters = Math.max(
    3.0,
    config.targetHeightMeters + config.goalRadiusMeters + 0.9,
    (candidate?.peakHeightMeters ?? 0) + 0.4,
    (profile?.peakHeightMeters ?? 0) + 0.4,
  );
  const xToPlot = (xMeters: number) => chart.left + (xMeters / plotEndDistance) * chartWidth;
  const yToPlot = (yMeters: number) => chart.top + chartHeight - (yMeters / maxHeightMeters) * chartHeight;

  context.strokeStyle = '#334155';
  context.lineWidth = 1;
  for (let i = 0; i <= 10; i += 1) {
    const x = chart.left + (chartWidth * i) / 10;
    line(context, x, chart.top, x, chart.top + chartHeight);
  }
  for (let i = 0; i <= 5; i += 1) {
    const y = chart.top + (chartHeight * i) / 5;
    line(context, chart.left, y, chart.left + chartWidth, y);
  }

  drawTargetOpening(context, evaluation, xToPlot, yToPlot);

  context.fillStyle = '#dbe7d4';
  context.font = '500 14px IBM Plex Mono, monospace';
  context.fillText(
    config.requiresHubOpeningClearance
      ? 'HUB side view: turret-compensated 3D field trajectory + full opening depth'
      : 'Passback side view: turret-compensated 3D field trajectory',
    chart.left,
    24,
  );

  if (candidate) drawRadialOnlyReferencePath(context, evaluation, xToPlot, yToPlot, plotEndDistance);
  if (!profile || profile.radialFieldSpeedMps <= 0.05) return;

  drawSideTurretLauncher(context, config, profile, xToPlot, yToPlot);
  context.strokeStyle = profile.targetOpeningValid ? '#f8c537' : '#fb7185';
  context.lineWidth = 4;
  context.beginPath();
  for (let i = 0; i <= 144; i += 1) {
    const xMeters = (plotEndDistance * i) / 144;
    const t = xMeters / profile.radialFieldSpeedMps;
    const yMeters = heightAtTime(config, profile.verticalLaunchSpeedMps, t);
    const cx = xToPlot(xMeters);
    const cy = yToPlot(yMeters);
    if (i === 0) context.moveTo(cx, cy);
    else context.lineTo(cx, cy);
  }
  context.stroke();

  drawOpeningSampleMarkers(context, evaluation, profile, xToPlot, yToPlot);
  context.fillStyle = profile.targetOpeningValid ? '#f8c537' : '#fb7185';
  context.beginPath();
  context.arc(xToPlot(evaluation.distanceMeters), yToPlot(profile.impactHeightMeters), 6, 0, Math.PI * 2);
  context.fill();

  context.fillStyle = '#dbe7d4';
  context.font = '500 12px IBM Plex Mono, monospace';
  context.fillText(
    `solid = turret-compensated | dashed = radial-only map | cross-track @ target ${profile.crossTrackAtTargetMeters.toFixed(3)} m`,
    chart.left,
    chart.top + chartHeight - 10,
  );
}

function drawRadialOnlyReferencePath(
  context: CanvasRenderingContext2D,
  evaluation: ShotEvaluation,
  xToPlot: (xMeters: number) => number,
  yToPlot: (yMeters: number) => number,
  plotEndDistance: number,
): void {
  const candidate = evaluation.candidate;
  if (!candidate) return;
  const config = evaluation.target.config;
  const launchAngleRad = degreesToRadians(candidate.launchAngleDeg);
  const targetLineSpeed = (candidate.exitSpeedMps * Math.cos(launchAngleRad)) + evaluation.radialVelocityMps;
  if (targetLineSpeed <= 0.05) return;
  context.save();
  context.strokeStyle = 'rgba(219, 231, 212, 0.36)';
  context.lineWidth = 2;
  context.setLineDash([8, 8]);
  context.beginPath();
  for (let i = 0; i <= 144; i += 1) {
    const xMeters = (plotEndDistance * i) / 144;
    const t = xMeters / targetLineSpeed;
    const yMeters = heightAtTime(
      config,
      candidate.exitSpeedMps * Math.sin(launchAngleRad),
      t,
    );
    const cx = xToPlot(xMeters);
    const cy = yToPlot(yMeters);
    if (i === 0) context.moveTo(cx, cy);
    else context.lineTo(cx, cy);
  }
  context.stroke();
  context.restore();
}

function drawSideTurretLauncher(
  context: CanvasRenderingContext2D,
  config: ShotCalculatorConfig,
  profile: TurretSideProfile,
  xToPlot: (xMeters: number) => number,
  yToPlot: (yMeters: number) => number,
): void {
  const barrelLengthMeters = 0.55;
  const sideProjectionPitchRad = Math.atan2(profile.verticalLaunchSpeedMps, profile.radialLaunchSpeedMps);
  const startX = xToPlot(0.0);
  const startY = yToPlot(config.shooterExitHeightMeters);
  const endX = xToPlot(Math.cos(sideProjectionPitchRad) * barrelLengthMeters);
  const endY = yToPlot(config.shooterExitHeightMeters + (Math.sin(sideProjectionPitchRad) * barrelLengthMeters));
  context.save();
  context.strokeStyle = '#fb923c';
  context.lineWidth = 5;
  line(context, startX, startY, endX, endY);
  context.fillStyle = '#fb923c';
  context.beginPath();
  context.arc(startX, startY, 5, 0, Math.PI * 2);
  context.fill();
  context.restore();
}

function drawOpeningSampleMarkers(
  context: CanvasRenderingContext2D,
  evaluation: ShotEvaluation,
  profile: TurretSideProfile,
  xToPlot: (xMeters: number) => number,
  yToPlot: (yMeters: number) => number,
): void {
  const config = evaluation.target.config;
  const lowerY = config.targetHeightMeters - config.goalRadiusMeters;
  const upperY = config.targetHeightMeters + config.goalRadiusMeters;
  for (const sample of profile.openingSamples) {
    const inOpening = sample.heightMeters >= lowerY
      && sample.heightMeters <= upperY
      && Math.abs(sample.crossTrackMeters) <= config.goalRadiusMeters;
    context.fillStyle = inOpening ? '#22c55e' : '#ef4444';
    context.beginPath();
    context.arc(xToPlot(sample.distanceMeters), yToPlot(sample.heightMeters), 3.5, 0, Math.PI * 2);
    context.fill();
  }
}

function drawTargetOpening(
  context: CanvasRenderingContext2D,
  evaluation: ShotEvaluation,
  xToPlot: (xMeters: number) => number,
  yToPlot: (yMeters: number) => number,
): void {
  const config = evaluation.target.config;
  const lowerY = config.targetHeightMeters - config.goalRadiusMeters;
  const upperY = config.targetHeightMeters + config.goalRadiusMeters;
  const frontX = config.requiresHubOpeningClearance
    ? Math.max(0.0, evaluation.distanceMeters - (config.hubOpeningDepthMeters / 2.0))
    : Math.max(0.0, evaluation.distanceMeters - 0.18);
  const backX = config.requiresHubOpeningClearance
    ? evaluation.distanceMeters + (config.hubOpeningDepthMeters / 2.0)
    : evaluation.distanceMeters + 0.18;
  const left = xToPlot(frontX);
  const right = xToPlot(backX);
  const top = yToPlot(upperY);
  const bottom = yToPlot(lowerY);

  context.fillStyle = 'rgba(34, 197, 94, 0.14)';
  context.fillRect(left, top, right - left, bottom - top);
  context.strokeStyle = '#22c55e';
  context.lineWidth = 3;
  context.strokeRect(left, top, right - left, bottom - top);

  const targetY = yToPlot(config.targetHeightMeters);
  context.strokeStyle = 'rgba(239, 68, 68, 0.86)';
  context.lineWidth = 2;
  line(context, xToPlot(0), targetY, xToPlot(backX + 0.5), targetY);

  if (config.requiresHubOpeningClearance) {
    const lipHeightMeters = config.hubFrontLipHeightMeters + config.fuelRadiusMeters + config.hubLipClearanceMeters;
    context.strokeStyle = 'rgba(248, 113, 113, 0.9)';
    context.lineWidth = 6;
    line(context, xToPlot(frontX), yToPlot(0), xToPlot(frontX), yToPlot(lipHeightMeters));
  }
}

function thresholdRecommendationRows(forceCompute = false): ThresholdRecommendation[] {
  const cacheKey = thresholdRowsCacheKey();
  if (cachedThresholdRows?.key === cacheKey) return cachedThresholdRows.rows;
  if (!forceCompute && cachedThresholdRows) return cachedThresholdRows.rows;

  const speedCaps = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, driveLimits.maxFreeSpeedMps];
  const accelCaps = [2.0, 3.0, 4.5, 6.0, 8.0];
  const requiredAccuracy = value('desiredAccuracy') / 100.0;
  const turretConfig = capsTurretControllerConfig();
  const rows: ThresholdRecommendation[] = [];
  for (const speedCapMps of speedCaps) {
    for (const accelCapMps2 of accelCaps) {
      const samples = thresholdSamplesForCaps(speedCapMps, accelCapMps2);
      const consideredSamples = samples.filter((sample) => sample.considered);
      const turretScore = scoreTurretForCaps(turretConfig, speedCapMps, accelCapMps2);
      const solvedSamples = consideredSamples.filter((sample) => sample.shotSolvable);
      const solved = solvedSamples.length;
      const solveRate = consideredSamples.length > 0 ? solved / consideredSamples.length : 0;
      const minAccuracy = solvedSamples.length > 0
        ? Math.min(...solvedSamples.map((sample) => sample.accuracyEstimate))
        : 0.0;
      const accuracyPassRate = consideredSamples.length > 0
        ? consideredSamples.filter((sample) => sample.shotSolvable && sample.accuracyEstimate >= requiredAccuracy).length / consideredSamples.length
        : 0.0;
      const turretQualified = turretScore.readyRate >= 0.99
        && turretScore.p95AbsErrorDeg <= turretConfig.aimToleranceDeg * 2.0;
      const qualified = solveRate >= 0.999 && minAccuracy >= requiredAccuracy && turretQualified;
      const speedScore = speedCapMps / driveLimits.maxFreeSpeedMps;
      const accelScore = accelCapMps2 / 8.0;
      const freedomScore = (speedScore * 0.72) + (accelScore * 0.28);
      const turretReliability = clamp(
        (turretScore.readyRate * 0.72)
          + ((1.0 - clamp(turretScore.p95AbsErrorDeg / Math.max(turretConfig.aimToleranceDeg * 6.0, 1e-9), 0.0, 1.0)) * 0.28),
        0.0,
        1.0,
      );
      const reliabilityScore = (solveRate * 0.40)
        + (accuracyPassRate * 0.25)
        + (minAccuracy * 0.15)
        + (turretReliability * 0.20);
      rows.push({
        speedCapMps,
        accelCapMps2,
        solveRate,
        minAccuracy,
        accuracyPassRate,
        turretReadyRate: turretScore.readyRate,
        turretP95ErrorDeg: turretScore.p95AbsErrorDeg,
        qualified,
        score: qualified
          ? 1.0 + freedomScore
          : (reliabilityScore * 0.90) + (freedomScore * 0.10),
      });
    }
  }
  const rankedRows = rows
    .sort(compareThresholdRows)
    .slice(0, 6);
  cachedThresholdRows = { key: cacheKey, rows: rankedRows };
  return rankedRows;
}

function thresholdSamplesForCaps(speedCapMps: number, accelCapMps2: number): ThresholdSample[] {
  if (appState.pageMode === 'path' && appState.loadedPath) {
    return downsample(appState.loadedPath.samples, 24).map((sample) => thresholdSampleForBase(
      baseMotionFromPathSampleForCaps(sample, speedCapMps, accelCapMps2),
      speedCapMps,
      accelCapMps2,
    ));
  }
  return envelopeSamplesForCaps(speedCapMps, accelCapMps2).map((sample) => thresholdSampleForBase(
    sample,
    speedCapMps,
    accelCapMps2,
  ));
}

function baseMotionFromPathSampleForCaps(
  sample: PathSample,
  speedCapMps: number,
  accelCapMps2: number,
): BaseMotionState {
  const scale = motionLimitScaleForSampleWithCaps(sample, speedCapMps, accelCapMps2);
  const accelerationScale = scale * scale;
  return {
    pose: sample.pose,
    velocity: scalePoint(sample.velocity, scale),
    acceleration: scalePoint(sample.acceleration, accelerationScale),
    omegaRadPerSec: sample.omegaRadPerSec * scale,
    angularAccelRadPerSec2: sample.angularAccelRadPerSec2 * accelerationScale,
    source: 'path',
  };
}

function thresholdSampleForBase(base: BaseMotionState, speedCapMps: number, accelCapMps2: number): ThresholdSample {
  const release = predictedReleaseState(base);
  const threshold = evaluateThresholds(release);
  const underCaps = threshold.speedMps <= speedCapMps
    && threshold.accelerationMps2 <= accelCapMps2
    && threshold.omegaDegPerSec <= value('shotOmegaCap')
    && threshold.alphaDegPerSec2 <= value('shotAlphaCap');
  const considered = isReachablePose(base.pose) && isReachablePose(release.predictedPose) && underCaps;
  const quality = considered ? fastShotQualityForBase(base) : { solvable: false, accuracyEstimate: 0.0 };
  return {
    considered,
    speedMps: threshold.speedMps,
    accelerationMps2: threshold.accelerationMps2,
    omegaDegPerSec: threshold.omegaDegPerSec,
    alphaDegPerSec2: threshold.alphaDegPerSec2,
    shotSolvable: considered && quality.solvable,
    accuracyEstimate: considered ? quality.accuracyEstimate : 0.0,
  };
}

function envelopeSamplesForCaps(speedCapMps: number, accelCapMps2: number): BaseMotionState[] {
  const pose = currentPose();
  const samples: BaseMotionState[] = [{
    pose,
    velocity: { x: 0, y: 0 },
    acceleration: { x: 0, y: 0 },
    omegaRadPerSec: 0,
    angularAccelRadPerSec2: 0,
    source: appState.pageMode,
  }];
  const velocityAngles = directionAngles(8);
  const accelAngles = directionAngles(4);
  for (const speed of [speedCapMps * 0.5, speedCapMps]) {
    for (const velocityAngle of velocityAngles) {
      samples.push({
        pose,
        velocity: polarPoint(speed, velocityAngle),
        acceleration: { x: 0, y: 0 },
        omegaRadPerSec: 0,
        angularAccelRadPerSec2: 0,
        source: appState.pageMode,
      });
    }
  }
  for (const velocityAngle of velocityAngles) {
    for (const accelAngle of accelAngles) {
      samples.push({
        pose,
        velocity: polarPoint(speedCapMps, velocityAngle),
        acceleration: polarPoint(accelCapMps2, accelAngle),
        omegaRadPerSec: 0,
        angularAccelRadPerSec2: 0,
        source: appState.pageMode,
      });
    }
  }
  return samples;
}

function fastShotQualityForBase(base: BaseMotionState): ShotQualityEstimate {
  const release = predictedReleaseState(base);
  const target = selectedTarget(appState.alliance, appState.targetMode, release.predictedPose);
  if (!isReachablePose(base.pose) || !isReachablePose(release.predictedPose)) {
    return { solvable: false, accuracyEstimate: 0.0 };
  }

  const radialTangential = radialTangentialVelocity(release.shooterPoint, target.point, release.shooterVelocity);
  const distanceMeters = distance(release.shooterPoint, target.point);
  if (target.usesPolynomial) {
    const input = { distanceMeters, radialVelocityMps: radialTangential.radialVelocityMps };
    if (!isInsideShotMapDomain(target.config, input) && isCloseRangeHubInput(target.config, input)) {
      const candidate = findBestCandidateCached(target.config, input, target.candidateSelection, true);
      return candidate
        ? { solvable: true, accuracyEstimate: estimateCandidateAccuracy(target.config, candidate) }
        : { solvable: false, accuracyEstimate: 0.0 };
    }
    if (!isInsideShotMapDomain(target.config, input)) return { solvable: false, accuracyEstimate: 0.0 };
    const hood = evaluatePolynomial(generatedCoefficients.hood, input);
    const roller = evaluatePolynomial(generatedCoefficients.roller, input);
    if (hood < target.config.minHoodRot
      || hood > target.config.maxHoodRot
      || Math.abs(roller) > target.config.maxRollerRps) {
      return { solvable: false, accuracyEstimate: 0.0 };
    }
    return {
      solvable: true,
      accuracyEstimate: estimatePolynomialAccuracy(target.config, distanceMeters, radialTangential.radialVelocityMps, hood, roller),
    };
  }

  return {
    solvable: isInsideShotMapDomain(target.config, { distanceMeters, radialVelocityMps: radialTangential.radialVelocityMps }),
    accuracyEstimate: estimatePassbackAccuracy(target.config, distanceMeters, radialTangential.radialVelocityMps),
  };
}

function estimateCandidateAccuracy(config: ShotCalculatorConfig, candidate: TrajectoryCandidate): number {
  const marginScore = clamp(candidate.heightMarginMeters / Math.max(config.goalRadiusMeters, 1e-9), 0.0, 1.0);
  const robustnessScore = clamp(candidate.robustnessMargin / 0.30, 0.0, 1.0);
  const rollerUse = clamp(Math.abs(candidate.rollerRps) / config.maxRollerRps, 0.0, 1.0);
  return clamp(
    0.84
    + (0.08 * marginScore)
    + (0.04 * robustnessScore)
    - (0.025 * rollerUse),
    0.0,
    0.98,
  );
}

function findBestCandidateCached(
  config: ShotCalculatorConfig,
  input: { distanceMeters: number; radialVelocityMps: number },
  selection: TargetSelection['candidateSelection'],
  allowCloseRangeHubFallback = false,
): TrajectoryCandidate | null {
  const quantizedInput = {
    distanceMeters: roundToStep(input.distanceMeters, 0.05),
    radialVelocityMps: roundToStep(input.radialVelocityMps, 0.05),
  };
  const solverConfig = allowCloseRangeHubFallback && isCloseRangeHubInput(config, quantizedInput)
    ? {
      ...config,
      minDistanceMeters: closeRangeHubConfig.minDistanceMeters,
      maxDistanceMeters: config.minDistanceMeters,
    }
    : config;
  const key = [
    selection,
    solverConfig.targetHeightMeters.toFixed(2),
    solverConfig.requiresHubOpeningClearance ? 'hub' : 'pass',
    solverConfig.minDistanceMeters.toFixed(2),
    solverConfig.maxDistanceMeters.toFixed(2),
    quantizedInput.distanceMeters.toFixed(2),
    quantizedInput.radialVelocityMps.toFixed(2),
  ].join('|');
  if (candidateCache.has(key)) return candidateCache.get(key) ?? null;
  const candidate = findBestCandidate(solverConfig, quantizedInput, selection);
  candidateCache.set(key, candidate);
  if (candidateCache.size > 2000) candidateCache.clear();
  return candidate;
}

function isCloseRangeHubInput(
  config: ShotCalculatorConfig,
  input: { distanceMeters: number; radialVelocityMps: number },
): boolean {
  return config.requiresHubOpeningClearance
    && input.distanceMeters >= closeRangeHubConfig.minDistanceMeters
    && input.distanceMeters < config.minDistanceMeters
    && input.radialVelocityMps >= config.minRadialVelocityMps
    && input.radialVelocityMps <= config.maxRadialVelocityMps;
}

function estimatePassbackAccuracy(
  config: ShotCalculatorConfig,
  distanceMeters: number,
  radialVelocityMps: number,
): number {
  if (!isInsideShotMapDomain(config, { distanceMeters, radialVelocityMps })) return 0.0;
  const distanceHeadroom = domainHeadroom(distanceMeters, config.minDistanceMeters, config.maxDistanceMeters);
  const radialHeadroom = domainHeadroom(radialVelocityMps, config.minRadialVelocityMps, config.maxRadialVelocityMps);
  const distanceUse = clamp(distanceMeters / config.maxDistanceMeters, 0.0, 1.0);
  return clamp(
    0.84
    + (0.10 * Math.min(distanceHeadroom, radialHeadroom))
    - (0.03 * distanceUse),
    0.0,
    0.94,
  );
}

function estimatePolynomialAccuracy(
  config: ShotCalculatorConfig,
  distanceMeters: number,
  radialVelocityMps: number,
  hoodRot: number,
  rollerRps: number,
): number {
  const distanceHeadroom = domainHeadroom(distanceMeters, config.minDistanceMeters, config.maxDistanceMeters);
  const radialHeadroom = domainHeadroom(radialVelocityMps, config.minRadialVelocityMps, config.maxRadialVelocityMps);
  const rollerUse = clamp(Math.abs(rollerRps) / config.maxRollerRps, 0.0, 1.0);
  const radialUse = clamp(Math.abs(radialVelocityMps) / Math.max(Math.abs(config.minRadialVelocityMps), Math.abs(config.maxRadialVelocityMps)), 0.0, 1.0);
  return clamp(
    0.91
    + (0.08 * Math.min(distanceHeadroom, radialHeadroom))
    - (0.025 * rollerUse)
    - (0.025 * radialUse),
    0.0,
    0.99,
  );
}

function compareThresholdRows(a: ThresholdRecommendation, b: ThresholdRecommendation): number {
  if (a.qualified !== b.qualified) return a.qualified ? -1 : 1;
  if (a.qualified && b.qualified) {
    const freedomA = (a.speedCapMps / driveLimits.maxFreeSpeedMps) + (a.accelCapMps2 / 8.0);
    const freedomB = (b.speedCapMps / driveLimits.maxFreeSpeedMps) + (b.accelCapMps2 / 8.0);
    return freedomB - freedomA;
  }
  if (Math.abs(b.solveRate - a.solveRate) > 1e-9) return b.solveRate - a.solveRate;
  if (Math.abs(b.accuracyPassRate - a.accuracyPassRate) > 1e-9) return b.accuracyPassRate - a.accuracyPassRate;
  if (Math.abs(b.minAccuracy - a.minAccuracy) > 1e-9) return b.minAccuracy - a.minAccuracy;
  return b.score - a.score;
}

function fastPathSampleReady(sample: PathSample): boolean {
  const base = baseMotionFromPathSample(sample, checked('applyVisualLimiter'));
  const release = predictedReleaseState(base);
  const threshold = evaluateThresholds(release);
  return threshold.ready && fastShotQualityForBase(base).solvable;
}

function pathOverlayStatus(): { samples: PathSample[]; ready: boolean[] } {
  if (!appState.loadedPath) return { samples: [], ready: [] };
  const key = `${pathAnalysisKey()}|overlay`;
  if (cachedPathOverlay?.key === key) return cachedPathOverlay;
  const samples = downsample(appState.loadedPath.samples, 42);
  const ready = samples.map(fastPathSampleReady);
  cachedPathOverlay = { key, samples, ready };
  return cachedPathOverlay;
}

function visualPathTimeScale(sample: PathSample): number {
  if (!checked('applyVisualLimiter')) return 1.0;
  return motionLimitScaleForSample(sample);
}

function motionLimitScaleForSample(sample: PathSample): number {
  return motionLimitScaleForSampleWithCaps(sample, value('shotSpeedCap'), value('shotAccelCap'));
}

function motionLimitScaleForSampleWithCaps(
  sample: PathSample,
  speedCapMps: number,
  accelCapMps2: number,
): number {
  const speedScale = limitScale(vectorMagnitude(sample.velocity), Math.min(speedCapMps, driveLimits.maxFreeSpeedMps));
  const accelScale = Math.sqrt(limitScale(vectorMagnitude(sample.acceleration), Math.min(accelCapMps2, driveLimits.maxLinearAccelMps2)));
  const omegaScale = limitScale(Math.abs(sample.omegaRadPerSec), Math.min(degreesToRadians(value('shotOmegaCap')), driveLimits.maxAngularRateRadPerSec));
  const alphaScale = Math.sqrt(limitScale(
    Math.abs(sample.angularAccelRadPerSec2),
    Math.min(degreesToRadians(value('shotAlphaCap')), driveLimits.maxAngularAccelRadPerSec2),
  ));
  return Math.max(0.0, Math.min(1.0, speedScale, accelScale, omegaScale, alphaScale));
}

function limitScale(actual: number, limit: number): number {
  if (actual <= 1e-9) return 1.0;
  return Math.max(0.0, Math.min(1.0, limit / actual));
}

function scalePoint(point: Point, scale: number): Point {
  return { x: point.x * scale, y: point.y * scale };
}

function applyOptimizedThresholds(): void {
  if (checked('capsUseOptimizedPid')) {
    try {
      const bestPid = bestTurretPidScoreForCurrentContext();
      cachedTurretPidOptimization = { key: turretPidOptimizationKey(), score: bestPid };
      applyTurretPidScore(bestPid);
      appState.turretOptimizeMessage = [
        `Caps optimizer used best PID score ${bestPid.score.toFixed(1)} before ranking movement caps.`,
        `ready ${(bestPid.readyRate * 100).toFixed(0)}%, p95 ${bestPid.p95AbsErrorDeg.toFixed(2)} deg.`,
        `kP ${bestPid.config.kP.toFixed(2)}, kI ${bestPid.config.kI.toFixed(3)}, kD ${bestPid.config.kD.toFixed(3)}.`,
      ].join(' ');
    } catch (error) {
      appState.turretOptimizeMessage = error instanceof Error ? error.message : 'Turret PID optimization failed.';
    }
  }
  cachedThresholdRows = null;
  const rows = thresholdRecommendationRows(true);
  const best = rows[0];
  if (!best) {
    appState.optimizeMessage = 'No scored samples available.';
    render();
    return;
  }

  setInputValue('shotSpeedCap', best.speedCapMps);
  setInputValue('shotAccelCap', best.accelCapMps2);
  appState.optimizeMessage = best.qualified
    ? `Applied max qualified envelope: ${best.speedCapMps.toFixed(2)} m/s, ${best.accelCapMps2.toFixed(1)} m/s², ${(best.minAccuracy * 100).toFixed(0)}% worst-case estimated accuracy, ${(best.turretReadyRate * 100).toFixed(0)}% turret-ready.`
    : `No full guarantee found; applied best tradeoff: ${best.speedCapMps.toFixed(2)} m/s, ${best.accelCapMps2.toFixed(1)} m/s², ${(best.solveRate * 100).toFixed(0)}% solvable, ${(best.minAccuracy * 100).toFixed(0)}% accuracy, ${(best.turretReadyRate * 100).toFixed(0)}% turret-ready.`;
  cachedPathOverlay = null;
  cachedThresholdRows = null;
  syncPoseToPathTime();
  render();
}

function applyOptimizedTurretPid(): void {
  let best: TurretPidScore;
  try {
    best = bestTurretPidScoreForCurrentContext();
  } catch (error) {
    appState.turretOptimizeMessage = error instanceof Error ? error.message : 'Turret PID optimization failed.';
    render();
    return;
  }
  cachedTurretPidOptimization = { key: turretPidOptimizationKey(), score: best };
  applyTurretPidScore(best);
  appState.turretOptimizeMessage = [
    `Applied PID score ${best.score.toFixed(1)} over ${best.sampleCount} samples.`,
    `ready ${(best.readyRate * 100).toFixed(0)}%, avg error ${best.avgAbsErrorDeg.toFixed(2)} deg, p95 ${best.p95AbsErrorDeg.toFixed(2)} deg, max ${best.maxAbsErrorDeg.toFixed(2)} deg.`,
    `kP ${best.config.kP.toFixed(2)}, kI ${best.config.kI.toFixed(3)}, kD ${best.config.kD.toFixed(3)}, kV ${best.config.kV.toFixed(3)}, kA ${best.config.kA.toFixed(3)}.`,
  ].join(' ');
  render();
}

function bestTurretPidScoreForCurrentContext(): TurretPidScore {
  const cacheKey = turretPidOptimizationKey();
  if (cachedTurretPidOptimization?.key === cacheKey) return cachedTurretPidOptimization.score;
  const samples = turretOptimizerSamples();
  if (samples.length < 4) {
    throw new Error('No turret optimization samples available.');
  }
  const candidates = turretPidCandidates(turretPidSearchBaseConfig());
  const best = candidates
    .map((config) => scoreTurretCandidateForCurrentContext(config, samples))
    .sort((a, b) => b.score - a.score)[0];
  if (!best) {
    throw new Error('No turret PID candidate produced a score.');
  }
  cachedTurretPidOptimization = { key: cacheKey, score: best };
  return best;
}

function turretPidSearchBaseConfig(): TurretControllerConfig {
  const current = turretControllerConfig();
  return {
    ...current,
    kP: defaultTurretControllerConfig.kP,
    kI: defaultTurretControllerConfig.kI,
    kD: defaultTurretControllerConfig.kD,
    kS: defaultTurretControllerConfig.kS,
    kV: defaultTurretControllerConfig.kV,
    kA: defaultTurretControllerConfig.kA,
  };
}

function applyTurretPidScore(best: TurretPidScore): void {
  setInputValue('turretKp', best.config.kP, 2);
  setInputValue('turretKi', best.config.kI, 3);
  setInputValue('turretKd', best.config.kD, 3);
  setInputValue('turretKv', best.config.kV, 3);
  setInputValue('turretKa', best.config.kA, 3);
  turretSim = createTurretSimState(value('turretCurrentAngle'));
}

function turretPidCandidates(baseConfig: TurretControllerConfig): TurretControllerConfig[] {
  const kPFactors = [0.85, 1.0, 1.15];
  const kIFactors = [0.0, 0.5, 1.0];
  const kDFactors = [0.8, 1.0, 1.2];
  const kVFactors = [0.95, 1.0, 1.05];
  const kAFactors = [0.9, 1.0, 1.1];
  const candidates: TurretControllerConfig[] = [baseConfig];
  for (const kPFactor of kPFactors) {
    for (const kIFactor of kIFactors) {
      for (const kDFactor of kDFactors) {
        for (const kVFactor of kVFactors) {
          for (const kAFactor of kAFactors) {
            candidates.push({
              ...baseConfig,
              kP: clamp(baseConfig.kP * kPFactor, 0.0, 20.0),
              kI: clamp(baseConfig.kI * kIFactor, 0.0, 5.0),
              kD: clamp(baseConfig.kD * kDFactor, 0.0, 5.0),
              kV: clamp(baseConfig.kV * kVFactor, 0.0, 1.0),
              kA: clamp(baseConfig.kA * kAFactor, 0.0, 0.1),
            });
          }
        }
      }
    }
  }
  return candidates;
}

function scoreTurretForCaps(
  config: TurretControllerConfig,
  speedCapMps: number,
  accelCapMps2: number,
): TurretPidScore {
  const samples = turretOptimizerSamplesForCaps(speedCapMps, accelCapMps2, 28);
  return appState.pageMode === 'path' && appState.loadedPath
    ? scoreTurretPidConfig(config, samples)
    : scoreTurretIndependentSamples(config, samples);
}

function scoreTurretCandidateForCurrentContext(
  config: TurretControllerConfig,
  samples: TurretOptimizerSample[],
): TurretPidScore {
  return appState.pageMode === 'path' && appState.loadedPath
    ? scoreTurretPidConfig(config, samples)
    : scoreTurretIndependentSamples(config, samples);
}

function scoreTurretPidConfig(
  config: TurretControllerConfig,
  samples: TurretOptimizerSample[],
): TurretPidScore {
  const firstGoal = firstValidTurretGoal(samples);
  if (firstGoal === null) {
    return emptyTurretPidScore(config);
  }
  let state = createTurretSimState(firstGoal);
  const errors: number[] = [];
  let readyCount = 0;
  let saturatedCount = 0;
  let currentLimitedCount = 0;
  let directionChanges = 0;
  let previousVelocitySign = 0;

  for (const sample of samples) {
    const evaluation = evaluateShotFromBaseMotion(sample.base, state.angleDeg);
    if (!isTurretScoreableShot(evaluation)) continue;
    const goalDeg = evaluation.turretSolution.legalGoalTurretYawDeg;
    state = stepTurretSimulator(state, goalDeg, sample.dtSec, config);
    const absError = Math.abs(state.aimErrorDeg);
    errors.push(absError);
    if (absError <= config.aimToleranceDeg) readyCount += 1;
    if (Math.abs(state.voltage) >= config.voltageLimit * 0.98) saturatedCount += 1;
    if (state.estimatedCurrentA >= config.supplyCurrentLimitA * 1.15) currentLimitedCount += 1;
    const velocitySign = Math.sign(state.velocityDegPerSec);
    if (previousVelocitySign !== 0 && velocitySign !== 0 && velocitySign !== previousVelocitySign) {
      directionChanges += 1;
    }
    if (velocitySign !== 0) previousVelocitySign = velocitySign;
  }

  return turretPidScoreFromErrors(config, errors, readyCount, saturatedCount, currentLimitedCount, directionChanges);
}

function firstValidTurretGoal(samples: TurretOptimizerSample[]): number | null {
  for (const sample of samples) {
    const evaluation = evaluateShotFromBaseMotion(sample.base, value('turretCurrentAngle'));
    if (isTurretScoreableShot(evaluation)) return evaluation.turretSolution.legalGoalTurretYawDeg;
  }
  return null;
}

function isTurretScoreableShot(evaluation: ShotEvaluation): boolean {
  return evaluation.reachable
    && evaluation.candidate !== null
    && evaluation.shotCommandValid
    && evaluation.turretSolution.valid
    && (evaluation.turretSideProfile?.targetOpeningValid ?? false);
}

function emptyTurretPidScore(config: TurretControllerConfig): TurretPidScore {
  return {
    config,
    score: -9999,
    sampleCount: 0,
    readyRate: 0.0,
    avgAbsErrorDeg: 999,
    p95AbsErrorDeg: 999,
    maxAbsErrorDeg: 999,
    saturationRate: 1.0,
    currentLimitRate: 1.0,
  };
}

function turretOptimizerSamples(): TurretOptimizerSample[] {
  return turretOptimizerSamplesForCaps(value('shotSpeedCap'), value('shotAccelCap'), 56);
}

function turretOptimizerSamplesForCaps(
  speedCapMps: number,
  accelCapMps2: number,
  maxSamples: number,
): TurretOptimizerSample[] {
  if (appState.pageMode === 'path' && appState.loadedPath) {
    const pathSamples = downsample(appState.loadedPath.samples, maxSamples);
    return pathSamples.map((sample, index) => {
      const previous = pathSamples[Math.max(0, index - 1)];
      const dtSec = index === 0
        ? 0.02
        : clamp(sample.timeSec - previous.timeSec, 0.01, 0.08);
      return {
        base: baseMotionFromPathSampleForCaps(sample, speedCapMps, accelCapMps2),
        dtSec,
      };
    });
  }

  return turretEnvelopeSamplesForCaps(speedCapMps, accelCapMps2, maxSamples);
}

function turretEnvelopeSamplesForCaps(
  speedCapMps: number,
  accelCapMps2: number,
  maxSamples: number,
): TurretOptimizerSample[] {
  const pose = currentPose();
  const turnFraction = value('turretOptimizeTurnFraction') / 100.0;
  const omegaCap = Math.min(
    driveLimits.maxAngularRateRadPerSec * clamp(turnFraction, 0.1, 0.8),
    degreesToRadians(value('shotOmegaCap')),
  );
  const alphaCap = Math.min(degreesToRadians(value('shotAlphaCap')), driveLimits.maxAngularAccelRadPerSec2);
  const velocityAngles = directionAngles(8);
  const accelAngles = directionAngles(4);
  const speedFractions = [0.0, 0.35, 0.7, 1.0];
  const cappedSpeed = Math.min(speedCapMps, driveLimits.maxFreeSpeedMps);
  const cappedAccel = Math.min(accelCapMps2, driveLimits.maxLinearAccelMps2);
  const dtSec = 0.04;
  const baseSamples: BaseMotionState[] = [{
    pose,
    velocity: { x: 0, y: 0 },
    acceleration: { x: 0, y: 0 },
    omegaRadPerSec: 0,
    angularAccelRadPerSec2: 0,
    source: appState.pageMode,
  }];

  for (const speedFraction of speedFractions) {
    const speed = cappedSpeed * speedFraction;
    for (const velocityAngle of velocityAngles) {
      baseSamples.push({
        pose,
        velocity: polarPoint(speed, velocityAngle),
        acceleration: { x: 0, y: 0 },
        omegaRadPerSec: 0,
        angularAccelRadPerSec2: 0,
        source: appState.pageMode,
      });
    }
  }

  for (const velocityAngle of velocityAngles) {
    for (const accelAngle of accelAngles) {
      baseSamples.push({
        pose,
        velocity: polarPoint(cappedSpeed, velocityAngle),
        acceleration: polarPoint(cappedAccel, accelAngle),
        omegaRadPerSec: 0,
        angularAccelRadPerSec2: 0,
        source: appState.pageMode,
      });
    }
  }

  for (const omegaSign of [-1, 1]) {
    for (const velocityAngle of velocityAngles) {
      baseSamples.push({
        pose,
        velocity: polarPoint(cappedSpeed * 0.7, velocityAngle),
        acceleration: { x: 0, y: 0 },
        omegaRadPerSec: omegaSign * omegaCap,
        angularAccelRadPerSec2: 0,
        source: appState.pageMode,
      });
    }
    for (const alphaSign of [-1, 1]) {
      baseSamples.push({
        pose,
        velocity: { x: 0, y: 0 },
        acceleration: { x: 0, y: 0 },
        omegaRadPerSec: omegaSign * omegaCap,
        angularAccelRadPerSec2: alphaSign * alphaCap,
        source: appState.pageMode,
      });
    }
  }

  return downsample(baseSamples, maxSamples).map((base) => ({
    dtSec,
    base: {
      ...base,
      pose: clampPoseInsideField(base.pose),
      velocity: clampVectorMagnitudeLocal(base.velocity, cappedSpeed),
      acceleration: clampVectorMagnitudeLocal(base.acceleration, cappedAccel),
    },
  }));
}

function scoreTurretIndependentSamples(
  config: TurretControllerConfig,
  samples: TurretOptimizerSample[],
): TurretPidScore {
  const errors: number[] = [];
  let readyCount = 0;
  let saturatedCount = 0;
  let currentLimitedCount = 0;

  for (const sample of samples) {
    const dt = clamp(sample.dtSec, 0.01, 0.05);
    const evaluationNow = evaluateShotFromBaseMotion(sample.base, 0.0);
    if (!isTurretScoreableShot(evaluationNow)) continue;

    const baseNext = advanceBaseMotion(sample.base, dt);
    const evaluationNext = evaluateShotFromBaseMotion(
      baseNext,
      evaluationNow.turretSolution.legalGoalTurretYawDeg,
    );
    if (!isTurretScoreableShot(evaluationNext)) continue;

    const baseAfterNext = advanceBaseMotion(baseNext, dt);
    const evaluationAfterNext = evaluateShotFromBaseMotion(
      baseAfterNext,
      evaluationNext.turretSolution.legalGoalTurretYawDeg,
    );

    const goal0 = evaluationNow.turretSolution.legalGoalTurretYawDeg;
    const goal1 = evaluationNext.turretSolution.legalGoalTurretYawDeg;
    const goal2 = isTurretScoreableShot(evaluationAfterNext)
      ? evaluationAfterNext.turretSolution.legalGoalTurretYawDeg
      : goal1;
    const goalVelocity0 = (goal1 - goal0) / dt;
    const goalVelocity1 = (goal2 - goal1) / dt;
    const goalAcceleration = (goalVelocity1 - goalVelocity0) / dt;
    let state = {
      ...createTurretSimState(goal0),
      velocityDegPerSec: goalVelocity0,
      previousGoalDeg: goal0 - (goalVelocity0 * dt),
      previousGoalVelocityDegPerSec: goalVelocity0,
    };
    state = stepTurretSimulator(state, goal1, dt, config);

    const absError = Math.abs(state.aimErrorDeg);
    const demandWithinLimits = Math.abs(goalVelocity0) <= effectiveTurretVelocityDegPerSecForConfig(config) * 0.92
      && Math.abs(goalAcceleration) <= effectiveTurretAccelerationDegPerSec2ForConfig(config) * 0.92;

    errors.push(absError);
    if (absError <= config.aimToleranceDeg && demandWithinLimits && !state.trackingRiskReason) readyCount += 1;
    if (Math.abs(state.voltage) >= config.voltageLimit * 0.98) saturatedCount += 1;
    if (state.estimatedCurrentA >= config.supplyCurrentLimitA * 1.15) currentLimitedCount += 1;
  }

  return turretPidScoreFromErrors(config, errors, readyCount, saturatedCount, currentLimitedCount, 0);
}

function turretPidScoreFromErrors(
  config: TurretControllerConfig,
  errors: number[],
  readyCount: number,
  saturatedCount: number,
  currentLimitedCount: number,
  directionChanges: number,
): TurretPidScore {
  if (errors.length === 0) return emptyTurretPidScore(config);
  const sortedErrors = [...errors].sort((a, b) => a - b);
  const avgAbsErrorDeg = errors.reduce((sum, error) => sum + error, 0.0) / errors.length;
  const p95AbsErrorDeg = sortedErrors[Math.min(sortedErrors.length - 1, Math.floor(sortedErrors.length * 0.95))] ?? 999;
  const maxAbsErrorDeg = sortedErrors[sortedErrors.length - 1] ?? 999;
  const readyRate = readyCount / errors.length;
  const saturationRate = saturatedCount / errors.length;
  const currentLimitRate = currentLimitedCount / errors.length;
  const smoothnessPenalty = directionChanges / errors.length;
  const score = (readyRate * 120.0)
    + 15.0
    - (avgAbsErrorDeg * 10.0)
    - (p95AbsErrorDeg * 5.0)
    - (maxAbsErrorDeg * 1.5)
    - (saturationRate * 20.0)
    - (currentLimitRate * 35.0)
    - (smoothnessPenalty * 12.0);

  return {
    config,
    score,
    sampleCount: errors.length,
    readyRate,
    avgAbsErrorDeg,
    p95AbsErrorDeg,
    maxAbsErrorDeg,
    saturationRate,
    currentLimitRate,
  };
}

function advanceBaseMotion(base: BaseMotionState, dtSec: number): BaseMotionState {
  const dt = Math.max(0.0, dtSec);
  return {
    pose: clampPoseInsideField({
      x: base.pose.x + (base.velocity.x * dt) + (0.5 * base.acceleration.x * dt * dt),
      y: base.pose.y + (base.velocity.y * dt) + (0.5 * base.acceleration.y * dt * dt),
      headingRad: wrapAngle(
        base.pose.headingRad
          + (base.omegaRadPerSec * dt)
          + (0.5 * base.angularAccelRadPerSec2 * dt * dt),
      ),
    }),
    velocity: {
      x: base.velocity.x + (base.acceleration.x * dt),
      y: base.velocity.y + (base.acceleration.y * dt),
    },
    acceleration: base.acceleration,
    omegaRadPerSec: base.omegaRadPerSec + (base.angularAccelRadPerSec2 * dt),
    angularAccelRadPerSec2: base.angularAccelRadPerSec2,
    source: base.source,
  };
}

function effectiveTurretVelocityDegPerSecForConfig(config: TurretControllerConfig): number {
  return config.maxVelocityDegPerSec * clamp(config.effectiveMotionScale, 0.05, 1.0);
}

function effectiveTurretAccelerationDegPerSec2ForConfig(config: TurretControllerConfig): number {
  const supplyCurrent = Math.max(0.1, config.supplyCurrentLimitA);
  const statorCurrent = Math.max(supplyCurrent, config.statorCurrentLimitA);
  const currentScale = clamp(supplyCurrent / statorCurrent, 0.12, 1.0);
  return config.maxAccelerationDegPerSec2 * clamp(config.effectiveMotionScale, 0.05, 1.0) * currentScale;
}

function thresholdRowsCacheKey(): string {
  if (appState.pageMode === 'path' && appState.loadedPath) return `${pathAnalysisKey()}|threshold`;
  const pose = currentPose();
  return [
    appState.pageMode,
    appState.alliance,
    appState.targetMode,
    pose.x.toFixed(2),
    pose.y.toFixed(2),
    pose.headingRad.toFixed(2),
    value('radial').toFixed(2),
    value('tangential').toFixed(2),
    driveSim.velocity.x.toFixed(2),
    driveSim.velocity.y.toFixed(2),
    driveSim.acceleration.x.toFixed(2),
    driveSim.acceleration.y.toFixed(2),
    driveSim.omegaRadPerSec.toFixed(2),
    driveSim.angularAccelRadPerSec2.toFixed(2),
    value('releaseLatency').toFixed(2),
    value('desiredAccuracy').toFixed(0),
    value('shotSpeedCap').toFixed(2),
    value('shotAccelCap').toFixed(2),
    value('shotOmegaCap').toFixed(0),
    value('shotAlphaCap').toFixed(0),
    checked('capsUseOptimizedPid') ? 'best-pid' : 'current-pid',
    cachedTurretPidOptimization?.key ?? 'no-pid-cache',
    value('turretKp').toFixed(2),
    value('turretKi').toFixed(3),
    value('turretKd').toFixed(3),
    value('turretKv').toFixed(3),
    value('turretKa').toFixed(3),
    value('turretEffectiveMotionScale').toFixed(0),
    value('turretSupplyCurrentLimit').toFixed(0),
    value('turretStatorCurrentLimit').toFixed(0),
    checked('applyVisualLimiter') ? 'limited' : 'raw',
  ].join('|');
}

function pathAnalysisKey(): string {
  const path = appState.loadedPath;
  return [
    path?.name ?? 'none',
    path?.source ?? 'none',
    path?.samples.length ?? 0,
    path?.totalTimeSec.toFixed(2) ?? '0',
    appState.alliance,
    appState.targetMode,
    value('releaseLatency').toFixed(2),
    value('desiredAccuracy').toFixed(0),
    value('shotSpeedCap').toFixed(2),
    value('shotAccelCap').toFixed(2),
    value('shotOmegaCap').toFixed(0),
    value('shotAlphaCap').toFixed(0),
    checked('capsUseOptimizedPid') ? 'best-pid' : 'current-pid',
    cachedTurretPidOptimization?.key ?? 'no-pid-cache',
    value('turretKp').toFixed(2),
    value('turretKi').toFixed(3),
    value('turretKd').toFixed(3),
    value('turretEffectiveMotionScale').toFixed(0),
  ].join('|');
}

async function loadPathFile(file: File): Promise<void> {
  try {
    const loaded = parsePathFile(file.name, await file.text());
    appState.loadedPath = loaded;
    appState.pathTimeSec = 0;
    appState.pathPlaying = false;
    appState.pathMessage = loaded.message;
    const pathTime = document.querySelector<HTMLInputElement>('#pathTime');
    if (pathTime) {
      pathTime.min = '0';
      pathTime.max = loaded.totalTimeSec.toFixed(2);
      pathTime.value = '0';
      pathTime.step = '0.01';
    }
    syncPoseToPathTime();
  } catch (error) {
    appState.loadedPath = null;
    appState.pathPlaying = false;
    appState.pathMessage = error instanceof Error ? error.message : 'Failed to load path file.';
  }
  render();
}

function syncPoseToPathTime(): void {
  if (!appState.loadedPath) return;
  const sample = sampleLoadedPath(appState.loadedPath, appState.pathTimeSec);
  const base = baseMotionFromPathSample(sample, checked('applyVisualLimiter'));
  robotPose = sample.pose;
  driveSim = {
    pose: base.pose,
    velocity: base.velocity,
    acceleration: base.acceleration,
    omegaRadPerSec: base.omegaRadPerSec,
    angularAccelRadPerSec2: base.angularAccelRadPerSec2,
  };
}

function baseMotionFromPathSample(sample: PathSample, applyLimiter: boolean): BaseMotionState {
  const scale = applyLimiter ? motionLimitScaleForSample(sample) : 1.0;
  const accelerationScale = scale * scale;
  return {
    pose: sample.pose,
    velocity: scalePoint(sample.velocity, scale),
    acceleration: scalePoint(sample.acceleration, accelerationScale),
    omegaRadPerSec: sample.omegaRadPerSec * scale,
    angularAccelRadPerSec2: sample.angularAccelRadPerSec2 * accelerationScale,
    source: 'path',
  };
}

function updateRobotFromPointer(event: PointerEvent): void {
  const rect = fieldCanvas.getBoundingClientRect();
  const bounds = fieldBounds();
  const canvasX = ((event.clientX - rect.left) / rect.width) * fieldCanvas.width;
  const canvasY = ((event.clientY - rect.top) / rect.height) * fieldCanvas.height;
  const desiredPose = clampPoseInsideField({
    ...robotPose,
    x: (canvasX - bounds.x) / bounds.scale,
    y: (bounds.y + bounds.height - canvasY) / bounds.scale,
  });
  if (isReachablePose(desiredPose)) {
    robotPose = desiredPose;
    driveSim = createDriveSimState(robotPose);
  }
  render();
}

function fieldBounds(): FieldBounds {
  const marginX = 24;
  const marginY = 34;
  const scale = Math.min(
    (fieldCanvas.width - marginX * 2) / field.lengthMeters,
    (fieldCanvas.height - marginY * 2) / field.widthMeters,
  );
  const width = field.lengthMeters * scale;
  const height = field.widthMeters * scale;
  return {
    x: (fieldCanvas.width - width) / 2,
    y: (fieldCanvas.height - height) / 2,
    width,
    height,
    scale,
  };
}

function fieldToCanvas(point: Point): Point {
  const bounds = fieldBounds();
  return {
    x: bounds.x + point.x * bounds.scale,
    y: bounds.y + bounds.height - point.y * bounds.scale,
  };
}

function slider(id: string, label: string, min: number, max: number, value: number, step: number): string {
  return `<label><span>${label}</span><input id="${id}" type="range" min="${min}" max="${max}" value="${value}" step="${step}" /><output></output></label>`;
}

function value(id: string): number {
  const input = document.querySelector<HTMLInputElement>(`#${id}`);
  return Number(input?.value ?? 0);
}

function checked(id: string): boolean {
  const input = document.querySelector<HTMLInputElement>(`#${id}`);
  return Boolean(input?.checked);
}

function setInputValue(id: string, nextValue: number, digits = 2): void {
  const input = document.querySelector<HTMLInputElement>(`#${id}`);
  if (input) input.value = nextValue.toFixed(digits);
}

function syncOutputs(): void {
  for (const label of appRoot.querySelectorAll('label')) {
    const input = label.querySelector('input');
    const output = label.querySelector('output');
    if (input && output && input.type !== 'file') output.textContent = Number(input.value).toFixed(2);
  }
  const status = document.querySelector<HTMLSpanElement>('#pathStatus');
  if (status) status.textContent = appState.loadedPath ? appState.loadedPath.message : appState.pathMessage;
  const play = document.querySelector<HTMLButtonElement>('#playPath');
  if (play) play.textContent = appState.pathPlaying ? 'Pause' : 'Play';
  const optimizeStatus = document.querySelector<HTMLSpanElement>('#optimizeStatus');
  if (optimizeStatus) optimizeStatus.textContent = appState.optimizeMessage;
  const turretOptimizeStatus = document.querySelector<HTMLSpanElement>('#turretOptimizeStatus');
  if (turretOptimizeStatus) turretOptimizeStatus.textContent = appState.turretOptimizeMessage;
}

function syncUiState(): void {
  appRoot.dataset.page = appState.pageMode;
  document.querySelector('#dragPage')?.classList.toggle('active', appState.pageMode === 'drag');
  document.querySelector('#simPage')?.classList.toggle('active', appState.pageMode === 'sim');
  document.querySelector('#pathPage')?.classList.toggle('active', appState.pageMode === 'path');
  document.querySelector('#blueAlliance')?.classList.toggle('active', appState.alliance === 'blue');
  document.querySelector('#redAlliance')?.classList.toggle('active', appState.alliance === 'red');
  document.querySelector('#autoTarget')?.classList.toggle('active', appState.targetMode === 'auto');
  document.querySelector('#hubTarget')?.classList.toggle('active', appState.targetMode === 'hub');
  document.querySelector('#neutralBackTarget')?.classList.toggle('active', appState.targetMode === 'neutral-back');
  document.querySelector('#opponentPassTarget')?.classList.toggle('active', appState.targetMode === 'opponent-passback');
}

function setPageMode(pageMode: PageMode, updateHash = true): void {
  appState.pageMode = pageMode;
  if (pageMode === 'sim') {
    driveSim = createDriveSimState(currentPose());
  } else if (pageMode === 'path') {
    syncPoseToPathTime();
  } else {
    robotPose = currentPose();
    const headingInput = document.querySelector<HTMLInputElement>('#heading');
    if (headingInput) headingInput.value = radiansToDegrees(robotPose.headingRad).toFixed(0);
  }
  if (updateHash) window.location.hash = `#${pageMode}`;
  render();
}

function setAlliance(alliance: Alliance): void {
  appState.alliance = alliance;
  render();
}

function setTargetMode(targetMode: TargetMode): void {
  appState.targetMode = targetMode;
  render();
}

function currentPose(): Pose2d {
  if (appState.pageMode === 'sim') return driveSim.pose;
  if (appState.pageMode === 'path' && appState.loadedPath) return sampleLoadedPath(appState.loadedPath, appState.pathTimeSec).pose;
  return robotPose;
}

function pageModeFromHash(): PageMode {
  const hash = window.location.hash.toLowerCase();
  if (hash.includes('path')) return 'path';
  if (hash.includes('sim')) return 'sim';
  return 'drag';
}

function pageModeLabel(pageMode: PageMode): string {
  switch (pageMode) {
    case 'drag':
      return 'Drag inspector';
    case 'sim':
      return 'Keyboard simulator';
    case 'path':
      return 'Path playback';
  }
}

function isDriveKey(key: string): boolean {
  return ['w', 'a', 's', 'd', 'i', 'j', 'k', 'l', 'q', 'e', 'r'].includes(key);
}

function shortElementName(name: string): string {
  return name
    .replace('Blue ', 'B ')
    .replace('Red ', 'R ')
    .replace(' structure', '')
    .replace(' low-clearance arm', '')
    .replace(' base', '');
}

function distance(a: Point, b: Point): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function vectorMagnitude(point: Point): number {
  return Math.hypot(point.x, point.y);
}

function clampVectorMagnitudeLocal(point: Point, maxMagnitude: number): Point {
  const magnitude = vectorMagnitude(point);
  if (magnitude <= maxMagnitude || magnitude <= 1e-9) return point;
  const scale = maxMagnitude / magnitude;
  return { x: point.x * scale, y: point.y * scale };
}

function dot2d(a: Point, b: Point): number {
  return (a.x * b.x) + (a.y * b.y);
}

function line(context: CanvasRenderingContext2D, x1: number, y1: number, x2: number, y2: number): void {
  context.beginPath();
  context.moveTo(x1, y1);
  context.lineTo(x2, y2);
  context.stroke();
}

function fieldVelocityFromRadialTangential(
  shooterPoint: Point,
  targetPoint: Point,
  radialVelocityMps: number,
  tangentialVelocityMps: number,
): Point {
  const dx = targetPoint.x - shooterPoint.x;
  const dy = targetPoint.y - shooterPoint.y;
  const targetDistance = Math.hypot(dx, dy);
  const ux = targetDistance > 1e-9 ? dx / targetDistance : 1;
  const uy = targetDistance > 1e-9 ? dy / targetDistance : 0;
  return {
    x: (radialVelocityMps * ux) + (tangentialVelocityMps * -uy),
    y: (radialVelocityMps * uy) + (tangentialVelocityMps * ux),
  };
}

function downsample<T>(items: T[], maxItems: number): T[] {
  if (items.length <= maxItems) return items;
  const result: T[] = [];
  for (let i = 0; i < maxItems; i += 1) {
    result.push(items[Math.round((i * (items.length - 1)) / (maxItems - 1))]);
  }
  return result;
}

function directionAngles(count: number): number[] {
  return Array.from({ length: count }, (_, index) => (2.0 * Math.PI * index) / count);
}

function polarPoint(magnitude: number, angleRad: number): Point {
  return {
    x: magnitude * Math.cos(angleRad),
    y: magnitude * Math.sin(angleRad),
  };
}

function domainHeadroom(valueToScore: number, min: number, max: number): number {
  const center = (min + max) / 2.0;
  const halfSpan = Math.max((max - min) / 2.0, 1e-9);
  return clamp(1.0 - (Math.abs(valueToScore - center) / halfSpan), 0.0, 1.0);
}

function clamp(valueToClamp: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, valueToClamp));
}

function roundToStep(valueToRound: number, step: number): number {
  return Math.round(valueToRound / step) * step;
}

function degreesToRadians(degrees: number): number {
  return (degrees * Math.PI) / 180.0;
}

function radiansToDegrees(radians: number): number {
  return (radians * 180.0) / Math.PI;
}

function wrapAngle(angleRad: number): number {
  return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
}

function escapeHtml(valueToEscape: string): string {
  return valueToEscape
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#039;');
}

interface BaseMotionState {
  pose: Pose2d;
  velocity: Point;
  acceleration: Point;
  omegaRadPerSec: number;
  angularAccelRadPerSec2: number;
  source: PageMode;
}

interface ReleaseState {
  latencySec: number;
  predictedPose: Pose2d;
  currentShooterPoint: Point;
  shooterPoint: Point;
  shooterVelocity: Point;
  shooterAcceleration: Point;
  omegaRadPerSec: number;
  angularAccelRadPerSec2: number;
}

interface ThresholdEvaluation {
  ready: boolean;
  reason: string;
  speedMps: number;
  accelerationMps2: number;
  omegaDegPerSec: number;
  alphaDegPerSec2: number;
}

interface ShotEvaluation {
  base: BaseMotionState;
  release: ReleaseState;
  target: TargetSelection;
  candidate: TrajectoryCandidate | null;
  hoodPrediction: number | null;
  rollerPrediction: number | null;
  polynomialValid: boolean;
  shotCommandValid: boolean;
  turretSolution: TurretSolution;
  turretTracking: TurretTrackingEvaluation;
  turretSideProfile: TurretSideProfile | null;
  threshold: ThresholdEvaluation;
  reachable: boolean;
  distanceMeters: number;
  radialVelocityMps: number;
  tangentialVelocityMps: number;
  shotReady: boolean;
  invalidReason: string;
}

interface LaunchCommand {
  launchAngleDeg: number;
  exitSpeedMps: number;
  source: 'polynomial' | 'candidate';
}

interface TurretTrackingEvaluation {
  ready: boolean;
  reason: string;
  simEnabled: boolean;
  currentAngleDeg: number;
  aimErrorDeg: number;
  toleranceDeg: number;
}

interface TurretSideProfile {
  radialLaunchSpeedMps: number;
  tangentialLaunchSpeedMps: number;
  radialFieldSpeedMps: number;
  tangentialFieldSpeedMps: number;
  verticalLaunchSpeedMps: number;
  targetDistanceMeters: number;
  flightTimeToTargetSec: number;
  impactHeightMeters: number;
  peakHeightMeters: number;
  crossTrackAtTargetMeters: number;
  targetOpeningValid: boolean;
  frontLipCleared: boolean;
  openingIntersectionValid: boolean;
  entryAngleDeg: number;
  entryAngleValid: boolean;
  openingSamples: TurretSideSample[];
}

interface TurretSideSample {
  distanceMeters: number;
  heightMeters: number;
  crossTrackMeters: number;
}

interface TurretOptimizerSample {
  base: BaseMotionState;
  dtSec: number;
}

interface TurretPidScore {
  config: TurretControllerConfig;
  score: number;
  sampleCount: number;
  readyRate: number;
  avgAbsErrorDeg: number;
  p95AbsErrorDeg: number;
  maxAbsErrorDeg: number;
  saturationRate: number;
  currentLimitRate: number;
}

interface ThresholdSample {
  considered: boolean;
  speedMps: number;
  accelerationMps2: number;
  omegaDegPerSec: number;
  alphaDegPerSec2: number;
  shotSolvable: boolean;
  accuracyEstimate: number;
}

interface ShotQualityEstimate {
  solvable: boolean;
  accuracyEstimate: number;
}

interface ThresholdRecommendation {
  speedCapMps: number;
  accelCapMps2: number;
  solveRate: number;
  minAccuracy: number;
  accuracyPassRate: number;
  turretReadyRate: number;
  turretP95ErrorDeg: number;
  qualified: boolean;
  score: number;
}
