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
import { createDriveSimState, stepDriveSimulator, type DriveSimState, type KeyboardState } from './math/simulator';
import { findBestCandidate } from './math/trajectory';
import { evaluatePolynomial } from './math/polynomial';
import { generatedCoefficients } from './math/generatedCoefficients';
import type { ShotCalculatorConfig, TrajectoryCandidate } from './math/types';
import './ui/styles.css';

const app = document.querySelector<HTMLElement>('#app');
if (!app) throw new Error('Missing #app root');
const appRoot = app;

const appState = {
  pageMode: pageModeFromHash(),
  alliance: 'blue' as Alliance,
  targetMode: 'auto' as TargetMode,
};

let robotPose: Pose2d = {
  x: 2.0,
  y: field.widthMeters / 2.0,
  headingRad: 0,
};
let driveSim: DriveSimState = createDriveSimState(robotPose);
const keyboard: KeyboardState = { keys: new Set() };
let draggingRobot = false;
let lastFrameTime = performance.now();

appRoot.innerHTML = `
  <section class="panel">
    <div>
      <p class="eyebrow">Ultimate SOTM</p>
      <h1>Shot Calculator</h1>
      <p class="summary">Inspect HUB scoring, passback shots, hard field geometry, and the real shooter-pivot offset on the 2026 REBUILT field.</p>
    </div>
    <div class="controls">
      <div class="control-group">
        <span class="control-label">Visualizer Page</span>
        <div class="segmented page-mode" aria-label="Visualizer page selector">
          <button id="dragPage" class="active" type="button">Drag Inspector</button>
          <button id="simPage" type="button">Keyboard Sim</button>
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
      <div class="drag-only">
        ${slider('heading', 'Robot heading (deg)', -180, 180, 0, 1)}
        ${slider('radial', 'Radial velocity (m/s)', -2.5, 2.5, 0, 0.05)}
        ${slider('tangential', 'Tangential velocity (m/s)', -2.5, 2.5, 0, 0.05)}
      </div>
      <div class="sim-card sim-only">
        <b>Keyboard simulator</b>
        <span>WASD = field-centric translation</span>
        <span>IJKL = robot-centric translation</span>
        <span>Q/E = rotate, R = reset velocity</span>
        <span>Limits: ${driveLimits.maxFreeSpeedMps.toFixed(2)} m/s, ${driveLimits.maxLinearAccelMps2.toFixed(1)} m/s², ${(driveLimits.maxAngularRateRadPerSec * 180 / Math.PI).toFixed(0)} deg/s</span>
      </div>
      ${slider('pitch', 'Robot pitch (deg)', -12, 12, 0, 0.25)}
      ${slider('roll', 'Robot roll (deg)', -12, 12, 0, 0.25)}
    </div>
  </section>
  <section class="field-panel">
    <canvas id="field" width="1180" height="600"></canvas>
  </section>
  <section class="readout">
    <canvas id="plot" width="920" height="460"></canvas>
    <div id="metrics"></div>
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
    }
    render();
  });
}

document.querySelector<HTMLButtonElement>('#dragPage')?.addEventListener('click', () => setPageMode('drag'));
document.querySelector<HTMLButtonElement>('#simPage')?.addEventListener('click', () => setPageMode('sim'));
document.querySelector<HTMLButtonElement>('#blueAlliance')?.addEventListener('click', () => setAlliance('blue'));
document.querySelector<HTMLButtonElement>('#redAlliance')?.addEventListener('click', () => setAlliance('red'));
document.querySelector<HTMLButtonElement>('#autoTarget')?.addEventListener('click', () => setTargetMode('auto'));
document.querySelector<HTMLButtonElement>('#hubTarget')?.addEventListener('click', () => setTargetMode('hub'));
document.querySelector<HTMLButtonElement>('#neutralBackTarget')?.addEventListener('click', () => setTargetMode('neutral-back'));
document.querySelector<HTMLButtonElement>('#opponentPassTarget')?.addEventListener('click', () => setTargetMode('opponent-passback'));

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
requestAnimationFrame(animationFrame);

function animationFrame(timestamp: number): void {
  const dtSec = (timestamp - lastFrameTime) / 1000.0;
  lastFrameTime = timestamp;

  if (appState.pageMode === 'sim') {
    driveSim = stepDriveSimulator(driveSim, keyboard, dtSec);
    robotPose = driveSim.pose;
  }

  render();
  requestAnimationFrame(animationFrame);
}

function render(): void {
  syncUiState();
  syncOutputs();

  const pose = currentPose();
  const shooterPoint = shooterFieldPosition(pose);
  const target = selectedTarget(appState.alliance, appState.targetMode, pose);
  const distanceMeters = distance(shooterPoint, target.point);
  const motion = activeShotMotion(pose, shooterPoint, target.point);
  const candidate = findBestCandidate(
    target.config,
    { distanceMeters, radialVelocityMps: motion.radialVelocityMps },
    target.candidateSelection,
  );
  const hoodPrediction = target.usesPolynomial
    ? evaluatePolynomial(generatedCoefficients.hood, {
      distanceMeters,
      radialVelocityMps: motion.radialVelocityMps,
    })
    : null;
  const rollerPrediction = target.usesPolynomial
    ? evaluatePolynomial(generatedCoefficients.roller, {
      distanceMeters,
      radialVelocityMps: motion.radialVelocityMps,
    })
    : null;

  metricsPanel.innerHTML = candidate
    ? `
      <h2>Selected ${target.shortLabel} Shot</h2>
      <p><b>Page:</b> ${appState.pageMode === 'sim' ? 'Keyboard simulator' : 'Drag inspector'}</p>
      <p><b>Requested mode:</b> ${targetModeLabel(appState.targetMode)}${appState.targetMode === 'auto' ? ` -> ${target.shortLabel}` : ''}</p>
      <p><b>Model:</b> ${target.usesPolynomial ? 'HUB robust map + generated polynomial' : 'Passback direct sweep with flat-fast selector'}</p>
      <p><b>Robot center:</b> x ${pose.x.toFixed(2)} m, y ${pose.y.toFixed(2)} m, heading ${radiansToDegrees(pose.headingRad).toFixed(1)} deg</p>
      <p><b>Shooter pivot:</b> x ${shooterPoint.x.toFixed(2)} m, y ${shooterPoint.y.toFixed(2)} m; offset ${robotGeometry.shooterOffset.x.toFixed(3)}, ${robotGeometry.shooterOffset.y.toFixed(3)} m</p>
      <p><b>Target:</b> ${target.label} at x ${target.point.x.toFixed(2)} m, y ${target.point.y.toFixed(2)} m, z ${target.config.targetHeightMeters.toFixed(2)} m</p>
      <p><b>Distance:</b> ${distanceMeters.toFixed(2)} m from shooter pivot</p>
      <p><b>Radial/tangential:</b> ${motion.radialVelocityMps.toFixed(2)} / ${motion.tangentialVelocityMps.toFixed(2)} m/s</p>
      <p><b>Drive velocity:</b> vx ${motion.fieldVelocity.x.toFixed(2)} m/s, vy ${motion.fieldVelocity.y.toFixed(2)} m/s, omega ${(driveSim.omegaRadPerSec * 180 / Math.PI).toFixed(0)} deg/s</p>
      <p><b>Hood:</b> ${candidate.hoodRot.toFixed(3)} rot (${candidate.launchAngleDeg.toFixed(1)} deg)</p>
      <p><b>Polynomial hood:</b> ${hoodPrediction === null ? 'N/A for passback' : `${hoodPrediction.toFixed(3)} rot`}</p>
      <p><b>Roller:</b> ${candidate.rollerRps.toFixed(1)} rps</p>
      <p><b>Polynomial roller:</b> ${rollerPrediction === null ? 'N/A for passback' : `${rollerPrediction.toFixed(1)} rps`}</p>
      <p><b>Flight time:</b> ${candidate.flightTimeSec.toFixed(2)} s</p>
      <p><b>Target margin:</b> ${candidate.heightMarginMeters.toFixed(3)} m</p>
      <p><b>Reachable pose:</b> ${isReachablePose(pose) ? 'yes' : 'blocked by field geometry'}</p>
      <p><b>Tilt input:</b> pitch ${value('pitch').toFixed(1)} deg, roll ${value('roll').toFixed(1)} deg</p>
    `
    : `
      <h2>No Valid ${target.shortLabel} Shot</h2>
      <p><b>Page:</b> ${appState.pageMode === 'sim' ? 'Keyboard simulator' : 'Drag inspector'}</p>
      <p><b>Requested mode:</b> ${targetModeLabel(appState.targetMode)}${appState.targetMode === 'auto' ? ` -> ${target.shortLabel}` : ''}</p>
      <p><b>Robot center:</b> x ${pose.x.toFixed(2)} m, y ${pose.y.toFixed(2)} m, heading ${radiansToDegrees(pose.headingRad).toFixed(1)} deg</p>
      <p><b>Shooter pivot:</b> x ${shooterPoint.x.toFixed(2)} m, y ${shooterPoint.y.toFixed(2)} m</p>
      <p><b>Target:</b> ${target.label} at x ${target.point.x.toFixed(2)} m, y ${target.point.y.toFixed(2)} m, z ${target.config.targetHeightMeters.toFixed(2)} m</p>
      <p>Distance ${distanceMeters.toFixed(2)} m or radial velocity ${motion.radialVelocityMps.toFixed(2)} m/s is outside this target envelope or has no valid trajectory.</p>
      <p><b>Reachable pose:</b> ${isReachablePose(pose) ? 'yes' : 'blocked by field geometry'}</p>
    `;

  drawField(candidate, target, pose, shooterPoint, motion);
  drawPlot(candidate, distanceMeters, motion.radialVelocityMps, target.config);
}

function activeShotMotion(pose: Pose2d, shooterPoint: Point, targetPoint: Point): ShotMotion {
  if (appState.pageMode === 'sim') {
    const shooterRadius = { x: shooterPoint.x - pose.x, y: shooterPoint.y - pose.y };
    const shooterVelocity = {
      x: driveSim.velocity.x - (driveSim.omegaRadPerSec * shooterRadius.y),
      y: driveSim.velocity.y + (driveSim.omegaRadPerSec * shooterRadius.x),
    };
    return {
      ...radialTangentialVelocity(shooterPoint, targetPoint, shooterVelocity),
      fieldVelocity: shooterVelocity,
    };
  }

  const dx = targetPoint.x - shooterPoint.x;
  const dy = targetPoint.y - shooterPoint.y;
  const targetDistance = Math.hypot(dx, dy);
  const ux = targetDistance > 1e-9 ? dx / targetDistance : 1;
  const uy = targetDistance > 1e-9 ? dy / targetDistance : 0;
  const radialVelocityMps = value('radial');
  const tangentialVelocityMps = value('tangential');
  return {
    radialVelocityMps,
    tangentialVelocityMps,
    fieldVelocity: {
      x: (radialVelocityMps * ux) + (tangentialVelocityMps * -uy),
      y: (radialVelocityMps * uy) + (tangentialVelocityMps * ux),
    },
  };
}

function drawField(
  candidate: TrajectoryCandidate | null,
  target: TargetSelection,
  pose: Pose2d,
  shooterPoint: Point,
  motion: ShotMotion,
): void {
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

  drawAimPoint(context, field.blueHubAim, '#bfdbfe', 'Blue aim');
  drawAimPoint(context, field.redHubAim, '#fecdd3', 'Red aim');
  drawPassTarget(context, field.blueBackUpper, '#38bdf8', 'B back upper', target.point === field.blueBackUpper, 'triangle');
  drawPassTarget(context, field.blueBackLower, '#38bdf8', 'B back lower', target.point === field.blueBackLower, 'triangle');
  drawPassTarget(context, field.redBackUpper, '#fb7185', 'R back upper', target.point === field.redBackUpper, 'triangle');
  drawPassTarget(context, field.redBackLower, '#fb7185', 'R back lower', target.point === field.redBackLower, 'triangle');
  drawPassTarget(context, field.opponentPassBackUpper, '#34d399', 'opp pass upper', target.point === field.opponentPassBackUpper, 'diamond');
  drawPassTarget(context, field.opponentPassBackLower, '#34d399', 'opp pass lower', target.point === field.opponentPassBackLower, 'diamond');

  const shooterCanvas = fieldToCanvas(shooterPoint);
  const targetCanvas = fieldToCanvas(target.point);
  context.strokeStyle = candidate ? target.color : '#ef4444';
  context.lineWidth = 4;
  line(context, shooterCanvas.x, shooterCanvas.y, targetCanvas.x, targetCanvas.y);
  drawRobot(context, pose, shooterPoint);

  context.fillStyle = '#f7f2e8';
  context.font = '600 22px Space Grotesk, sans-serif';
  context.fillText('2026 REBUILT Field - physical geometry + shooter offset', bounds.x, 34);
  context.font = '500 15px IBM Plex Mono, monospace';
  context.fillText(
    `${appState.pageMode === 'sim' ? 'keyboard sim' : 'drag inspector'} | ${target.shortLabel} | shooter distance ${distance(shooterPoint, target.point).toFixed(2)}m | radial ${motion.radialVelocityMps.toFixed(2)}m/s | ${candidate ? 'valid shot' : 'no shot'}`,
    bounds.x,
    fieldCanvas.height - 24,
  );
  drawLegend(context, bounds);
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
    context.fillStyle = 'rgba(245, 158, 11, 0.36)';
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

function drawRobot(context: CanvasRenderingContext2D, pose: Pose2d, shooterPoint: Point): void {
  const footprint = robotFootprint(pose).map(fieldToCanvas);
  context.beginPath();
  footprint.forEach((point, index) => {
    if (index === 0) context.moveTo(point.x, point.y);
    else context.lineTo(point.x, point.y);
  });
  context.closePath();
  context.fillStyle = isReachablePose(pose) ? '#f8c537' : '#ef4444';
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
  const x = bounds.x + bounds.width - 316;
  const y = 24;
  context.fillStyle = 'rgba(16, 24, 32, 0.78)';
  context.fillRect(x - 12, y - 16, 328, 88);
  context.fillStyle = '#dbe7d4';
  context.font = '500 12px IBM Plex Mono, monospace';
  context.fillText('red outline = hard collision/no-reach', x, y);
  context.fillText('amber hatch = TRENCH low clearance', x, y + 20);
  context.fillText('blue/red translucent = BUMP ramp', x, y + 40);
  context.fillText('crosshair = constants HUB aim point', x, y + 60);
}

function drawPlot(
  candidate: TrajectoryCandidate | null,
  distanceMeters: number,
  radialVelocityMps: number,
  config: ShotCalculatorConfig,
): void {
  const context = plot.getContext('2d');
  if (!context) return;
  context.clearRect(0, 0, plot.width, plot.height);
  context.fillStyle = '#101820';
  context.fillRect(0, 0, plot.width, plot.height);

  const chart = {
    left: 54,
    top: 40,
    right: 42,
    bottom: 50,
  };
  const chartWidth = plot.width - chart.left - chart.right;
  const chartHeight = plot.height - chart.top - chart.bottom;
  const maxDistanceMeters = Math.max(8.0, Math.ceil(distanceMeters + 1.0));
  const maxHeightMeters = Math.max(
    3.0,
    config.targetHeightMeters + config.goalRadiusMeters + 0.9,
    (candidate?.peakHeightMeters ?? 0) + 0.4,
  );
  const xToPlot = (xMeters: number) => chart.left + (xMeters / maxDistanceMeters) * chartWidth;
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

  const targetY = yToPlot(config.targetHeightMeters);
  context.strokeStyle = '#22c55e';
  context.lineWidth = 3;
  line(
    context,
    xToPlot(distanceMeters),
    yToPlot(config.targetHeightMeters - config.goalRadiusMeters),
    xToPlot(distanceMeters),
    yToPlot(config.targetHeightMeters + config.goalRadiusMeters),
  );
  context.strokeStyle = '#ef4444';
  line(context, chart.left, targetY, chart.left + chartWidth, targetY);

  if (config.requiresHubOpeningClearance) {
    const lipDistanceMeters = Math.max(0.0, distanceMeters - (config.hubOpeningDepthMeters / 2.0));
    const lipHeightMeters = config.hubFrontLipHeightMeters + config.fuelRadiusMeters + config.hubLipClearanceMeters;
    context.strokeStyle = 'rgba(248, 113, 113, 0.85)';
    context.lineWidth = 5;
    line(context, xToPlot(lipDistanceMeters), yToPlot(0), xToPlot(lipDistanceMeters), yToPlot(lipHeightMeters));
  }

  context.fillStyle = '#dbe7d4';
  context.font = '500 14px IBM Plex Mono, monospace';
  context.fillText(config.requiresHubOpeningClearance ? 'HUB opening/lip clearance active' : 'Passback target: HUB lip clearance disabled', chart.left, 24);

  if (!candidate) return;
  context.strokeStyle = '#f8c537';
  context.lineWidth = 4;
  context.beginPath();
  for (let i = 0; i <= 96; i += 1) {
    const xMeters = (distanceMeters * i) / 96;
    const t = xMeters / ((candidate.exitSpeedMps * Math.cos(candidate.launchAngleDeg * Math.PI / 180)) + radialVelocityMps);
    const yMeters = config.shooterExitHeightMeters
      + (candidate.exitSpeedMps * Math.sin(candidate.launchAngleDeg * Math.PI / 180) * t)
      - (0.5 * config.gravityMps2 * t * t);
    const cx = xToPlot(xMeters);
    const cy = yToPlot(yMeters);
    if (i === 0) context.moveTo(cx, cy);
    else context.lineTo(cx, cy);
  }
  context.stroke();
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
  const marginX = 54;
  const marginY = 66;
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

function syncOutputs(): void {
  for (const label of appRoot.querySelectorAll('label')) {
    const input = label.querySelector('input');
    const output = label.querySelector('output');
    if (input && output) output.textContent = Number(input.value).toFixed(2);
  }
}

function syncUiState(): void {
  appRoot.dataset.page = appState.pageMode;
  document.querySelector('#dragPage')?.classList.toggle('active', appState.pageMode === 'drag');
  document.querySelector('#simPage')?.classList.toggle('active', appState.pageMode === 'sim');
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
  } else {
    robotPose = driveSim.pose;
    const headingInput = document.querySelector<HTMLInputElement>('#heading');
    if (headingInput) headingInput.value = radiansToDegrees(robotPose.headingRad).toFixed(0);
  }
  if (updateHash) window.location.hash = pageMode === 'sim' ? '#sim' : '#drag';
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
  return appState.pageMode === 'sim' ? driveSim.pose : robotPose;
}

function pageModeFromHash(): PageMode {
  return window.location.hash.toLowerCase().includes('sim') ? 'sim' : 'drag';
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

function line(context: CanvasRenderingContext2D, x1: number, y1: number, x2: number, y2: number): void {
  context.beginPath();
  context.moveTo(x1, y1);
  context.lineTo(x2, y2);
  context.stroke();
}

function degreesToRadians(degrees: number): number {
  return (degrees * Math.PI) / 180.0;
}

function radiansToDegrees(radians: number): number {
  return (radians * 180.0) / Math.PI;
}

interface ShotMotion {
  radialVelocityMps: number;
  tangentialVelocityMps: number;
  fieldVelocity: Point;
}
