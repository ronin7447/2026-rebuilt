// Presentation bridge appended to the existing calculator module by sync-tools.mjs.
function renderBinderReadout(evaluation: ShotEvaluation): void {
  const root = document.querySelector('#binder-readout');
  if (!root) return;
  const profile = evaluation.turretSideProfile;
  const metric = (name: string, value: string) => `<div><span>${name}</span><strong>${value}</strong></div>`;
  root.innerHTML = [
    metric('Target', escapeHtml(evaluation.target.shortLabel)),
    metric('Release distance', `${evaluation.distanceMeters.toFixed(2)} m`),
    metric('Radial / lateral', `${evaluation.radialVelocityMps.toFixed(2)} / ${evaluation.tangentialVelocityMps.toFixed(2)} m/s`),
    metric('Polynomial hood', evaluation.hoodPrediction === null ? 'Direct search' : `${evaluation.hoodPrediction.toFixed(3)} rot`),
    metric('Polynomial flywheel', evaluation.rollerPrediction === null ? 'Direct search' : `${Math.abs(evaluation.rollerPrediction).toFixed(1)} RPS`),
    metric('Turret goal', `${evaluation.turretSolution.legalGoalTurretYawDeg.toFixed(1)}°`),
    metric('Flight time', profile && Number.isFinite(profile.flightTimeToTargetSec) ? `${profile.flightTimeToTargetSec.toFixed(2)} s` : '—'),
    metric('Trajectory geometry', profile?.targetOpeningValid ? 'Feasible' : 'Explore another state'),
  ].join('');
}
const binderToolbar = document.createElement('div');
binderToolbar.className = 'binder-toolbar';
binderToolbar.innerHTML = '<span>Original shot-calculator engine</span><button id="binder-tuning" type="button" aria-expanded="false">Show engineering controls</button>';
appRoot.prepend(binderToolbar);
document.querySelector('#binder-tuning')?.addEventListener('click', (event) => {
  const active = document.body.classList.toggle('advanced');
  const button = event.currentTarget as HTMLButtonElement;
  button.textContent = active ? 'Hide engineering controls' : 'Show engineering controls';
  button.setAttribute('aria-expanded', String(active));
});
const readout = document.createElement('div');
readout.id = 'binder-readout';
readout.setAttribute('aria-label', 'Live calculated shot values');
document.querySelector('.tuning-workspace')?.append(readout);
const samples = document.createElement('div');
samples.className = 'binder-samples';
samples.innerHTML = '<span>Replay a repository path</span><button type="button" data-path="BlueLeftSOTM">Moving shot</button><button type="button" data-path="BlueLeftSweepLoop">Fuel sweep</button><button type="button" data-path="BlueLeftWideSweepLoop">Wide sweep</button><span id="sample-message" role="status"></span>';
document.querySelector('.panel')?.after(samples);
samples.querySelectorAll<HTMLButtonElement>('[data-path]').forEach(button => button.addEventListener('click', async () => {
  const name = button.dataset.path!;
  try {
    const response = await fetch(`../assets/paths/${name}.path`);
    if (!response.ok) throw new Error('Path could not be loaded. Choose another path.');
    setPageMode('path');
    await loadPathFile(new File([await response.text()], `${name}.path`));
    document.querySelector('#sample-message')!.textContent = `${name} loaded. Press Play to run.`;
  } catch (error) {
    document.querySelector('#sample-message')!.textContent = error instanceof Error ? error.message : 'Path could not be loaded.';
  }
}));
fieldCanvas.setAttribute('aria-label', 'Interactive REBUILT field: drag to place the robot. Use Keyboard Sim for keyboard driving.');
fieldCanvas.setAttribute('tabindex', '0');
plot.setAttribute('aria-label', 'Calculated side-view trajectory, Hub opening, front lip, and entry geometry.');
document.querySelector('.panel h1')!.textContent = 'Explore the shot.';
document.querySelector('.panel .eyebrow')!.textContent = 'Rōnin · Physics in motion';
document.querySelector('.panel .summary')!.textContent = 'Place the robot on the field. Change its motion. Watch the turret, flight path, and launch settings respond.';
window.addEventListener('blur', () => keyboard.keys.clear());
let lastHeight = 0;
new ResizeObserver(() => {
  const height = Math.ceil(appRoot.getBoundingClientRect().height + 8);
  if (height !== lastHeight) {
    lastHeight = height;
    parent.postMessage({ type: 'ronin-calculator-height', height }, location.origin);
  }
}).observe(appRoot);
render();
window.addEventListener('message', async event => {
  if (event.origin !== location.origin || event.source !== parent || event.data?.type !== 'ronin-replay') return;
  const allowed = ['BlueLeftSOTM', 'BlueLeftSweepLoop', 'BlueLeftWideSweepLoop'];
  if (!allowed.includes(event.data.path)) return;
  const button = samples.querySelector<HTMLButtonElement>(`[data-path="${event.data.path}"]`);
  button?.click();
});
parent.postMessage({ type: 'ronin-calculator-ready' }, location.origin);
