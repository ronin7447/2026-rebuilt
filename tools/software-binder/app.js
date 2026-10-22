/* Rōnin software binder: explanatory interactions, independent of robot runtime. */
const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
const state = { preset: 'lateral', speed: 1.2, acceleration: 0, rotation: 0, playing: false };
const controls = ['speed', 'acceleration', 'rotation'];
const presets = {
  lateral: { speed: 1.2, acceleration: 0, rotation: 0 },
  approach: { speed: 1.4, acceleration: 0.5, rotation: 0 },
  rotate: { speed: 1.0, acceleration: 0.5, rotation: 90 },
};
const descriptions = {
  lateral: 'As the robot strafes, the turret aims against lateral motion to cancel the velocity Fuel inherits.',
  approach: 'Moving toward the Hub adds forward velocity to Fuel. The shooter supplies the remaining launch velocity.',
  rotate: 'Chassis rotation adds velocity at the offset turret pivot. The aim compensates for translation and rotation together.',
};
let flight = { start: [158, 272], end: [564, 185] };
let frame = null;
let animationStart = null;

function renderShot() {
  const horizon = 0.14; // Constants.ShotMap.BASE_RELEASE_LATENCY_SEC at source revision.
  const scale = 82; // Pixels per illustrative field meter.
  const flightTime = 0.8;
  const axis = state.preset === 'approach' ? [1, 0] : [0, 1];
  const displacement = state.speed * horizon + 0.5 * state.acceleration * horizon ** 2;
  const releaseVelocity = state.speed + state.acceleration * horizon;
  const omega = state.rotation * Math.PI / 180;
  const yaw = omega * horizon;
  // Actual branch turret-pivot offset, rotated into the predicted field frame.
  const r = [-0.1651 * Math.cos(yaw) + 0.2159 * Math.sin(yaw),
    -0.1651 * Math.sin(yaw) - 0.2159 * Math.cos(yaw)];
  const center = [158 + displacement * axis[0] * scale, 272 - displacement * axis[1] * scale];
  const pivot = [center[0] + r[0] * scale, center[1] - r[1] * scale];
  const inherited = [releaseVelocity * axis[0] - omega * r[1], releaseVelocity * axis[1] + omega * r[0]];
  const target = [564, 185];
  // With an illustrative fixed time of flight, required ground velocity is Δp/t.
  const launch = [(target[0] - pivot[0]) / scale / flightTime - inherited[0],
    (pivot[1] - target[1]) / scale / flightTime - inherited[1]];
  const magnitude = Math.hypot(...launch);
  const aim = [pivot[0] + launch[0] / magnitude * 128, pivot[1] - launch[1] / magnitude * 128];
  const motionEnd = [pivot[0] + inherited[0] * 46, pivot[1] - inherited[1] * 46];
  $('#lab-ghost').setAttribute('transform', `translate(${center[0]} ${center[1]}) rotate(${-state.rotation * horizon})`);
  $('#lab-flight').setAttribute('d', `M${pivot.join(' ')} L${target.join(' ')}`);
  $('#lab-aim').setAttribute('d', `M${pivot.join(' ')} L${aim.join(' ')}`);
  $('#lab-motion').setAttribute('d', `M${pivot.join(' ')} L${motionEnd.join(' ')}`);
  $('#lab-motion').style.opacity = Math.hypot(...inherited) < 0.015 ? '0' : '1';
  $('#lab-turret').setAttribute('transform', `rotate(${-Math.atan2(launch[1], launch[0]) * 180 / Math.PI})`);
  $('#release-label').setAttribute('x', center[0] + 43);
  $('#release-label').setAttribute('y', center[1] - 41);
  $('#release-distance').innerHTML = `${(Math.abs(displacement) * 100).toFixed(1)} <small>cm</small>`;
  $('#scenario-text').textContent = descriptions[state.preset];
  $('#speed-value').textContent = `${state.speed.toFixed(1)} m/s`;
  $('#acceleration-value').textContent = `${state.acceleration.toFixed(1)} m/s²`;
  $('#rotation-value').textContent = `${state.rotation}°/s`;
  controls.forEach((key) => {
    $(`#${key}`).value = state[key];
    $(`#${key}`).setAttribute('aria-valuetext', $(`#${key}-value`).textContent);
  });
  $$('[data-preset]').forEach((button) => {
    const active = button.dataset.preset === state.preset;
    button.classList.toggle('selected', active);
    button.setAttribute('aria-pressed', String(active));
  });
  flight = { start: pivot, end: target };
  if (!state.playing) placeFuel(0.56);
}

function placeFuel(t) {
  $('#lab-fuel').setAttribute('cx', flight.start[0] + (flight.end[0] - flight.start[0]) * t);
  $('#lab-fuel').setAttribute('cy', flight.start[1] + (flight.end[1] - flight.start[1]) * t);
}
function animate(time) {
  if (!state.playing) return;
  if (animationStart === null) animationStart = time;
  placeFuel(((time - animationStart) % 2200) / 2200);
  frame = requestAnimationFrame(animate);
}
function setPlaying(playing) {
  state.playing = playing;
  $('#animate').innerHTML = playing ? 'Pause motion <span aria-hidden="true">Ⅱ</span>' : 'Play motion <span aria-hidden="true">▶</span>';
  $('#animate').setAttribute('aria-pressed', String(playing));
  if (frame !== null) cancelAnimationFrame(frame);
  frame = null;
  animationStart = null;
  if (playing) frame = requestAnimationFrame(animate);
}
controls.forEach((key) => $(`#${key}`).addEventListener('input', (event) => {
  state[key] = Number(event.target.value);
  renderShot();
}));
$$('[data-preset]').forEach((button) => button.addEventListener('click', () => {
  state.preset = button.dataset.preset;
  Object.assign(state, presets[state.preset]);
  renderShot();
}));
$('#animate').addEventListener('click', () => setPlaying(!state.playing));
$('#reset').addEventListener('click', () => {
  setPlaying(false);
  Object.assign(state, { preset: 'lateral', ...presets.lateral });
  renderShot();
});
document.addEventListener('visibilitychange', () => { if (document.hidden) setPlaying(false); });

const stages = {
  ready: {
    timing: 't = 0 ms · Readiness established', title: 'The whole shot gets the green light.',
    description: 'The solution is fresh and feasible. The flywheel, hood, and turret meet their targets. Measured and predicted motion fit the configured shooting envelope.',
    outtake: false, indexer: false,
  },
  outtake: {
    timing: 't = 50 ms · Final feed starts', title: 'Start at the shooter. Build the stream.',
    description: 'After 50 ms of staging, the final-feed motor starts. The downstream transport comes online before the spindexer supplies more Fuel.',
    outtake: true, indexer: false,
  },
  indexer: {
    timing: 't = 75 ms · Spindexer joins', title: 'The Fuel path runs as one.',
    description: 'An additional 25 ms brings the spindexer online. Both feed motors run while the controller continuously evaluates the shot and mechanism readiness.',
    outtake: true, indexer: true,
  },
};
function renderStage(key) {
  const stage = stages[key];
  $('#stage-timing').textContent = stage.timing;
  $('#stage-title').textContent = stage.title;
  $('#stage-description').textContent = stage.description;
  for (const motor of ['outtake', 'indexer']) {
    const row = $(`#${motor}-status`);
    row.classList.toggle('running', stage[motor]);
    row.querySelector('b').textContent = stage[motor] ? 'Running' : 'Staging';
  }
  $$('[data-stage]').forEach((button) => {
    const active = button.dataset.stage === key;
    button.classList.toggle('selected', active);
    button.setAttribute('aria-pressed', String(active));
  });
}
$$('[data-stage]').forEach((button) => button.addEventListener('click', () => renderStage(button.dataset.stage)));

const sourceRevision = '3323e73';
$$('[data-source]').forEach((link) => {
  link.href = `https://github.com/ronin7447/2026-rebuilt/blob/${sourceRevision}/${link.dataset.source}`;
  link.target = '_blank';
  link.rel = 'noopener noreferrer';
  link.setAttribute('aria-label', `${link.textContent.trim().replace('↗', '').trim()} source code (opens in new tab)`);
});
const nav = $$('.main-nav a');
function onScroll() {
  const height = document.documentElement.scrollHeight - window.innerHeight;
  $('#progress').style.width = `${height > 0 ? window.scrollY / height * 100 : 0}%`;
  let active = null;
  nav.forEach((link) => { if ($(link.hash).getBoundingClientRect().top <= 150) active = link; });
  nav.forEach((link) => {
    link.classList.toggle('active', link === active);
    if (link === active) link.setAttribute('aria-current', 'location');
    else link.removeAttribute('aria-current');
  });
}
let scrollScheduled = false;
window.addEventListener('scroll', () => {
  if (scrollScheduled) return;
  scrollScheduled = true;
  requestAnimationFrame(() => { onScroll(); scrollScheduled = false; });
}, { passive: true });
window.addEventListener('resize', onScroll);
// Expand the technical binder for printing, then restore the reader's choices.
let previousDetails = [];
function preparePrint() {
  setPlaying(false);
  if (!previousDetails.length) previousDetails = $$('details').map((detail) => [detail, detail.open]);
  $$('details').forEach((detail) => { detail.open = true; });
}
window.addEventListener('beforeprint', preparePrint);
window.addEventListener('afterprint', () => {
  previousDetails.forEach(([detail, open]) => { detail.open = open; });
  previousDetails = [];
});
$('#print').addEventListener('click', () => window.print());
renderShot();
renderStage('ready');
onScroll();
