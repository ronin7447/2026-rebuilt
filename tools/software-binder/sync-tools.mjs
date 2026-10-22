import { cp, mkdir, readFile, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';
import { createHash } from 'node:crypto';

// Snapshot the branch tools without changing robot or engineering-tool sources.
const repo = resolve('../..');
await mkdir('vendor/shot-calculator', { recursive: true });
await cp('../shot-calculator/src', 'vendor/shot-calculator/src', { recursive: true });
await cp('../shot-calculator/tsconfig.json', 'vendor/shot-calculator/tsconfig.json');
await cp('../power-analysis/analyze.mjs', 'vendor/power-analyze-source.mjs');
await mkdir('assets/paths', { recursive: true });
for (const name of ['BlueLeftSOTM', 'BlueLeftSweepLoop', 'BlueLeftWideSweepLoop']) {
  await cp(`${repo}/src/main/deploy/pathplanner/paths/${name}.path`, `assets/paths/${name}.path`);
}
const original = await readFile('../shot-calculator/src/main.ts', 'utf8');
let main = original.replace('  renderMetrics(evaluation);', '  renderMetrics(evaluation);\n  renderBinderReadout(evaluation);');
// Presentation terminology: scores from the simulator remain explicitly model scores.
main = main.replaceAll('estimated accuracy', 'model score').replaceAll('estimated Accuracy', 'model score')
  .replaceAll('Required estimated accuracy (%)', 'Required model score (%)')
  .replaceAll('Worst Acc', 'Model score').replaceAll('% accuracy,', '% model score,')
  .replaceAll('No full guarantee found; applied best tradeoff:', 'Applied highest-ranked tradeoff:')
  .replaceAll('Applied max qualified envelope:', 'Applied selected simulation envelope:');
main += '\n' + await readFile('calculator-bridge.ts', 'utf8');
await writeFile('vendor/shot-calculator/src/main.ts', main);
await writeFile('vendor/shot-calculator/index.html', '<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Rōnin · Live shot laboratory</title><link rel="icon" href="../../favicon.svg"><link rel="stylesheet" href="./binder.css"></head><body><main id="app"></main><script type="module" src="./src/main.ts"></script></body></html>');
await cp('calculator.css', 'vendor/shot-calculator/binder.css');
const power = await readFile('../power-analysis/analyze.mjs', 'utf8');
const browserPower = power.replace(/^import .*;\n/gm, '')
  .replace(/export function writeReport\([\s\S]*?(?=function contiguousEvents)/, '')
  .replace(/\nif \(process\.argv\[1\][\s\S]*$/, '');
await mkdir('assets', { recursive: true });
await writeFile('assets/power-core.mjs', browserPower);
// Retain the exact, already-authored sample inputs from the analyzer's fixture suite.
const fixtureSource = await readFile('../power-analysis/test/analyze.test.mjs', 'utf8');
const fixtures = [...fixtureSource.matchAll(/parseCsv\(`([^`]+)`\)/g)].map(match => match[1].replaceAll('\\n', '\n'));
await writeFile('assets/power-voltage-fixture.csv', fixtures[0]);
await writeFile('assets/power-mechanism-fixture.csv', fixtures.at(-1));
const coeffSource = await readFile('../shot-calculator/src/math/generatedCoefficients.ts', 'utf8');
const coefficients = Object.fromEntries(['hood', 'roller'].map(key => [key,
  coeffSource.match(new RegExp(`${key}: \\[([^\\]]+)`))[1].split(',').map(x => x.trim()).filter(Boolean).map(Number)]));
await writeFile('assets/shotmap.json', JSON.stringify(coefficients, null, 2));
await writeFile('assets/provenance.json', JSON.stringify({ branch: 'ultimate-sotm', revision: '3323e73', calculatorSha256: createHash('sha256').update(original).digest('hex'), powerAnalyzerSha256: createHash('sha256').update(power).digest('hex'), calculatorPresentation: 'Blue presentation skin, concise live readout, repository path presets, model-score labels. Physics, trajectory, path sampling and controller implementations retained.', powerPresentation: 'Node filesystem and CLI entry points removed; parseCsv, analyzeRows and their helpers retained.' }, null, 2));
console.log('Synced calculator, analyzer core, coefficients, path files, and original sample fixtures.');
