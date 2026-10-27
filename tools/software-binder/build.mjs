import { mkdir, copyFile, cp, access } from 'node:fs/promises';
import { spawnSync } from 'node:child_process';
import { resolve } from 'node:path';
await mkdir('dist', { recursive: true });
for (const file of ['index.html', 'styles.css', 'app.js', 'deep-dive.js', 'deep-dive.css', 'favicon.svg']) {
  await copyFile(file, `dist/${file}`);
}
await cp('assets', 'dist/assets', { recursive: true });
// The neighboring calculator provides the build runtime in the robot checkout.
// A standalone checkout may install its own pinned dev dependency instead.
const candidates = [process.env.RONIN_VITE_BIN, resolve('node_modules/vite/bin/vite.js'), resolve('../shot-calculator/node_modules/vite/bin/vite.js')].filter(Boolean);
let vite;
for (const candidate of candidates) { try { await access(candidate); vite = candidate; break; } catch {} }
if (!vite) throw new Error('Install the pinned Vite dependency or set RONIN_VITE_BIN.');
const result = spawnSync(process.execPath, [vite, 'build', 'vendor/shot-calculator', '--base=./', '--outDir', resolve('dist/calculator')], { stdio: 'inherit' });
if (result.status !== 0) process.exit(result.status || 1);
console.log('Built the software binder in dist/');
