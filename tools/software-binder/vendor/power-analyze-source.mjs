import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ALIASES = {
  timestampSec: ['timestamp_sec', 'timestamp', 'time_sec', 'time_s', 'power_timestamp_sec'],
  timestampMs: ['timestamp_ms', 'time_ms'],
  voltageV: ['pdh_voltage_v', 'voltage_v', 'battery_voltage_v', 'power_pdh_voltage_v'],
  totalCurrentA: ['pdh_total_current_a', 'total_current_a', 'power_pdh_total_current_a'],
  brownedOut: ['browned_out', 'rio_browned_out', 'power_robo_rio_browned_out'],
  shooterTargetRps: ['shooter_target_rps', 'power_shooter_target_rps'],
  shooterVelocityRps: ['shooter_velocity_rps', 'power_motor_shooter_roller_velocity_rps'],
  intakePositionRot: ['intake_position_rot', 'power_motor_intake_extension_master_position_rot'],
};

export function parseCsv(text) {
  const lines = text.split(/\r?\n/).filter((line) => line.trim() !== '');
  if (lines.length < 2) throw new Error('CSV must contain a header and at least one data row');
  const headers = splitCsvLine(lines[0]).map(canonicalHeader);
  const timestampSecIndex = findAlias(headers, ALIASES.timestampSec);
  const timestampMsIndex = findAlias(headers, ALIASES.timestampMs);
  if (timestampSecIndex < 0 && timestampMsIndex < 0) {
    throw new Error(`Missing timestamp column; accepted names: ${[...ALIASES.timestampSec, ...ALIASES.timestampMs].join(', ')}`);
  }

  const rows = lines.slice(1).map((line, sourceIndex) => {
    const values = splitCsvLine(line);
    if (values.length !== headers.length) {
      throw new Error(`Row ${sourceIndex + 2} has ${values.length} fields; expected ${headers.length}`);
    }
    const timestampRaw = numeric(values[timestampSecIndex >= 0 ? timestampSecIndex : timestampMsIndex]);
    if (!Number.isFinite(timestampRaw)) throw new Error(`Row ${sourceIndex + 2} has an invalid timestamp`);
    const timestampSec = timestampSecIndex >= 0 ? timestampRaw : timestampRaw / 1000.0;
    const raw = Object.fromEntries(headers.map((header, index) => [header, values[index].trim()]));
    return {
      timestampSec,
      voltageV: valueFor(raw, headers, ALIASES.voltageV),
      totalCurrentA: valueFor(raw, headers, ALIASES.totalCurrentA),
      brownedOut: boolFor(raw, headers, ALIASES.brownedOut),
      shooterTargetRps: valueFor(raw, headers, ALIASES.shooterTargetRps),
      shooterVelocityRps: valueFor(raw, headers, ALIASES.shooterVelocityRps),
      intakePositionRot: valueFor(raw, headers, ALIASES.intakePositionRot),
      mechanismSupplyCurrentA: Object.fromEntries(headers
        .filter((header) => header.endsWith('_supply_current_a'))
        .map((header) => [header.replace(/^power_motor_/, '').replace(/_supply_current_a$/, ''), numeric(raw[header])])),
      raw,
    };
  }).sort((a, b) => a.timestampSec - b.timestampSec);

  for (let index = 1; index < rows.length; index++) {
    if (rows[index].timestampSec === rows[index - 1].timestampSec) {
      throw new Error(`Duplicate timestamp ${rows[index].timestampSec} sec`);
    }
  }
  return { headers, rows };
}

export function analyzeRows(rows, options = {}) {
  const settings = {
    voltageDipThresholdV: options.voltageDipThresholdV ?? 7.0,
    joinMaxAgeSec: options.joinMaxAgeSec ?? 0.05,
    eventWindowSec: options.eventWindowSec ?? 0.5,
    shooterRecoveryToleranceRps: options.shooterRecoveryToleranceRps ?? 2.0,
    currentLimitsA: options.currentLimitsA ?? {},
  };
  if (!rows.length) throw new Error('No data rows to analyze');

  const finiteVoltages = rows.filter((row) => Number.isFinite(row.voltageV));
  const minimumVoltageRow = finiteVoltages.reduce(
    (best, row) => best === null || row.voltageV < best.voltageV ? row : best,
    null,
  );
  const voltageDips = contiguousEvents(rows, (row) => Number.isFinite(row.voltageV)
    && row.voltageV <= settings.voltageDipThresholdV, settings.joinMaxAgeSec);
  const brownouts = contiguousEvents(rows, (row) => row.brownedOut === true, settings.joinMaxAgeSec);
  const shooterRecoveries = recoveryEvents(rows, settings.shooterRecoveryToleranceRps, settings.joinMaxAgeSec);
  const intakeMovements = movementEvents(rows, settings.joinMaxAgeSec);
  const nearLimitOccupancy = {};
  for (const [mechanism, limitA] of Object.entries(settings.currentLimitsA)) {
    nearLimitOccupancy[mechanism] = durationWhere(
      rows,
      (row) => Number.isFinite(row.mechanismSupplyCurrentA[mechanism])
        && row.mechanismSupplyCurrentA[mechanism] >= 0.9 * limitA,
      settings.joinMaxAgeSec,
    );
  }

  const summedMotorSupplyCurrent = rows.map((row) => {
    const values = Object.values(row.mechanismSupplyCurrentA).filter(Number.isFinite);
    return { timestampSec: row.timestampSec, valueA: values.length ? values.reduce((sum, value) => sum + value, 0) : null };
  });
  const eventTimes = [...brownouts.map((event) => event.startSec), ...(minimumVoltageRow ? [minimumVoltageRow.timestampSec] : [])];
  const eventWindows = eventTimes.map((eventTimeSec) => ({
    eventTimeSec,
    rows: rows.filter((row) => Math.abs(row.timestampSec - eventTimeSec) <= settings.eventWindowSec),
  }));

  return {
    schemaVersion: 1,
    settings,
    sampleCount: rows.length,
    timeRangeSec: [rows[0].timestampSec, rows.at(-1).timestampSec],
    minimumVoltage: minimumVoltageRow
      ? { timestampSec: minimumVoltageRow.timestampSec, voltageV: minimumVoltageRow.voltageV }
      : null,
    voltageDips,
    brownouts,
    shooterRecoveries,
    intakeMovements,
    nearLimitOccupancy,
    pdhTotalCurrent: rows.map((row) => ({ timestampSec: row.timestampSec, valueA: finiteOrNull(row.totalCurrentA) })),
    summedMotorSupplyCurrent,
    eventWindows,
    limitations: [
      'PDH total current and summed motor supply current are distinct measurements.',
      'Stator current is never used as battery-current attribution.',
      'Near-limit occupancy is inferred unless an authoritative limiter flag is present.',
      'Missing samples are preserved; values are not forward-filled across gaps.',
      'Uninstrumented loads and asynchronous device sampling prevent exact attribution.',
    ],
  };
}

export function writeReport(inputPath, outputDirectory, identity, options = {}) {
  const parsed = parseCsv(fs.readFileSync(inputPath, 'utf8'));
  const analysis = analyzeRows(parsed.rows, options);
  const report = { identity, inputPath: path.resolve(inputPath), ...analysis };
  fs.mkdirSync(outputDirectory, { recursive: true });
  fs.writeFileSync(path.join(outputDirectory, 'report.json'), `${JSON.stringify(report, null, 2)}\n`);
  fs.writeFileSync(path.join(outputDirectory, 'power.svg'), createPlotSvg(parsed.rows));
  return report;
}

function contiguousEvents(rows, predicate, maxGapSec) {
  const events = [];
  let current = null;
  for (const row of rows) {
    if (predicate(row)) {
      if (!current || row.timestampSec - current.lastSampleSec > maxGapSec) {
        if (current) events.push(finishEvent(current));
        current = { startSec: row.timestampSec, endSec: row.timestampSec, lastSampleSec: row.timestampSec, samples: 1 };
      } else {
        current.endSec = row.timestampSec;
        current.lastSampleSec = row.timestampSec;
        current.samples += 1;
      }
    } else if (current) {
      events.push(finishEvent(current));
      current = null;
    }
  }
  if (current) events.push(finishEvent(current));
  return events;
}

function finishEvent(event) {
  return { startSec: event.startSec, endSec: event.endSec, durationSec: event.endSec - event.startSec, samples: event.samples };
}

function durationWhere(rows, predicate, maxGapSec) {
  let durationSec = 0;
  for (let index = 1; index < rows.length; index++) {
    const dt = rows[index].timestampSec - rows[index - 1].timestampSec;
    if (dt <= maxGapSec && predicate(rows[index - 1]) && predicate(rows[index])) durationSec += dt;
  }
  return durationSec;
}

function recoveryEvents(rows, toleranceRps, maxGapSec) {
  const events = [];
  let start = null;
  for (let index = 0; index < rows.length; index++) {
    const row = rows[index];
    const error = Number.isFinite(row.shooterTargetRps) && Number.isFinite(row.shooterVelocityRps)
      ? Math.abs(row.shooterTargetRps - row.shooterVelocityRps) : null;
    if (error !== null && error > toleranceRps && start === null) start = row.timestampSec;
    if (start !== null && error !== null && error <= toleranceRps) {
      events.push({ startSec: start, recoveredSec: row.timestampSec, durationSec: row.timestampSec - start });
      start = null;
    }
    if (index > 0 && row.timestampSec - rows[index - 1].timestampSec > maxGapSec) start = null;
  }
  return events;
}

function movementEvents(rows, maxGapSec) {
  const events = [];
  let start = null;
  for (let index = 1; index < rows.length; index++) {
    const previous = rows[index - 1];
    const row = rows[index];
    if (row.timestampSec - previous.timestampSec > maxGapSec) { start = null; continue; }
    if (!Number.isFinite(previous.intakePositionRot) || !Number.isFinite(row.intakePositionRot)) continue;
    const moving = Math.abs(row.intakePositionRot - previous.intakePositionRot) > 0.002;
    if (moving && start === null) start = previous.timestampSec;
    if (!moving && start !== null) {
      events.push({ startSec: start, endSec: row.timestampSec, durationSec: row.timestampSec - start });
      start = null;
    }
  }
  return events;
}

export function createPlotSvg(rows) {
  const width = 1100; const height = 660; const pad = 55;
  const times = rows.map((row) => row.timestampSec);
  const minT = Math.min(...times); const maxT = Math.max(...times);
  const x = (time) => pad + ((time - minT) / Math.max(maxT - minT, 1e-9)) * (width - 2 * pad);
  const voltageTop = 55; const voltageBottom = 275;
  const currentTop = 355; const currentBottom = 600;
  const points = (valueForRow, maxValue, top, bottom) => rows
    .map((row) => ({ row, value: valueForRow(row) }))
    .filter(({ value }) => Number.isFinite(value))
    .map(({ row, value }) => `${x(row.timestampSec).toFixed(1)},${(bottom - (value / maxValue) * (bottom - top)).toFixed(1)}`)
    .join(' ');
  const mechanisms = [...new Set(rows.flatMap((row) => Object.keys(row.mechanismSupplyCurrentA)))].sort();
  const currentValues = rows.flatMap((row) => [row.totalCurrentA, ...Object.values(row.mechanismSupplyCurrentA)])
    .filter(Number.isFinite);
  const currentMax = Math.max(1, ...currentValues) * 1.1;
  const mechanismLines = mechanisms.map((mechanism, index) => {
    const color = `hsl(${Math.round(index * 360 / Math.max(1, mechanisms.length))} 70% 42%)`;
    return `<polyline fill="none" stroke="${color}" stroke-width="1.5" points="${points(
      (row) => row.mechanismSupplyCurrentA[mechanism], currentMax, currentTop, currentBottom,
    )}"/><text x="${pad + (index % 4) * 255}" y="${315 + Math.floor(index / 4) * 15}" fill="${color}" font-size="11">${escapeXml(mechanism)}</text>`;
  }).join('\n');
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}">
<rect width="100%" height="100%" fill="white"/><text x="${pad}" y="25">PDH voltage (0–16 V)</text>
<line x1="${pad}" y1="${voltageBottom}" x2="${width - pad}" y2="${voltageBottom}" stroke="#94a3b8"/>
<polyline fill="none" stroke="#2563eb" stroke-width="2" points="${points((row) => row.voltageV, 16, voltageTop, voltageBottom)}"/>
<text x="${pad}" y="335">Supply current (shared scale 0–${currentMax.toFixed(1)} A): PDH total is black; motor mechanisms are colored</text>
<line x1="${pad}" y1="${currentBottom}" x2="${width - pad}" y2="${currentBottom}" stroke="#94a3b8"/>
<polyline fill="none" stroke="#111827" stroke-width="2.5" points="${points((row) => row.totalCurrentA, currentMax, currentTop, currentBottom)}"/>
${mechanismLines}
</svg>\n`;
}

function splitCsvLine(line) {
  const values = []; let current = ''; let quoted = false;
  for (let index = 0; index < line.length; index++) {
    const char = line[index];
    if (char === '"' && line[index + 1] === '"' && quoted) { current += '"'; index += 1; }
    else if (char === '"') quoted = !quoted;
    else if (char === ',' && !quoted) { values.push(current); current = ''; }
    else current += char;
  }
  values.push(current); return values;
}
function canonicalHeader(value) {
  return value.trim().replace(/^\/+/, '').replace(/([A-Z]+)([A-Z][a-z])/g, '$1_$2')
    .replace(/([a-z0-9])([A-Z])/g, '$1_$2')
    .toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '');
}
function escapeXml(value) { return value.replace(/[&<>"']/g, (char) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&apos;' }[char])); }
function findAlias(headers, aliases) { return aliases.map((alias) => headers.indexOf(alias)).find((index) => index >= 0) ?? -1; }
function valueFor(raw, headers, aliases) { const index = findAlias(headers, aliases); return index >= 0 ? numeric(raw[headers[index]]) : Number.NaN; }
function boolFor(raw, headers, aliases) { const index = findAlias(headers, aliases); if (index < 0) return null; const value = raw[headers[index]].toLowerCase(); return value === 'true' || value === '1' ? true : value === 'false' || value === '0' ? false : null; }
function numeric(value) { return value === undefined || value.trim() === '' ? Number.NaN : Number(value); }
function finiteOrNull(value) { return Number.isFinite(value) ? value : null; }

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  const [inputPath, outputDirectory, batteryLabel = 'UNKNOWN', configurationId = 'UNKNOWN'] = process.argv.slice(2);
  if (!inputPath || !outputDirectory) {
    console.error('Usage: npm run analyze -- <export.csv> <output-directory> [battery-label] [configuration-id]');
    process.exitCode = 2;
  } else {
    const report = writeReport(inputPath, outputDirectory, { batteryLabel, configurationId });
    console.log(`Wrote ${report.sampleCount} samples to ${path.resolve(outputDirectory)}`);
  }
}
