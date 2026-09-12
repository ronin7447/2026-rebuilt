import test from 'node:test';
import assert from 'node:assert/strict';
import { analyzeRows, createPlotSvg, parseCsv } from '../analyze.mjs';

test('sorts reordered millisecond records and finds a known voltage dip', () => {
  const { rows } = parseCsv(`timestamp_ms,pdh_voltage_v,pdh_total_current_a,browned_out\n100,6.8,100,true\n0,12.5,0,false\n50,7.2,80,false\n150,8.0,20,false\n`);
  const result = analyzeRows(rows, { voltageDipThresholdV: 7.0, joinMaxAgeSec: 0.06 });
  assert.deepEqual(result.timeRangeSec, [0, 0.15]);
  assert.equal(result.minimumVoltage.voltageV, 6.8);
  assert.equal(result.voltageDips.length, 1);
  assert.equal(result.brownouts.length, 1);
});

test('preserves missing measurements and zero current', () => {
  const { rows } = parseCsv(`timestamp_sec,pdh_voltage_v,pdh_total_current_a\n0,12.0,0\n0.02,,0\n`);
  assert.ok(Number.isNaN(rows[1].voltageV));
  const result = analyzeRows(rows);
  assert.equal(result.pdhTotalCurrent[0].valueA, 0);
  assert.equal(result.minimumVoltage.voltageV, 12);
});

test('does not bridge an event across a timestamp gap', () => {
  const { rows } = parseCsv(`timestamp_sec,pdh_voltage_v\n0,6.5\n1,6.4\n`);
  const result = analyzeRows(rows, { joinMaxAgeSec: 0.05 });
  assert.equal(result.voltageDips.length, 2);
});

test('rejects missing timestamps and malformed rows', () => {
  assert.throws(() => parseCsv('voltage_v\n12\n'), /Missing timestamp/);
  assert.throws(() => parseCsv('timestamp_sec,voltage_v\n0,12,extra\n'), /expected 2/);
});

test('accepts direct WPILib DataLog names and plots each motor supply stream', () => {
  const { rows } = parseCsv(`Power/TimestampSec,Power/PDHVoltageV,Power/PDHTotalCurrentA,Power/Motor/turret/SupplyCurrentA,Power/Motor/shooter-roller/VelocityRps,Power/Motor/intake-extension-master/PositionRot\n0,12.4,30,4,20,0\n0.02,12.1,45,8,40,1\n`);
  assert.equal(rows[1].mechanismSupplyCurrentA.turret, 8);
  assert.equal(rows[1].shooterVelocityRps, 40);
  assert.equal(rows[1].intakePositionRot, 1);
  const svg = createPlotSvg(rows);
  assert.match(svg, /turret/);
  assert.match(svg, /PDH total is black/);
});
