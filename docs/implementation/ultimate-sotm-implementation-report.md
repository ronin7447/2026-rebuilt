# Ultimate SOTM implementation report

Date: 2026-09-12  
Starting revision: `bb932a869bd61fdbb047fdd811c1bb70bc5d6f76` (`ultimate-sotm`)  
Configuration ID: `ultimate-sotm-v1`

The 2026-10-08 [motion-envelope follow-up](ultimate-sotm-motion-envelope.md) adds shared timestamped acceleration estimation, motion readiness gates, and teleop command ramps under configuration `ultimate-sotm-v2-motion`. The evidence below describes the original v1 implementation.

## Status

| Area | Implemented | Software verified | Hardware verified | Competition qualified |
|---|---|---|---|---|
| Coordinate/evaluator contract | yes | yes | pending | no |
| Synchronized power/health recording | yes | failure paths tested | pending | no |
| Offline power analysis | yes | synthetic CSV tests pass | pending on real logs | no |
| Continuous feed/vision policy | yes | state tests pass | pending | no |
| Turret diagnostics/Test tuning | yes | pure metric tests pass | pending | no |
| Homing/config/simulation stability | yes | state/retry tests pass | pending | no |

## Preserved starting work and intentional differences

The starting edits to `RobustShotModel`, its tests, and the shot calculator were retained and extended. The approved coordinate differences remove the Java-only red Hub `0.20 m` offset and set blue passback Y to `6.0/2.5`. Generated launch calibration coefficients and all current limits remain unchanged.

The final compensated trajectory now owns feasibility, target-plane crossing time, opening intersection, Hub lip clearance, and descending entry angle. Heuristic robustness is reported separately. Hub polynomial commands and bounded close-range direct commands remain distinct; passback uses the calculator's flat-fast bounded sweep with quantized candidate inputs and continuous final evaluation. Rejection does not silently fall back to legacy.

Shared fixture data: `shared/shot-evaluator-fixtures-v1.csv`. The fixture convention uses literal `NaN` for representable nonfinite rejection. Tolerances are included per row.

## Provisional software defaults

- REV PDH CAN ID `1` — verify on hardware.
- Hood tolerance `0.02 rot` — measure under load.
- Intake homing timeout `3.0 s` at `-1.0 V` — bench validate travel/stall behavior.
- Release latency `0.14 s` — measure actual Fuel departure.
- Battery label `UNKNOWN` — set per run.

## Software verification

Run from the preserved working tree on 2026-09-12:

- `./gradlew test --offline` — 56 tests passed. This includes shared evaluator parity, coordinate, feed transition/override, vision timestamp, logger failure/staleness/channel-label, homing, configuration retry, and turret metric cases. The intentional failed-configuration test prints a Driver Station error while passing.
- `npm test && npm run build` in `tools/shot-calculator` — 28 tests passed and the TypeScript/Vite production build succeeded.
- `npm test` in `tools/power-analysis` — 5 synthetic CSV/analyzer tests passed, including direct WPILib DataLog column names and per-mechanism SVG output.
- `git diff --check` — passed with no whitespace errors.

The Java compiler reports six existing Phoenix `TalonFX(int, String)` deprecation warnings. No test or build failures remain. These are desktop/software results only; roboRIO solver duration and loop timing remain unmeasured.

## Hardware-pending gates

Verify the PDH/channel map, roboRIO model/version, USB availability, signal physical rates/ages, firmware/configuration readback, mechanism zeroing, hood tolerance, release latency, turret tuning, solver worst-case roboRIO duration under full load, moving Hub/passback scoring, blocked-tag operation, and three consecutive 150-second combined-load trials across representative serviceable batteries.

Simulation or desktop success cannot mark these gates complete. Use the run sheet, power dictionary/analyzer, turret field guide, and deployment/rollback checklist for the next field session.
