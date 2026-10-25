# Far-shot comparison

Gravity-only, level robot, perfect actuators, current regression. Distance/velocity at release. Trim treated as physical speed change with nominal lead preserved. Plane diagnostic is horizontal rim and rectangular depth proxy, not full hex/net geometry. Not measured scoring probability.

Stationary values before the empirical 5% RPS trim. Negative hood rotations follow current calibration. Positive rim-plane offset means beyond Hub center; negative means nearer the shooter.

| Model | Distance m | Angle deg | Hood rot | RPS magnitude | Flight s | Lip margin m | Entry deg | Rim-plane offset m | Legal | Geometric gate | Rim proxy |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| fixed63 | 4.0 | 63.00 | -2.090 | 73.16 | 1.138 | 0.513 | 50.5 | 0.002 | True | True | True |
| fixed63 | 4.5 | 63.00 | -2.090 | 75.83 | 1.226 | 0.538 | 52.7 | -0.027 | True | True | True |
| fixed63 | 4.9 | 63.00 | -2.090 | 77.92 | 1.292 | 0.551 | 54.0 | -0.046 | True | True | True |
| fixed63 | 5.0 | 63.00 | -2.090 | 78.45 | 1.307 | 0.554 | 54.3 | -0.050 | True | True | True |
| fixed63 | 5.5 | 63.00 | -2.090 | 81.02 | 1.384 | 0.569 | 55.4 | -0.065 | True | True | True |
| fixed63 | 6.0 | 63.00 | -2.090 | 83.55 | 1.455 | 0.585 | 56.3 | -0.073 | True | True | True |
| fixed63 | 7.0 | 63.00 | -2.090 | 88.49 | 1.587 | 0.630 | 57.4 | -0.070 | True | True | True |
| fixed63 | 8.0 | 63.00 | -2.090 | 93.24 | 1.706 | 0.698 | 58.1 | -0.044 | True | True | True |
| active-blend | 4.0 | 67.09 | -1.739 | 75.45 | 1.278 | 0.661 | 58.7 | -0.030 | True | True | True |
| active-blend | 4.5 | 66.60 | -1.781 | 78.18 | 1.350 | 0.693 | 59.0 | -0.027 | True | True | True |
| active-blend | 4.9 | 66.25 | -1.811 | 80.28 | 1.405 | 0.714 | 59.2 | -0.025 | True | True | True |
| active-blend | 5.0 | 65.94 | -1.838 | 80.62 | 1.409 | 0.712 | 58.8 | -0.021 | True | True | True |
| active-blend | 5.5 | 63.00 | -2.090 | 81.02 | 1.384 | 0.569 | 55.4 | -0.065 | True | True | True |
| active-blend | 6.0 | 63.00 | -2.090 | 83.55 | 1.455 | 0.585 | 56.3 | -0.073 | True | True | True |
| active-blend | 7.0 | 63.00 | -2.090 | 88.49 | 1.587 | 0.630 | 57.4 | -0.070 | True | True | True |
| active-blend | 8.0 | 63.00 | -2.090 | 93.24 | 1.706 | 0.698 | 58.1 | -0.044 | True | True | True |
| no-edge-fit | 4.0 | 60.59 | -2.296 | 72.00 | 1.073 | 0.391 | 46.3 | -0.044 | True | True | True |
| no-edge-fit | 4.5 | 59.92 | -2.354 | 74.59 | 1.132 | 0.440 | 46.9 | -0.018 | False | False | False |
| no-edge-fit | 4.9 | 59.56 | -2.385 | 76.63 | 1.181 | 0.479 | 47.5 | 0.002 | False | False | False |
| no-edge-fit | 5.0 | 59.49 | -2.391 | 77.14 | 1.193 | 0.489 | 47.6 | 0.006 | False | False | False |
| no-edge-fit | 5.5 | 59.31 | -2.406 | 79.64 | 1.256 | 0.535 | 48.5 | 0.024 | False | False | False |
| no-edge-fit | 6.0 | 59.38 | -2.400 | 82.09 | 1.325 | 0.572 | 49.7 | 0.030 | False | False | False |
| no-edge-fit | 7.0 | 60.26 | -2.325 | 86.87 | 1.484 | 0.578 | 53.2 | -0.036 | True | True | True |
| no-edge-fit | 8.0 | 62.13 | -2.164 | 91.46 | 1.694 | 0.330 | 58.5 | -0.288 | True | True | True |
| no-edge-search | 4.0 | 60.01 | -2.346 | 71.66 | 1.060 | 0.348 | 45.6 | -0.077 | True | True | True |
| no-edge-search | 4.5 | 60.01 | -2.346 | 74.38 | 1.140 | 0.403 | 47.7 | -0.065 | True | True | True |
| no-edge-search | 4.9 | 60.59 | -2.296 | 76.71 | 1.217 | 0.457 | 50.0 | -0.059 | True | True | True |
| no-edge-search | 5.0 | 60.30 | -2.321 | 77.09 | 1.223 | 0.453 | 49.8 | -0.061 | True | True | True |
| no-edge-search | 5.5 | 61.47 | -2.221 | 80.20 | 1.331 | 0.526 | 52.9 | -0.056 | True | True | True |
| no-edge-search | 6.0 | 61.18 | -2.246 | 82.53 | 1.391 | 0.540 | 53.4 | -0.059 | True | True | True |
| no-edge-search | 7.0 | 60.30 | -2.321 | 86.80 | 1.487 | 0.557 | 53.4 | -0.056 | True | True | True |
| no-edge-search | 8.0 | 60.01 | -2.346 | 91.07 | 1.593 | 0.580 | 54.0 | -0.055 | True | True | True |

The runtime column is the existing geometric evaluator combined with actuator limits, not all live feed gates. The rim proxy is an additional diagnostic using a flat horizontal rim plane and a conservative rectangular depth; actual hexagonal rim and net collisions are not simulated.

The 5% trim comparison and the release-state grid sweep are in far-shot-comparison.json. Counts are deterministic model samples, not measured scoring rates. The grid spans 4–8 m in 0.05 m increments, radial velocity -2.5 to +2.5 m/s in 0.5 m/s increments, and sideways velocity -1, 0, +1 m/s. Acceleration, tilt, feedback error, air drag and ball variation are not swept.

No robot configuration was changed. Reproduce with `node --import tsx src/cli/compare-far-shots.ts` from tools/shot-calculator.
