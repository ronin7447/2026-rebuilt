# Far-Hub map trials

Set `Constants.ShotMap.FAR_HUB_SHOT_MAP_TRIAL`, rebuild, and deploy:

- `1`: generated 63-degree map (selected first).
- `2`: generated 61-degree map (flatter comparison).
- `0`: original map, with the existing runtime flattening controlled by `FAR_HUB_FLAT_SHOT_ENABLED`. Set that boolean false too to restore the original unflattened map.

Both trials retain the original map through 4 m, smoothly blend over 4–5.5 m, and use their fitted far map beyond 5.5 m. Distances are predicted release-to-Hub distances, not Tower field coordinates. The runtime flattening override is skipped for profiles 1 and 2. Its angle/distance settings only affect profile 0.

The existing global 1.05 RPS trim remains; the additional far-distance 8% boost is removed. Manual calibration and passback do not use these trial maps. Robot runtime still checks the compensated trajectory and mechanism limits before normal feed readiness.

Try profile 1 stationary at the same far location first, then profile 2 for a comparison. Record requested/actual hood and RPS and where misses land. These maps use the existing gravity-only model and launch calibration; they are experiments, not evidence that the physical short-shot problem is solved. Global output trim is applied after model validation and is not reflected in modeled flight time or lead.

Regenerate from this directory with `npm run generate:far-trials`. This writes `GeneratedFarShotTrials.java` and `far-shot-trial-report.json` without overwriting the original map. The report checks an intermediate grid at 0.05 m / 0.1 m/s spacing before global output trim; acceptance counts include unreachable combinations. It does not establish field accuracy. Runtime checks also account for sideways motion and tilt. The browser calculator's existing default preview remains the original map; the trial report is the comparison artifact.

All seven sampled stationary distances (4, 4.5, 5, 5.5, 6, 7, 8 m) pass each trial's nominal geometry check. Max training RPS residual: 0.688 RPS for 63 degrees, 0.517 RPS for 61 degrees. The report contains coefficients and detailed setpoints.
