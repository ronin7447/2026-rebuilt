# Release manifest, operator check, and incident record

## Candidate identity

- Release name / date / reviewer:
- Git revision and dirty status:
- Hardware/flywheel revision:
- Calibration artifact ID/hash and source dataset/holdout report:
- Controller/current/profile configuration and criteria revision:
- Firmware and vendor versions:
- Qualified distance/motion/target/voltage domain:
- Active default flags and disabled experiments:
- Matching-hardware rollback revision/configuration/calibration:
- Raw evidence manifest location:

## Release gates

| Gate | Implemented | Software verified | Practice verified | Field qualified | Evidence or pending reason |
|---|---|---|---|---|---|
| References/control ownership | | | | | |
| Logs/preflight | | | | | |
| Turret/hood/shooter control | | | | | |
| New flywheel calibration | | | | | |
| Stationary Hub/passback | | | | | |
| Moving shooting | | | | | |
| Sustained feed/recovery | | | | | |
| Combined-load electrical | | | | | |
| Occlusion/reacquisition/modes | | | | | |
| Runtime timing | | | | | |
| Autonomous/alliance geometry | | | | | |
| Three match-duration trials | | | | | |

No untested row silently becomes green. A restricted stationary/practice envelope can be an explicit choice; document what is disabled and why.

## Match-day operator check

- [ ] Battery/power and mechanism inspection complete.
- [ ] Robot revision/configuration/calibration match the reviewed release.
- [ ] References and essential configuration health valid.
- [ ] Logging and battery/match labeling verified.
- [ ] Autonomous/start pose and alliance behavior checked.
- [ ] Driver knows qualified range, degraded-pose indication and fallback/disable controls.
- [ ] No unreviewed test mode or calibration setting active.

## Incident

- Match/run and exact time interval:
- Driver request / expected behavior / actual behavior:
- Mode, active target/profile, battery, calibration and reference states:
- DS brownout/disconnect indication, controller reset or mechanism symptom:
- Log/video URI and checksums:
- Recovery action / override used / lost time:
- Suspected cause (hypothesis, not conclusion):
- Next packet/agent request:

Requalify affected stages after wheel/compression/gearing/reference/camera/firmware or significant control/model changes. Never call an old-wheel coefficient set a proven new-wheel rollback.
