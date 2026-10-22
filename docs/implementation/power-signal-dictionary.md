# Power and health signal dictionary

The single `PowerHealthRecorder` starts WPILib DataLog once, retains Phoenix `.hoot` logging, and writes cached signals without scheduler-thread waits. WPILib selects an attached USB log location when supported and otherwise uses its internal fallback. `PowerLogger/Healthy`, `PowerLogger/Status`, and `PowerLogger/PDHAvailable` expose failure without blocking control.

These indicators are not proof of disk persistence or live PDH communication: the current adapters report availability after initialization, and `LastWriteSec` is the sampling timestamp. Retrieve a real log and verify changing values/identity before relying on it. General NetworkTables logging is disabled. Full feed blockers and shot-motion derivatives are explicitly recorded; turret profile/trial metrics still need explicit recording through [packet 02](../field-readiness/ai-handoffs/02-session-recording-and-preflight.md).

All `Power/` records use FPGA-relative seconds at recorder write time. Every motor measurement also has `/Valid` and `/AgeSec`; age is the Phoenix source timestamp latency, not the recorder loop age.

| Record family | Unit/rate | Notes |
|---|---|---|
| `Power/PDHVoltageV`, `Power/PDHTotalCurrentA` | V/A, 50 Hz requested | REV PDH CAN ID is provisional `1`; set a negative ID only during setup to use WPILib-supported discovery |
| `Power/PDHChannel/<n>/CurrentA` | A, 50 Hz requested | Numeric labels are intentional until wiring is physically verified |
| `Power/Motor/<name>/SupplyCurrentA` | A, 50 Hz requested | Battery-side motor-controller current |
| `Power/Motor/<name>/StatorCurrentA` | A, 50 Hz requested | Torque-producing winding current; never sum as battery load |
| `Power/Motor/<name>/MotorVoltageV` | V, 50 Hz requested | Replaces prior 1000 Hz requests |
| `Power/Motor/<name>/PositionRot`, `VelocityRps` | rotations/rps, control-owned rate | The recorder does not lower swerve odometry or mechanism feedback rates |
| `Power/TurretMotion/PositionRot`, `VelocityRps` | rotations/rps, 100 Hz requested/write | One recorder-owned notifier; source validity/age included |
| `Power/Motor/<name>/TemperatureC`, `FaultField`, `ResetDuringEnable`, `BridgeBrownout` | C/bitfield/bool, 10 Hz | Slow health and sticky controller-reset/brownout data |
| `Power/RoboRIO*`, `Power/*CAN*` | mixed, 10 Hz | Brownout, rail, CPU, and bus status |
| `Power/Enabled`, `Mode`, `MatchTimeSec`, `ShootRequested`, `FeedPermitted`, `ManualOverride`, `FeedTransitionTimestampSec` | state, 50 Hz | Synchronized match/feed context |
| `Power/Intake*`, `Power/AcceptedVisionAgeSec` | state/sec, 50 Hz | Intake command/homing state and degraded-pose context |
| `Power/FeedBlockers` | string, 50 Hz | Last feed-readiness explanation, including motion limits; command end clears the active explanation |
| `Power/ShotMotionDerivativeValid`, `Power/ShotMotionAgeSec` | bool/sec, 50 Hz | Derivative validity and age at recorder capture; age uses the Phoenix clock on both sides, then is logged alongside the FPGA write time |
| `Power/ShotRawAccelerationMps2`, `Power/ShotRawAngularAccelerationRadPerSec2` | m/s², rad/s², 50 Hz | Unfiltered vector/angular magnitudes; can exceed the predictor clamp, intentionally |
| `Power/ShotFieldAccelXMps2`, `Power/ShotFieldAccelYMps2`, `Power/ShotAngularAccelRadPerSec2` | m/s², rad/s², 50 Hz | Filtered derivatives used to predict release; unavailable motion is NaN, not zero |
| `Power/ConfigurationId`, `BatteryLabel`, `Config/*` | identity, 10 Hz | Requested and applied configuration remain distinct |

Registered motor names are `intake-extension-master`, `intake-extension-follower`, `intake-roller`, `turret`, `shooter-hood`, `shooter-roller`, `spindexer`, `final-feed`, and each front/back left/right swerve drive/steer motor.

Current-limit values are unchanged by this implementation. Requested values remain in `Constants`, subsystem configurations, and `TunerConstants`; an `Applied=true` result confirms the configuration API succeeded, not an independent hardware readback. Physically verify firmware and readback before qualification.

Use AdvantageScope to open `.wpilog`/`.hoot`, select synchronized fields, and export CSV for `tools/power-analysis`. The analyzer does not claim binary-log support.
