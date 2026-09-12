# Ultimate SOTM hardware run sheet

Status: hardware pending. Never replace unknowns with assumptions.

## Configuration identity

- Date/operator:
- Code revision:
- Configuration ID:
- roboRIO model/version: UNKNOWN
- REV PDH CAN ID: provisional software default `1`; VERIFY
- PDH channel-to-load labels: UNKNOWN; VERIFY EACH CHANNEL
- CAN buses and utilization:
- Motor firmware/readback:
- Log storage path (USB preferred, internal fallback):
- Battery label, resting voltage, inspection result:
- Hood tolerance: provisional `0.02 rot`; MEASURE
- Intake homing timeout: provisional `3.0 s`; BENCH VALIDATE
- Measured Fuel release latency: UNKNOWN

## Controlled order

1. Inspect battery, lugs, main breaker, PDH connections, and mechanism freedom.
2. Verify logs open and timestamps/signal ages are credible.
3. Run isolated drive, intake extension, intake roller, shooter spin-up, and staged feed trials.
4. Establish hood/roller stationary calibration and turret stationary step response.
5. Run moving Hub and passback trials for both alliances, then deliberate AprilTag occlusion trials.
6. Change one parameter at a time only after a recorded baseline. The first later candidate is gentler drivetrain acceleration under combined load.
7. Run three consecutive 150-second combined-load match trials on representative serviceable batteries.

## Per-run observations

| Run | Battery | Config | Minimum V | Brownouts/resets | Shooter recovery | Intake completion/failure | Attempts/makes | Notes / next experiment |
|---|---|---|---:|---|---:|---|---|---|
| | | | | | | | | |

Competition qualification requires all three match trials with no DS brownouts, controller resets, failed intake movements, or sustained loop overruns. Desktop and simulated results do not satisfy this gate.
