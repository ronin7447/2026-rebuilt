# Launch-speed calibration trials

On the `launch-speed-calibration` branch, Xbox controller port 0 uses **A to start**
and **B to stop** a manual shooter trial in enabled teleop. A is latched: release
the button after pressing it; the trial continues until stopped. Back also stops.
B stops the shooter, outtake, spindexer, and intake and disables turret/hood output.
Disabling the robot or interrupting the trial stops the shooter/feed and disables
turret/hood output. A new press of A is required to restart.

## Set a trial speed

All calibration inputs use **positive Kraken motor rotations per second (RPS)**,
matching `Constants.Shooter` and the current launch-speed calibration. For example,
1500 RPM is **25 RPS**, and 2000 RPM is **33.3333 RPS**. This is motor speed, not
Fuel exit speed in m/s. The command applies `ROLLER_DIRECTION_SIGN` internally
(currently negative). Input must be finite, greater than zero, and no more than
`Constants.Shooter.MAX_ROLLER_RPS` (currently 100 RPS).

Choose either source on SmartDashboard:

- Set `LaunchSpeedCalibration/UseDashboardRPS` to **true** and edit
  `LaunchSpeedCalibration/RequestedRPS` (initial default: 25).
- Set `LaunchSpeedCalibration/UseDashboardRPS` to **false** to use
  `Constants.LaunchSpeedCalibration.ROLLER_SPEED_RPS`. Edit that constant and
  rebuild/deploy to change the code value. `USE_DASHBOARD_RPS` sets the initial
  dashboard source default; an existing dashboard value takes precedence.

Press A to sample the selected source and speed. Changes during a run do not
change the trial: press B, release it, then press A to apply a new value. Pressing
A again while already running does not restart or change the trial. Invalid
inputs stop the mechanisms and report `invalid-speed-or-hood-target`.

## Measured readiness

During each trial the turret holds **0 degrees relative to the robot**, and the
hood holds **70 degrees launch angle** using the current hood calibration
(-1.489029 motor rotations). The calibration command owns the turret, shooter,
outtake, and spindexer so auto-aim cannot overwrite the trial.

Both outtake and spindexer run only while the shooter is at its measured target
speed and the hood/turret are at their targets. Existing feedback freshness and
position/speed tolerances apply (shooter error must be less than 1 RPS). Feeding
pauses as soon as readiness is lost and resumes when all checks pass. There is
**no fixed spin-up, feed staging, or recovery delay** in this A/B command.

Normal right-trigger shooting is unbound on this lab branch. Start and turret
POV controls are ignored during a trial or while B/Back is held. Drivetrain and
intake controls otherwise retain their usual bindings; B has priority while held.
The calibration branch does not change the existing launch-speed fit coefficients.

## Observe and record

SmartDashboard publishes under `LaunchSpeedCalibration/`:

| Key | Meaning |
| --- | --- |
| `ActiveSource` | Source sampled when A was pressed |
| `ActiveRPS` | Positive motor-speed magnitude sampled for the trial |
| `ActualRPS` | Measured motor-speed magnitude during the trial |
| `HoodTargetRot` | Hood motor-position target from the existing calibration |
| `Running` | A valid calibration trial is active |
| `Feeding` | Both feed motors are commanded on |
| `Status` | Feeding, the current readiness blocker, stopped, or invalid input |

For the lab, record measured motor RPS alongside each trial's time in air,
horizontal distance, and launch angle. Physically confirm the hood and turret
references before collecting measurements; the code uses the existing encoder
zeroing and hood-angle mapping.

Run `./gradlew test` for automated input, lifecycle, readiness, and scheduler tests.
On the robot, verify A spins up with feed off, feed follows measured readiness,
B stops during both spin-up and feeding, and disable/re-enable does not restart
a trial. Software tests do not establish physical angle or speed accuracy.
