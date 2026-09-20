# Setup, hardware identity, and preflight record

Copy this blank form per hardware revision/test session. Blank means unknown, not zero or passed. See [human guide](../HUMAN-GUIDE.md).

## Identity and evidence

- Date / session ID / recorder:
- Repository revision / branch / dirty changes:
- Robot hardware revision / flywheel revision:
- Configuration ID / calibration ID and hash / criteria revision:
- Shared evidence location (portable team path/URI, no credentials):
- Raw log/video filenames and checksums:
- Battery ID / inspection and charge condition:
- Facility / test lead / driver:

## Test-space measurements

| Measurement | Value | Unit | Method / uncertainty |
|---|---:|---|---|
| Clear firing distance from release point | | m | |
| Minimum usable ceiling height over arc | | m | |
| Release height | | m | |
| Target/landing height | | m | |
| Camera scale plane and marker spacing | | m | |
| Frame timing/PTS availability | | s | |

Target type: net / flat plane / partial goal / realistic Hub opening / field. Circle one. Record opening/lip geometry only when physically present and measured. Intended near/mid/far distances and allowable motion:

## Hardware and wiring

| Item | Actual model / ID / gearing | Reference procedure / legal travel | Verified by / evidence |
|---|---|---|---|
| Turret motor and encoder | | | |
| Hood motor and encoder | | | |
| Shooter wheel/motor/gearing | | | |
| Intake extension master/follower | | | |
| Intake roller | | | |
| Spindexer / final feed | | | |
| Drive/steer motors | | | |
| REV PDH / roboRIO | | | |
| Cameras / transforms / firmware | | | |

Record wheel diameter/material, compression/gap, replaced parts and any backspin behavior observed. Do not identify a motor solely from a code comment.

PDH channel map:

| PDH channel | Physical load | Motor CAN ID and bus if applicable | Confirmed independently |
|---|---|---|---|
| | | | |

## Before first Fuel launch

- [ ] Mechanical freedom and travel checked; controls/disable procedure understood.
- [ ] Packet 01 implemented and unloaded direction/reverse/cancel tests passed.
- [ ] Actual reference procedure completed successfully; reset recovery understood.
- [ ] Homing success, interruption and timeout behavior verified without destructive obstruction.
- [ ] Firmware/configuration results and essential health reviewed.
- [ ] PDH identity and retrieved log values plausible; logfile opened successfully.
- [ ] Session/battery/hardware identity present in the retrieved log.
- [ ] Proposed launch arc and target containment fit measured space.
- [ ] Test conditions/criteria approved before collecting results.

Stop/failure reason and next action:
