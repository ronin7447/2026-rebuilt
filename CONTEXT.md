# Team 7447 robot

The robot collects, transports, and launches Fuel while moving around the REBUILT field.

## Language

**Fuel**:
The game piece collected and launched by the robot.
_Avoid_: Ball when naming game-specific behavior.

**Hub shot**:
A launch intended to score Fuel in the alliance Hub.

**Passback**:
A launch intended to transport Fuel to a selected field location rather than score directly in a Hub.

**Turret**:
The mechanism that changes the shooter's horizontal aiming direction relative to the robot.

**Hood**:
The mechanism that adjusts the launch elevation of Fuel.

**Spindexer**:
The rotating Fuel transport mechanism upstream of the final shooter feed.
_Avoid_: Shooter when referring to this transport mechanism.

**Feed**:
The transport of Fuel into the shooter for launch.

**Release state**:
The robot and shooter motion at the predicted instant Fuel leaves the shooter.

**Release latency**:
The interval between evaluating a shot and the predicted departure of Fuel from the shooter.

**Radial velocity**:
The shooter's velocity component along the line toward the selected target; positive motion approaches the target.

**Tangential velocity**:
The shooter's velocity component perpendicular to the line toward the selected target.

**Trajectory feasibility**:
Whether a predicted launch satisfies the geometric and physical constraints for its target.
_Avoid_: Accuracy or scoring probability.

**Shot readiness**:
Whether a feasible shot and the current mechanism state permit automatic feeding.
_Avoid_: Shooter speed ready when describing readiness of the entire shot.

**Robustness score**:
A heuristic measure used to compare candidate shots; it is not an observed probability of scoring.

**Degraded pose**:
A position estimate that continues without recent accepted vision measurements and may accumulate error.

**Supply current**:
The current drawn from the robot's power supply by a motor controller.

**Stator current**:
The motor winding current associated with torque production.

**Brownout**:
A low-voltage condition that invokes robot protection; a disconnect or mechanism stall alone does not establish a brownout.

**Manual feed override**:
A deliberate driver request to bypass automatic shot readiness checks while retaining actuator protections.
