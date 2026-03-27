package frc.robot.util;

import java.util.OptionalDouble;

public interface ShooterCalibration {
    OptionalDouble hoodPositionForLaunchAngleDeg(double launchAngleDeg);

    OptionalDouble rollerRpsForExitSpeedMps(double exitSpeedMps);
}
