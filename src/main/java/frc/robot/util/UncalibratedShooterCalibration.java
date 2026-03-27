package frc.robot.util;

import java.util.OptionalDouble;

public class UncalibratedShooterCalibration implements ShooterCalibration {
    @Override
    public OptionalDouble hoodPositionForLaunchAngleDeg(double launchAngleDeg) {
        return OptionalDouble.empty();
    }

    @Override
    public OptionalDouble rollerRpsForExitSpeedMps(double exitSpeedMps) {
        return OptionalDouble.empty();
    }
}
