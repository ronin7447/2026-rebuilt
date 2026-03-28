package frc.robot.util;

import java.util.OptionalDouble;

import frc.robot.constants.Constants;

public class LinearShooterCalibration implements ShooterCalibration {
    @Override
    public OptionalDouble hoodPositionForLaunchAngleDeg(double launchAngleDeg) {
        return OptionalDouble.of(
            (Constants.Shooter.HOOD_POSITION_PER_LAUNCH_DEG_SLOPE * launchAngleDeg)
                + Constants.Shooter.HOOD_POSITION_PER_LAUNCH_DEG_INTERCEPT
        );
    }

    @Override
    public OptionalDouble rollerRpsForExitSpeedMps(double exitSpeedMps) {
        return OptionalDouble.of(
            (Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_SLOPE * exitSpeedMps)
                + Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_INTERCEPT
        );
    }
}
