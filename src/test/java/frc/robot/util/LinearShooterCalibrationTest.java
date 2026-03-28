package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;

import org.junit.jupiter.api.Test;

class LinearShooterCalibrationTest {
    private static final double EPSILON = 1e-9;

    private final LinearShooterCalibration calibration = new LinearShooterCalibration();

    @Test
    void convertsLaunchPitchToHoodEncoderPosition() {
        OptionalDouble result = calibration.hoodPositionForLaunchAngleDeg(45.0);

        assertTrue(result.isPresent());
        assertEquals((0.0857913 * 45.0) - 7.49442, result.getAsDouble(), EPSILON);
    }

    @Test
    void convertsExitSpeedToRollerRps() {
        OptionalDouble result = calibration.rollerRpsForExitSpeedMps(8.0);

        assertTrue(result.isPresent());
        assertEquals(-((8.01922 * 8.0) + 10.82394), result.getAsDouble(), EPSILON);
    }
}
