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
        // Current regression at 8 m/s, before the auto-aim-only output trim.
        assertEquals(-75.153653, result.getAsDouble(), EPSILON);
    }
}
