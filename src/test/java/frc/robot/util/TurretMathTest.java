package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.constants.Constants;

class TurretMathTest {
    private static final double EPSILON = 1e-9;

    @Test
    void wrapsFullRotationTargetsIntoLegalRange() {
        double currentTurretRotations = 0.0;
        double legalLow = 0.0;
        double legalHigh = 1.0;

        assertEquals(
            20.0 / 360.0,
            TurretMath.getNearestEquivalentLegalTurretRotation(20.0 / 360.0, currentTurretRotations, legalLow, legalHigh),
            EPSILON
        );
        assertEquals(
            20.0 / 360.0,
            TurretMath.getNearestEquivalentLegalTurretRotation(380.0 / 360.0, currentTurretRotations, legalLow, legalHigh),
            EPSILON
        );
        assertEquals(
            340.0 / 360.0,
            TurretMath.getNearestEquivalentLegalTurretRotation(-20.0 / 360.0, currentTurretRotations, legalLow, legalHigh),
            EPSILON
        );
        assertEquals(
            0.0,
            TurretMath.getNearestEquivalentLegalTurretRotation(720.0 / 360.0, currentTurretRotations, legalLow, legalHigh),
            EPSILON
        );
        assertEquals(
            180.0 / 360.0,
            TurretMath.getNearestEquivalentLegalTurretRotation(540.0 / 360.0, currentTurretRotations, legalLow, legalHigh),
            EPSILON
        );
    }

    @Test
    void prefersClosestLegalBoundaryEquivalent() {
        double currentTurretRotations = 350.0 / 360.0;

        assertEquals(
            1.0,
            TurretMath.getNearestEquivalentLegalTurretRotation(720.0 / 360.0, currentTurretRotations, 0.0, 1.0),
            EPSILON
        );
    }

    @Test
    void convertsTurretAnglesToMotorRotationsForCurrentRatio() {
        double ratio = Constants.Turret.MOTOR_ROTATIONS_PER_TURRET_ROTATION;

        assertEquals(
            Constants.Turret.SOFT_LIMIT_LOW * ratio,
            Constants.Turret.TALON_SOFT_LIMIT_LOW,
            EPSILON
        );
        assertEquals(
            Constants.Turret.SOFT_LIMIT_HIGH * ratio,
            Constants.Turret.TALON_SOFT_LIMIT_HIGH,
            EPSILON
        );
        assertEquals((20.0 / 360.0) * ratio, TurretMath.convertTurretDegreesToMotorRotations(20.0, ratio), EPSILON);
        assertEquals((340.0 / 360.0) * ratio, TurretMath.convertTurretDegreesToMotorRotations(340.0, ratio), EPSILON);
        assertEquals(0.5 * ratio, TurretMath.convertTurretDegreesToMotorRotations(180.0, ratio), EPSILON);
        assertEquals(ratio, TurretMath.convertTurretDegreesToMotorRotations(360.0, ratio), EPSILON);
        assertEquals(360.0 / ratio, TurretMath.convertMotorRotationsToTurretDegrees(1.0, ratio), EPSILON);
    }
}
