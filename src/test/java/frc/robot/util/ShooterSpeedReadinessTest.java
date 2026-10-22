package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import frc.robot.constants.Constants;

class ShooterSpeedReadinessTest {
    private ShooterSpeedReadiness readiness() {
        return new ShooterSpeedReadiness(Constants.Shooter.ROLLER_START_TOLERANCE_RPS,
            Constants.Shooter.ROLLER_START_TOLERANCE_SCALE,
            Constants.Shooter.ROLLER_RUNNING_TOLERANCE_RPS,
            Constants.Shooter.ROLLER_RUNNING_TOLERANCE_SCALE);
    }

    @Test
    void twentyFiveRpsRequestAcceptsFloatingTwentySixPointFiveImmediately() {
        var gate = readiness();
        assertTrue(gate.update(26.5, 25, true));
        for (double actual : new double[] {25.9, 27.5, 28.5, 26.5, 24.0, 21.0}) {
            assertTrue(gate.update(actual, 25, true), "ordinary variation " + actual);
        }
        assertFalse(gate.update(19.9, 25, true));
        assertFalse(gate.update(29, 25, true)); // Wider running band cannot start a cold gate.
        assertTrue(gate.update(26.5, 25, true));
    }

    @Test
    void negativeShooterDirectionUsesSameBandsButOppositeDirectionCannotPass() {
        var gate = readiness();
        assertTrue(gate.update(-26.5, -25, true));
        assertTrue(gate.update(-29, -25, true));
        assertFalse(gate.update(26.5, -25, true));
        assertFalse(gate.update(0, -25, true));
    }

    @Test
    void fractionalBandScalesWithHigherTargets() {
        var gate = readiness();
        assertFalse(gate.update(56, 50, true));
        assertTrue(gate.update(55, 50, true));
        assertTrue(gate.update(57.5, 50, true));
        assertFalse(gate.update(57.6, 50, true));
    }

    @Test
    void disabledOrInvalidFeedbackDropsLatchAndZeroTargetsAreNotShotReady() {
        var gate = readiness();
        assertTrue(gate.update(26.5, 25, true));
        assertFalse(gate.update(26.5, 25, false));
        assertFalse(gate.update(29, 25, true));
        assertFalse(gate.update(Double.NaN, 25, true));
        assertFalse(gate.update(25, Double.POSITIVE_INFINITY, true));
        assertFalse(gate.update(0, 0, true));
        assertFalse(gate.update(1, 0, true));
    }

    @Test
    void targetJumpAndExplicitResetRequireReentryIntoStartBand() {
        var gate = readiness();
        assertTrue(gate.update(25, 25, true));
        assertFalse(gate.update(31, 35, true));
        assertTrue(gate.update(35, 35, true));
        gate.reset();
        assertFalse(gate.update(39, 35, true));
    }
}
