package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import frc.robot.constants.Constants;

class FarHubRpsTrimTest {
    @Test
    void measuredPointGetsSmallCorrectionAndCloseShotsStayUnchanged() {
        assertEquals(1, FarHubRpsTrim.scale(4.9, true, 4.9, 5.5, 1.012));
        assertEquals(1.006, FarHubRpsTrim.scale(5.2, true, 4.9, 5.5, 1.012), 1e-12);
        assertEquals(-87.9934, -86.95 * FarHubRpsTrim.scale(5.6633, true, 4.9, 5.5, 1.012), 1e-9);
        assertEquals(1.012, FarHubRpsTrim.scale(8, true, 4.9, 5.5, 1.012));
    }

    @Test
    void disabledPassbackAndInvalidSettingsBypassCorrection() {
        assertEquals(1, FarHubRpsTrim.scale(6, false, 4.9, 5.5, 1.012));
        assertEquals(1, FarHubRpsTrim.scale(Double.NaN, true, 4.9, 5.5, 1.012));
        assertEquals(1, FarHubRpsTrim.scale(6, true, 5.5, 4.9, 1.012));
    }

    @Test
    void moreSpeedAtFixedPitchMovesDescendingRimCrossingFarther() {
        double pitch = Math.toRadians((-2.089 - Constants.Shooter.HOOD_POSITION_PER_LAUNCH_DEG_INTERCEPT)
            / Constants.Shooter.HOOD_POSITION_PER_LAUNCH_DEG_SLOPE);
        double slope = Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_SLOPE;
        double intercept = Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_INTERCEPT;
        double originalSpeed = (86.95 - intercept) / slope;
        double correctedSpeed = (87.9934 - intercept) / slope;
        double rise = 1.83 + (5.91 * .0254 / 2) - Constants.Shooter.EXIT_HEIGHT_METERS;
        double originalVz = originalSpeed * Math.sin(pitch);
        double correctedVz = correctedSpeed * Math.sin(pitch);
        double originalTime = (originalVz + Math.sqrt(originalVz * originalVz - 2 * 9.81 * rise)) / 9.81;
        double correctedTime = (correctedVz + Math.sqrt(correctedVz * correctedVz - 2 * 9.81 * rise)) / 9.81;
        assertTrue(correctedSpeed * Math.cos(pitch) * correctedTime
            > originalSpeed * Math.cos(pitch) * originalTime);
    }
}
