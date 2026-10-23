package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;

class TurretGoalStabilizerTest {
    private TurretGoalStabilizer stabilizer() {
        return new TurretGoalStabilizer(0.15, 0.10, -381.15 / 360.0, 171.0 / 360.0);
    }

    @Test
    void suppressesSmallJitterButAccumulatesSlowRealMotion() {
        var filter = stabilizer();
        assertEquals(10, filter.calculate(Rotation2d.fromDegrees(10), 0, 0, true).getDegrees(), 1e-9);
        assertEquals(10, filter.calculate(Rotation2d.fromDegrees(10.10), 0, 0.02, true).getDegrees(), 1e-9);
        assertEquals(10, filter.calculate(Rotation2d.fromDegrees(9.90), 0, 0.04, true).getDegrees(), 1e-9);
        assertEquals(10.20, filter.calculate(Rotation2d.fromDegrees(10.20), 0, 0.06, true).getDegrees(), 1e-9);
    }

    @Test
    void continuousCounterrotationHasNoTimeFilterLag() {
        var filter = stabilizer();
        for (int i = 0; i < 20; i++) {
            double goalDeg = -90.0 * i * 0.02;
            assertEquals(goalDeg, filter.calculate(Rotation2d.fromDegrees(goalDeg), 0, i * 0.02, true)
                .getDegrees(), 1e-9);
        }
    }

    @Test
    void preservesLegalUnwrappedBranchAcrossAngleWrapAndForcesLegalRewind() {
        var filter = stabilizer();
        assertEquals(-181, filter.calculate(Rotation2d.fromDegrees(179), -180.0 / 360, 0, true)
            .getDegrees(), 1e-9);
        assertEquals(-179, filter.calculate(Rotation2d.fromDegrees(-179), -180.0 / 360, 0.02, true)
            .getDegrees(), 1e-9);
        filter.reset();
        assertEquals(170, filter.calculate(Rotation2d.fromDegrees(170), 170.0 / 360, 0, true).getDegrees(), 1e-9);
        assertEquals(-188, filter.calculate(Rotation2d.fromDegrees(172), 170.0 / 360, 0.02, true).getDegrees(), 1e-9);
    }

    @Test
    void keepsChosenBranchWhenMeasuredPositionWouldPreferAnotherEquivalent() {
        var filter = stabilizer();
        assertEquals(-370, filter.calculate(Rotation2d.fromDegrees(-10), -360.0 / 360, 0, true)
            .getDegrees(), 1e-9);
        assertEquals(-369, filter.calculate(Rotation2d.fromDegrees(-9), -180.0 / 360, 0.02, true)
            .getDegrees(), 1e-9);
    }

    @Test
    void disableResetClockReversalAndSampleGapDiscardHistory() {
        var filter = stabilizer();
        filter.calculate(Rotation2d.fromDegrees(10), 0, 0, true);
        assertEquals(10.05, filter.calculate(Rotation2d.fromDegrees(10.05), 0, 0.02, false).getDegrees(), 1e-9);
        assertEquals(10.10, filter.calculate(Rotation2d.fromDegrees(10.10), 0, 0.04, true).getDegrees(), 1e-9);
        assertEquals(10.15, filter.calculate(Rotation2d.fromDegrees(10.15), 0, 0.20, true).getDegrees(), 1e-9);
        assertEquals(10.20, filter.calculate(Rotation2d.fromDegrees(10.20), 0, 0.10, true).getDegrees(), 1e-9);
        filter.reset();
        assertEquals(10.25, filter.calculate(Rotation2d.fromDegrees(10.25), 0, 0.12, true).getDegrees(), 1e-9);
    }
}
