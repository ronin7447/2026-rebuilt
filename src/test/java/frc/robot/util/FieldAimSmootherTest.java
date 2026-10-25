package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class FieldAimSmootherTest {
    @Test
    void attenuatesAlternatingNoiseAndConvergesToPersistentCorrection() {
        var filter = new FieldAimSmoother();
        filter.calculate(Rotation2d.kZero, 0, true, .06, .1);
        for (int i = 1; i <= 50; i++) {
            double output = filter.calculate(Rotation2d.fromDegrees(i % 2 == 0 ? -1 : 1), i * .02,
                true, .06, .1).getDegrees();
            assertTrue(Math.abs(output) < .3);
        }
        for (int i = 51; i <= 100; i++) filter.calculate(Rotation2d.fromDegrees(5), i * .02, true, .06, .1);
        assertEquals(5, filter.calculate(Rotation2d.fromDegrees(5), 2.02, true, .06, .1).getDegrees(), .001);
    }

    @Test
    void wrapsShortestPathAndBypassesOrResetsWithoutStaleHistory() {
        var filter = new FieldAimSmoother();
        filter.calculate(Rotation2d.fromDegrees(179), 0, true, .06, .1);
        var result = filter.calculate(Rotation2d.fromDegrees(-179), .02, true, .06, .1);
        assertTrue(Math.abs(result.minus(Rotation2d.fromDegrees(179)).getDegrees()) < 1);
        assertEquals(30, filter.calculate(Rotation2d.fromDegrees(30), .04, false, .06, .1).getDegrees(), 1e-9);
        assertEquals(60, filter.calculate(Rotation2d.fromDegrees(60), .06, true, 0, .1).getDegrees(), 1e-9);
        assertEquals(90, filter.calculate(Rotation2d.fromDegrees(90), 1, true, .06, .1).getDegrees(), 1e-9);
        filter.reset();
        assertEquals(0, filter.calculate(Rotation2d.kZero, 1.02, true, .06, .1).getDegrees(), 1e-9);
    }

    @Test
    void fieldTargetFilterDoesNotDelayChassisCounterrotation() {
        var filter = new FieldAimSmoother();
        var field = Rotation2d.fromDegrees(70);
        filter.calculate(field, 0, true, .06, .1);
        for (int i = 1; i <= 10; i++) {
            var heading = Rotation2d.fromDegrees(i * 3);
            var smoothed = filter.calculate(field, i * .02, true, .06, .1);
            var solverGoal = field.minus(heading);
            var commanded = solverGoal.plus(smoothed.minus(field));
            assertEquals(70 - i * 3, commanded.getDegrees(), 1e-9);
        }
    }
}
