package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

class FieldTargetsTest {
    private static final double EPSILON = 1e-9;

    @Test
    void blueAlliancePrefersUpperBackTargetWhenRobotYIsCloserToUpper() {
        Translation2d selected = Constants.FieldTargets.getPreferredBackTargetForAlliance(
            DriverStation.Alliance.Blue,
            5.8
        );

        assertEquals(2.26, selected.getX(), EPSILON);
        assertEquals(6.00, selected.getY(), EPSILON);
    }

    @Test
    void redAlliancePrefersLowerBackTargetWhenRobotYIsCloserToLower() {
        Translation2d selected = Constants.FieldTargets.getPreferredBackTargetForAlliance(
            DriverStation.Alliance.Red,
            2.7
        );

        assertEquals(14.5, selected.getX(), EPSILON);
        assertEquals(2.50, selected.getY(), EPSILON);
    }

    @Test
    void exactTiePrefersLowerYBackTarget() {
        Translation2d selected = Constants.FieldTargets.getPreferredBackTargetForAlliance(
            DriverStation.Alliance.Blue,
            4.25
        );

        assertEquals(2.26, selected.getX(), EPSILON);
        assertEquals(2.50, selected.getY(), EPSILON);
    }
}
