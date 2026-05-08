package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

class FieldTargetsTest {
    private static final double EPSILON = 1e-9;

    @Test
    void hubAimPointsUseActualHubCenters() {
        assertEquals(158.6 * 0.0254, Constants.FieldTargets.BLUE_HUB_AIM_POINT.getX(), EPSILON);
        assertEquals(317.7 * 0.0254 / 2.0, Constants.FieldTargets.BLUE_HUB_AIM_POINT.getY(), EPSILON);
        assertEquals(
            (651.2 - 158.6) * 0.0254,
            Constants.FieldTargets.RED_HUB_AIM_POINT.getX(),
            EPSILON
        );
        assertEquals(317.7 * 0.0254 / 2.0, Constants.FieldTargets.RED_HUB_AIM_POINT.getY(), EPSILON);
    }

    @Test
    void blueAlliancePrefersUpperBackTargetWhenRobotYIsCloserToUpper() {
        Translation2d selected = Constants.FieldTargets.getPreferredBackTargetForAlliance(
            DriverStation.Alliance.Blue,
            5.8
        );

        assertEquals(Constants.FieldTargets.BLUE_BACK_TARGET_UPPER.getX(), selected.getX(), EPSILON);
        assertEquals(6.00, selected.getY(), EPSILON);
    }

    @Test
    void redAlliancePrefersLowerBackTargetWhenRobotYIsCloserToLower() {
        Translation2d selected = Constants.FieldTargets.getPreferredBackTargetForAlliance(
            DriverStation.Alliance.Red,
            2.7
        );

        assertEquals(Constants.FieldTargets.RED_BACK_TARGET_LOWER.getX(), selected.getX(), EPSILON);
        assertEquals(2.50, selected.getY(), EPSILON);
    }

    @Test
    void exactTiePrefersLowerYBackTarget() {
        Translation2d selected = Constants.FieldTargets.getPreferredBackTargetForAlliance(
            DriverStation.Alliance.Blue,
            4.25
        );

        assertEquals(Constants.FieldTargets.BLUE_BACK_TARGET_LOWER.getX(), selected.getX(), EPSILON);
        assertEquals(2.50, selected.getY(), EPSILON);
    }

    @Test
    void opponentPassBackTargetPrefersUpperWhenRobotYIsCloserToUpper() {
        Translation2d selected = Constants.FieldTargets.getPreferredOpponentPassBackTarget(6.8);

        assertEquals(8.295, selected.getX(), EPSILON);
        assertEquals(7.00, selected.getY(), EPSILON);
    }

    @Test
    void opponentPassBackTargetTiePrefersLowerY() {
        Translation2d selected = Constants.FieldTargets.getPreferredOpponentPassBackTarget(4.0);

        assertEquals(8.295, selected.getX(), EPSILON);
        assertEquals(1.00, selected.getY(), EPSILON);
    }
}
