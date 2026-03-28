package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

class AutoAimTargetingTest {
    private static final double EPSILON = 1e-9;

    @Test
    void ownAllianceZoneUsesHubTargetMode() {
        AutoAimTargeting.TargetSelection selection = AutoAimTargeting.selectTarget(
            DriverStation.Alliance.Blue,
            new Translation2d(1.0, 4.0),
            4.0
        );

        assertEquals(AutoAimTargeting.TargetMode.HUB, selection.targetMode());
        assertTrue(selection.inAllianceZone());
        assertFalse(selection.inNeutralZone());
        assertEquals(Constants.FieldTargets.BLUE_HUB_AIM_POINT.getX(), selection.targetPoint().getX(), EPSILON);
        assertEquals(Constants.FieldTargets.BLUE_HUB_AIM_POINT.getY(), selection.targetPoint().getY(), EPSILON);
        assertEquals(Constants.FieldTargets.HUB_TARGET_HEIGHT_METERS, selection.targetPoint3d().getZ(), EPSILON);
    }

    @Test
    void neutralZoneUsesPreferredBackTargetMode() {
        AutoAimTargeting.TargetSelection selection = AutoAimTargeting.selectTarget(
            DriverStation.Alliance.Red,
            new Translation2d(8.0, 5.6),
            5.6
        );

        assertEquals(AutoAimTargeting.TargetMode.NEUTRAL_BACK, selection.targetMode());
        assertFalse(selection.inAllianceZone());
        assertTrue(selection.inNeutralZone());
        assertEquals(14.5, selection.targetPoint().getX(), EPSILON);
        assertEquals(6.00, selection.targetPoint().getY(), EPSILON);
        assertEquals(0.2, selection.targetPoint3d().getZ(), EPSILON);
    }

    @Test
    void opponentAllianceZoneUsesHoldMode() {
        AutoAimTargeting.TargetSelection selection = AutoAimTargeting.selectTarget(
            DriverStation.Alliance.Blue,
            new Translation2d(15.5, 4.0),
            4.0
        );

        assertEquals(AutoAimTargeting.TargetMode.HOLD, selection.targetMode());
        assertFalse(selection.inAllianceZone());
        assertFalse(selection.inNeutralZone());
        assertEquals(0.0, selection.targetPoint().getX(), EPSILON);
        assertEquals(0.0, selection.targetPoint().getY(), EPSILON);
    }
}
