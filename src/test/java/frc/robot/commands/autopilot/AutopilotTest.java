package frc.robot.commands.autopilot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class AutopilotTest {
    @Test
    void usesShortRangeAlignWhenInsideThreshold() {
        Pose2d currentPose = new Pose2d(3.0, 4.0, Rotation2d.kZero);
        Pose2d goal = new Pose2d(3.6, 4.0, Rotation2d.fromDegrees(45.0));

        assertTrue(Autopilot.shouldUseShortRangeAlign(goal, currentPose, 0.75));
    }

    @Test
    void usesPathfindWhenOutsideThreshold() {
        Pose2d currentPose = new Pose2d(3.0, 4.0, Rotation2d.kZero);
        Pose2d goal = new Pose2d(3.76, 4.0, Rotation2d.fromDegrees(45.0));

        assertFalse(Autopilot.shouldUseShortRangeAlign(goal, currentPose, 0.75));
    }

    @Test
    void translationDistanceOnlyDrivesShortRangeSelection() {
        Pose2d currentPose = new Pose2d(3.0, 4.0, Rotation2d.kZero);
        Pose2d goal = new Pose2d(3.0, 4.0, Rotation2d.k180deg);

        assertTrue(Autopilot.shouldUseShortRangeAlign(goal, currentPose, 0.75));
    }
}
