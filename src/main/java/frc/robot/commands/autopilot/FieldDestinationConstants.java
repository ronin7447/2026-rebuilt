package frc.robot.commands.autopilot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FieldDestinationConstants {
    private FieldDestinationConstants() {}

    // Absolute blue-origin field coordinates; deliberately not alliance-flipped.
    public static final Pose2d BUMP_EXIT_TARGET =
        new Pose2d(1.3, 2.4, Rotation2d.fromDegrees(45.0));

    public static final Pose2d RED_LEFT =
        new Pose2d(14.8, 6.1, Rotation2d.fromDegrees(180));

    public static final Pose2d RED_RIGHT =
        new Pose2d(14.8, 1.9, Rotation2d.fromDegrees(180));

    public static final Pose2d BLUE_LEFT =
        new Pose2d(2.2, 6.1, Rotation2d.fromDegrees(0));

    public static final Pose2d BLUE_RIGHT =
        new Pose2d(2.2, 1.9, Rotation2d.fromDegrees(0));

    public static final Pose2d NEUTRAL_LEFT =
        new Pose2d(8.5, 6.0, Rotation2d.fromDegrees(0));

    public static final Pose2d NEUTRAL_RIGHT =
        new Pose2d(8.5, 2.0, Rotation2d.fromDegrees(0));
}
