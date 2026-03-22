package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class AimMath {
    private AimMath() {}

    public static Translation2d computeTurretPivotFieldPosition(
        Pose2d robotPose,
        Translation2d robotToTurretPivot
    ) {
        return robotPose.getTranslation().plus(robotToTurretPivot.rotateBy(robotPose.getRotation()));
    }

    public static Rotation2d computeDesiredTurretAngle(
        Pose2d robotPose,
        Translation2d robotToTurretPivot,
        Rotation2d turretZeroOffset,
        Translation2d hubFieldPoint
    ) {
        Translation2d turretPivotFieldPosition = computeTurretPivotFieldPosition(robotPose, robotToTurretPivot);
        Translation2d toHub = hubFieldPoint.minus(turretPivotFieldPosition);
        Rotation2d desiredFieldYaw = toHub.getAngle();

        return desiredFieldYaw
            .minus(robotPose.getRotation())
            .minus(turretZeroOffset);
    }
}
