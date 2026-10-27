package frc.robot.util.sotm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class ShooterFrameTransform {
    private static final double EPSILON = 1e-9;

    private ShooterFrameTransform() {}

    public static Translation3d fieldVectorToRobotFrame(
        Translation3d fieldVector,
        Rotation3d robotOrientation
    ) {
        return fieldVector.rotateBy(robotOrientation.unaryMinus());
    }

    public static Rotation2d turretYawForLaunchVector(
        Translation3d fieldLaunchVector,
        Rotation3d robotOrientation,
        Rotation2d turretZeroOffset
    ) {
        Translation3d robotVector = fieldVectorToRobotFrame(fieldLaunchVector, robotOrientation);
        double horizontalNorm = Math.hypot(robotVector.getX(), robotVector.getY());
        if (horizontalNorm <= EPSILON) {
            return Rotation2d.kZero;
        }
        return new Rotation2d(robotVector.getX(), robotVector.getY()).minus(turretZeroOffset);
    }

    public static Rotation2d localLaunchPitchForLaunchVector(
        Translation3d fieldLaunchVector,
        Rotation3d robotOrientation
    ) {
        Translation3d robotVector = fieldVectorToRobotFrame(fieldLaunchVector, robotOrientation);
        double horizontalNorm = Math.hypot(robotVector.getX(), robotVector.getY());
        return Rotation2d.fromRadians(Math.atan2(robotVector.getZ(), horizontalNorm));
    }
}
