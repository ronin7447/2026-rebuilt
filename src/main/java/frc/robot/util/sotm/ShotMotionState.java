package frc.robot.util.sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record ShotMotionState(
    Pose2d robotPose,
    ChassisSpeeds robotRelativeSpeeds,
    ChassisSpeeds fieldRelativeSpeeds,
    double fieldAccelXMps2,
    double fieldAccelYMps2,
    double angularAccelRadPerSec2,
    Rotation3d robotOrientation
) {
    /** Ignore pitch/roll for shooting without changing measured translation, acceleration or yaw. */
    public ShotMotionState withTiltCompensation(boolean enabled) {
        if (enabled) return this;
        return new ShotMotionState(robotPose, robotRelativeSpeeds, fieldRelativeSpeeds,
            fieldAccelXMps2, fieldAccelYMps2, angularAccelRadPerSec2,
            new Rotation3d(0.0, 0.0, robotOrientation.getZ()));
    }
}
