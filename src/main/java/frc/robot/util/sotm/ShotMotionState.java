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
) {}
