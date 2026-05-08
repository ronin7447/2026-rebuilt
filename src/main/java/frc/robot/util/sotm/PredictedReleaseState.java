package frc.robot.util.sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record PredictedReleaseState(
    Pose2d robotPose,
    ChassisSpeeds robotRelativeSpeeds,
    ChassisSpeeds fieldRelativeSpeeds,
    Rotation3d robotOrientation,
    double releaseLatencySec
) {}
