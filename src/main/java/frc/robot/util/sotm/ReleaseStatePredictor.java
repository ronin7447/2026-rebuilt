package frc.robot.util.sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ReleaseStatePredictor {
    private ReleaseStatePredictor() {}

    public static PredictedReleaseState predict(ShotMotionState motionState, double releaseLatencySec) {
        double latencySec = Math.max(0.0, releaseLatencySec);
        Pose2d pose = motionState.robotPose();
        ChassisSpeeds fieldSpeeds = motionState.fieldRelativeSpeeds();

        double predictedX = pose.getX()
            + (fieldSpeeds.vxMetersPerSecond * latencySec)
            + (0.5 * motionState.fieldAccelXMps2() * latencySec * latencySec);
        double predictedY = pose.getY()
            + (fieldSpeeds.vyMetersPerSecond * latencySec)
            + (0.5 * motionState.fieldAccelYMps2() * latencySec * latencySec);
        double predictedOmega = fieldSpeeds.omegaRadiansPerSecond
            + (motionState.angularAccelRadPerSec2() * latencySec);
        double predictedHeadingRad = pose.getRotation().getRadians()
            + (fieldSpeeds.omegaRadiansPerSecond * latencySec)
            + (0.5 * motionState.angularAccelRadPerSec2() * latencySec * latencySec);

        Rotation2d predictedHeading = new Rotation2d(predictedHeadingRad);
        ChassisSpeeds predictedFieldSpeeds = new ChassisSpeeds(
            fieldSpeeds.vxMetersPerSecond + (motionState.fieldAccelXMps2() * latencySec),
            fieldSpeeds.vyMetersPerSecond + (motionState.fieldAccelYMps2() * latencySec),
            predictedOmega
        );
        ChassisSpeeds predictedRobotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            predictedFieldSpeeds.vxMetersPerSecond,
            predictedFieldSpeeds.vyMetersPerSecond,
            predictedFieldSpeeds.omegaRadiansPerSecond,
            predictedHeading
        );
        Rotation3d orientation = motionState.robotOrientation();
        Rotation3d predictedOrientation = new Rotation3d(
            orientation.getX(),
            orientation.getY(),
            predictedHeadingRad
        );

        return new PredictedReleaseState(
            new Pose2d(predictedX, predictedY, predictedHeading),
            predictedRobotSpeeds,
            predictedFieldSpeeds,
            predictedOrientation,
            latencySec
        );
    }
}
