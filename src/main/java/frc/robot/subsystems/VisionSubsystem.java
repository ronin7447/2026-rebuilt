package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class VisionSubsystem extends SubsystemBase {
    private static final double MAX_SINGLE_TAG_AMBIGUITY = Constants.Vision.MAX_SINGLE_TAG_AMBIGUITY;
    private static final double MAX_POSE_Z_METERS = Constants.Vision.MAX_POSE_Z_METERS;
    private static final double MAX_CORRECTION_METERS = Constants.Vision.MAX_CORRECTION_METERS;

    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout fieldLayout;
    private final List<CameraContext> cameras;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        fieldLayout = AprilTagFieldLayout.loadField(Constants.Vision.FIELD_LAYOUT);
        cameras = List.of(
            new CameraContext(Constants.Vision.APRILTAG_CAM1, Constants.Vision.ROBOT_TO_CAM1),
            new CameraContext(Constants.Vision.APRILTAG_CAM2, Constants.Vision.ROBOT_TO_CAM2)
        );
    }

    @Override
    public void periodic() {
        for (CameraContext camera : cameras) {
            for (PhotonPipelineResult result : camera.camera.getAllUnreadResults()) {
                processResult(camera, result);
            }
        }
    }

    private void processResult(CameraContext camera, PhotonPipelineResult result) {
        if (!result.hasTargets()) {
            return;
        }

        Optional<EstimatedRobotPose> estimate = camera.estimator.estimateCoprocMultiTagPose(result);
        boolean isMultiTagEstimate = estimate.isPresent();
        if (estimate.isEmpty()) {
            estimate = camera.estimator.estimateLowestAmbiguityPose(result);
        }
        if (estimate.isEmpty()) {
            return;
        }

        EstimatedRobotPose estimatedRobotPose = estimate.get();
        Pose2d visionPose = estimatedRobotPose.estimatedPose.toPose2d();
        Pose2d referencePose = drivetrain.samplePoseAt(estimatedRobotPose.timestampSeconds)
            .orElse(drivetrain.getState().Pose);

        if (!isValidMeasurement(result, estimatedRobotPose, visionPose, referencePose, isMultiTagEstimate)) {
            return;
        }

        drivetrain.addVisionMeasurement(
            visionPose,
            estimatedRobotPose.timestampSeconds,
            calculateVisionStdDevs(estimatedRobotPose, isMultiTagEstimate)
        );
    }

    private boolean isValidMeasurement(
        PhotonPipelineResult result,
        EstimatedRobotPose estimatedRobotPose,
        Pose2d visionPose,
        Pose2d referencePose,
        boolean isMultiTagEstimate
    ) {
        if (!isPoseOnField(visionPose)) {
            return false;
        }
        if (Math.abs(estimatedRobotPose.estimatedPose.getZ()) > MAX_POSE_Z_METERS) {
            return false;
        }
        if (!isMultiTagEstimate && hasHighSingleTagAmbiguity(result)) {
            return false;
        }
        return visionPose.getTranslation().getDistance(referencePose.getTranslation()) <= MAX_CORRECTION_METERS;
    }

    private boolean isPoseOnField(Pose2d pose) {
        return pose.getX() >= 0.0
            && pose.getX() <= fieldLayout.getFieldLength()
            && pose.getY() >= 0.0
            && pose.getY() <= fieldLayout.getFieldWidth();
    }

    private boolean hasHighSingleTagAmbiguity(PhotonPipelineResult result) {
        if (result.getTargets().size() != 1) {
            return false;
        }

        PhotonTrackedTarget target = result.getBestTarget();
        double ambiguity = target.getPoseAmbiguity();
        return ambiguity >= 0.0 && ambiguity > MAX_SINGLE_TAG_AMBIGUITY;
    }

    private Matrix<N3, N1> calculateVisionStdDevs(
        EstimatedRobotPose estimatedRobotPose,
        boolean isMultiTagEstimate
    ) {
        double avgTargetDistanceMeters = averageTargetDistanceMeters(estimatedRobotPose.targetsUsed);
        double distanceScale = 1.0 + (avgTargetDistanceMeters * avgTargetDistanceMeters / 25.0);

        double xyStdDev = isMultiTagEstimate ? 0.30 : 0.70;
        double thetaStdDev = isMultiTagEstimate ? 0.80 : 3.00;

        xyStdDev *= distanceScale;
        thetaStdDev *= distanceScale;

        if (estimatedRobotPose.targetsUsed.size() == 1) {
            thetaStdDev = 999.0;
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    private double averageTargetDistanceMeters(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) {
            return Double.POSITIVE_INFINITY;
        }

        double totalDistance = 0.0;
        for (PhotonTrackedTarget target : targets) {
            totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        return totalDistance / targets.size();
    }

    private static class CameraContext {
        final PhotonCamera camera;
        final PhotonPoseEstimator estimator;

        CameraContext(String cameraName, edu.wpi.first.math.geometry.Transform3d robotToCamera) {
            camera = new PhotonCamera(cameraName);
            estimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(Constants.Vision.FIELD_LAYOUT),
                robotToCamera
            );
        }
    }
}
