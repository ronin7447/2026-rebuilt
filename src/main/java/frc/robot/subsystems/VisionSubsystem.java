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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout fieldLayout;
    private final List<VisionCamera> cameras;

    private double lastAcceptedVisionTimestampSec = -1.0;
    private int lastVisionTagCount = 0;
    private Matrix<N3, N1> lastVisionStdDevs = VecBuilder.fill(
        Double.POSITIVE_INFINITY,
        Double.POSITIVE_INFINITY,
        Double.POSITIVE_INFINITY
    );
    private boolean visionHealthy = false;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        fieldLayout = Constants.Vision.FIELD_LAYOUT_OBJECT;
        cameras = List.of(
            new VisionCamera(Constants.Vision.APRILTAG_CAM1, Constants.Vision.ROBOT_TO_CAM1, fieldLayout),
            new VisionCamera(Constants.Vision.APRILTAG_CAM2, Constants.Vision.ROBOT_TO_CAM2, fieldLayout)
        );
    }

    @Override
    public void periodic() {
        for (VisionCamera camera : cameras) {
            for (PhotonPipelineResult result : camera.camera.getAllUnreadResults()) {
                processResult(camera, result);
            }
        }

        visionHealthy = Constants.Vision.FORCE_ACCEPT_VISION_MEASUREMENTS
            || getVisionAgeSec() <= Constants.Vision.VISION_HEALTHY_AGE_SEC;
        publishTelemetry();
    }

    public boolean isVisionHealthy() {
        return visionHealthy;
    }

    public double getVisionAgeSec() {
        if (lastAcceptedVisionTimestampSec < 0.0) {
            return Double.POSITIVE_INFINITY;
        }
        return Timer.getFPGATimestamp() - lastAcceptedVisionTimestampSec;
    }

    public double getLastAcceptedVisionTimestampSec() {
        return lastAcceptedVisionTimestampSec;
    }

    public int getLastVisionTagCount() {
        return lastVisionTagCount;
    }

    public Matrix<N3, N1> getLastVisionStdDevs() {
        return lastVisionStdDevs;
    }

    private void processResult(VisionCamera camera, PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> estimateOpt = estimateRobotPose(camera.estimator, result);
        if (estimateOpt.isEmpty()) {
            camera.rejectedCount++;
            return;
        }

        EstimatedRobotPose estimatedRobotPose = estimateOpt.get();
        Pose2d visionPose = estimatedRobotPose.estimatedPose.toPose2d();
        Pose2d referencePose = drivetrain.samplePoseAt(estimatedRobotPose.timestampSeconds)
            .orElse(drivetrain.getState().Pose);
        double measurementAgeSec = Timer.getFPGATimestamp() - estimatedRobotPose.timestampSeconds;
        double averageDistanceMeters = averageTargetDistanceMeters(estimatedRobotPose.targetsUsed);
        Matrix<N3, N1> stdDevs = calculateVisionStdDevs(estimatedRobotPose.targetsUsed.size(), averageDistanceMeters);

        if (!isValidMeasurement(result, estimatedRobotPose, visionPose, referencePose, measurementAgeSec, averageDistanceMeters)) {
            camera.rejectedCount++;
            return;
        }

        // CommandSwerveDrivetrain already converts FPGA timestamps into CTRE's current-time epoch.
        drivetrain.addVisionMeasurement(visionPose, estimatedRobotPose.timestampSeconds, stdDevs);

        lastAcceptedVisionTimestampSec = Timer.getFPGATimestamp();
        lastVisionTagCount = estimatedRobotPose.targetsUsed.size();
        lastVisionStdDevs = stdDevs;
        camera.acceptedCount++;
    }

    private Optional<EstimatedRobotPose> estimateRobotPose(
        PhotonPoseEstimator estimator,
        PhotonPipelineResult result
    ) {
        Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
        if (estimate.isPresent()) {
            return estimate;
        }
        return estimator.estimateLowestAmbiguityPose(result);
    }

    private boolean isValidMeasurement(
        PhotonPipelineResult result,
        EstimatedRobotPose estimatedRobotPose,
        Pose2d visionPose,
        Pose2d referencePose,
        double measurementAgeSec,
        double averageDistanceMeters
    ) {
        if (!result.hasTargets()) {
            return false;
        }
        if (Constants.Vision.FORCE_ACCEPT_VISION_MEASUREMENTS) {
            return true;
        }
        if (measurementAgeSec > Constants.Vision.MAX_MEASUREMENT_AGE_SEC) {
            return false;
        }
        if (!isPoseOnField(visionPose)) {
            return false;
        }
        if (!isRotationReasonable(estimatedRobotPose.estimatedPose.getRotation())) {
            return false;
        }
        if (Math.abs(estimatedRobotPose.estimatedPose.getZ()) > Constants.Vision.MAX_POSE_Z_METERS) {
            return false;
        }
        if (visionPose.getTranslation().getDistance(referencePose.getTranslation()) > Constants.Vision.MAX_CORRECTION_METERS) {
            return false;
        }
        if (result.getTargets().size() == 1 && hasHighSingleTagAmbiguity(result)) {
            return false;
        }
        return result.getTargets().size() > 1 || averageDistanceMeters <= Constants.Vision.SINGLE_TAG_MAX_DISTANCE_METERS;
    }

    private boolean isPoseOnField(Pose2d pose) {
        return pose.getX() >= 0.0
            && pose.getX() <= fieldLayout.getFieldLength()
            && pose.getY() >= 0.0
            && pose.getY() <= fieldLayout.getFieldWidth();
    }

    private boolean isRotationReasonable(Rotation3d rotation) {
        return Math.abs(Units.radiansToDegrees(rotation.getX())) <= Constants.Vision.MAX_POSE_ROLL_PITCH_DEG
            && Math.abs(Units.radiansToDegrees(rotation.getY())) <= Constants.Vision.MAX_POSE_ROLL_PITCH_DEG;
    }

    private boolean hasHighSingleTagAmbiguity(PhotonPipelineResult result) {
        PhotonTrackedTarget target = result.getBestTarget();
        double ambiguity = target.getPoseAmbiguity();
        return ambiguity >= 0.0 && ambiguity > Constants.Vision.MAX_SINGLE_TAG_AMBIGUITY;
    }

    private Matrix<N3, N1> calculateVisionStdDevs(int tagCount, double averageDistanceMeters) {
        double xyStdDev = tagCount >= 2
            ? Constants.Vision.MULTI_TAG_BASE_XY_STD_DEV_METERS
            : Constants.Vision.SINGLE_TAG_BASE_XY_STD_DEV_METERS;
        double thetaStdDev = tagCount >= 2
            ? Constants.Vision.MULTI_TAG_BASE_THETA_STD_DEV_RAD
            : Constants.Vision.SINGLE_TAG_BASE_THETA_STD_DEV_RAD;

        double distanceScale = 1.0 + (averageDistanceMeters * averageDistanceMeters * Constants.Vision.DISTANCE_STD_DEV_SCALE);
        return VecBuilder.fill(xyStdDev * distanceScale, xyStdDev * distanceScale, thetaStdDev * distanceScale);
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

    private void publishTelemetry() {
        SmartDashboard.putBoolean("Vision/ForceAcceptMeasurements", Constants.Vision.FORCE_ACCEPT_VISION_MEASUREMENTS);
        SmartDashboard.putBoolean("Vision/Healthy", visionHealthy);
        SmartDashboard.putNumber("Vision/AgeSec", getVisionAgeSec());
        SmartDashboard.putNumber("Vision/LastAcceptedTsSec", lastAcceptedVisionTimestampSec);
        SmartDashboard.putNumber("Vision/LastTagCount", lastVisionTagCount);
        SmartDashboard.putNumber("Vision/LastStdDevXY", lastVisionStdDevs.get(0, 0));
        SmartDashboard.putNumber("Vision/LastStdDevThetaDeg", Units.radiansToDegrees(lastVisionStdDevs.get(2, 0)));

        for (VisionCamera camera : cameras) {
            SmartDashboard.putNumber("Vision/" + camera.name + "/AcceptedCount", camera.acceptedCount);
            SmartDashboard.putNumber("Vision/" + camera.name + "/RejectedCount", camera.rejectedCount);
        }
    }

    private static class VisionCamera {
        final String name;
        final PhotonCamera camera;
        final PhotonPoseEstimator estimator;
        int acceptedCount = 0;
        int rejectedCount = 0;

        VisionCamera(String cameraName, edu.wpi.first.math.geometry.Transform3d robotToCamera, AprilTagFieldLayout layout) {
            name = cameraName;
            camera = new PhotonCamera(cameraName);
            estimator = new PhotonPoseEstimator(layout, robotToCamera);
        }
    }
}
