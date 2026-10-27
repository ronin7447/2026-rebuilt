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
import frc.robot.util.VisionTimestampPolicy;
import frc.robot.util.VisionQualityPolicy;

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

        for (VisionCamera cam: cameras) {

            cam.camera.setFPSLimit(20);
        }
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
        return Math.max(0.0, Timer.getFPGATimestamp() - lastAcceptedVisionTimestampSec);
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
        if (!result.hasTargets()) {
            return;
        }

        Optional<EstimatedRobotPose> estimateOpt = estimateRobotPose(camera.estimator, result);
        if (estimateOpt.isEmpty()) {
            camera.rejectedCount++;
            camera.lastDecision = "no-pose-estimate";
            return;
        }

        EstimatedRobotPose estimatedRobotPose = estimateOpt.get();
        Pose2d visionPose = estimatedRobotPose.estimatedPose.toPose2d();
        double averageDistanceMeters = averageTargetDistanceMeters(estimatedRobotPose.targetsUsed);
        int usedTags = estimatedRobotPose.targetsUsed.size();
        camera.lastDistance = averageDistanceMeters;
        camera.lastTagCount = usedTags;
        double ambiguity = usedTags == 1 ? estimatedRobotPose.targetsUsed.get(0).getPoseAmbiguity() : 0;
        String rejection = VisionQualityPolicy.rejectionReason(usedTags, averageDistanceMeters, ambiguity,
            Constants.Vision.MAX_SINGLE_TAG_DISTANCE_METERS, Constants.Vision.MAX_MULTI_TAG_DISTANCE_METERS,
            Constants.Vision.MAX_SINGLE_TAG_AMBIGUITY);
        if (!Constants.Vision.FORCE_ACCEPT_VISION_MEASUREMENTS && !rejection.isEmpty()) {
            camera.rejectedCount++;
            camera.lastDecision = rejection;
            return;
        }
        Matrix<N3, N1> stdDevs = calculateVisionStdDevs(estimatedRobotPose.targetsUsed.size(), averageDistanceMeters);

        if (!isValidMeasurement(estimatedRobotPose, visionPose)) {
            camera.rejectedCount++;
            camera.lastDecision = "pose-or-timestamp";
            return;
        }

        // CommandSwerveDrivetrain already converts FPGA timestamps into CTRE's current-time epoch.
        drivetrain.addVisionMeasurement(visionPose, estimatedRobotPose.timestampSeconds, stdDevs);

        // Preserve capture time (not processing time), and never move backward when cameras arrive out of order.
        lastAcceptedVisionTimestampSec = VisionTimestampPolicy.monotonicAcceptedTimestamp(
            lastAcceptedVisionTimestampSec,
            estimatedRobotPose.timestampSeconds
        );
        lastVisionTagCount = estimatedRobotPose.targetsUsed.size();
        lastVisionStdDevs = stdDevs;
        camera.acceptedCount++;
        camera.lastDecision = "accepted";
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
        EstimatedRobotPose estimatedRobotPose,
        Pose2d visionPose
    ) {
        if (!VisionTimestampPolicy.isValid(
            Timer.getFPGATimestamp(),
            estimatedRobotPose.timestampSeconds,
            0.05,
            Constants.Vision.FORCE_ACCEPT_VISION_MEASUREMENTS
                ? Double.POSITIVE_INFINITY
                : Constants.Vision.MAX_MEASUREMENT_AGE_SEC
        )) {
            return false;
        }
        if (Constants.Vision.FORCE_ACCEPT_VISION_MEASUREMENTS) {
            return true;
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
        return true;
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

    private Matrix<N3, N1> calculateVisionStdDevs(int tagCount, double averageDistanceMeters) {
        double xyStdDev = tagCount >= 2
            ? Constants.Vision.MULTI_TAG_BASE_XY_STD_DEV_METERS
            : Constants.Vision.SINGLE_TAG_BASE_XY_STD_DEV_METERS;
        double thetaStdDev = tagCount >= 2
            ? Constants.Vision.MULTI_TAG_BASE_THETA_STD_DEV_RAD
            : Constants.Vision.SINGLE_TAG_BASE_THETA_STD_DEV_RAD;

        double distanceScale = 1.0 + (averageDistanceMeters * averageDistanceMeters * Constants.Vision.DISTANCE_STD_DEV_SCALE);
        return VecBuilder.fill(xyStdDev * distanceScale, xyStdDev * distanceScale,
            VisionQualityPolicy.headingStdDev(Constants.Vision.USE_VISION_HEADING, thetaStdDev * distanceScale));
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
        SmartDashboard.putBoolean("Vision/UseVisionHeading", Constants.Vision.USE_VISION_HEADING);
        SmartDashboard.putBoolean("Vision/ForceAcceptMeasurements", Constants.Vision.FORCE_ACCEPT_VISION_MEASUREMENTS);
        SmartDashboard.putBoolean("Vision/Healthy", visionHealthy);
        SmartDashboard.putNumber("Vision/AgeSec", getVisionAgeSec());
        SmartDashboard.putNumber("Vision/LastAcceptedTsSec", lastAcceptedVisionTimestampSec);
        SmartDashboard.putNumber("Vision/LastTagCount", lastVisionTagCount);
        SmartDashboard.putNumber("Vision/LastStdDevXY", lastVisionStdDevs.get(0, 0));
        SmartDashboard.putNumber("Vision/LastStdDevThetaDeg", Units.radiansToDegrees(lastVisionStdDevs.get(2, 0)));

        for (VisionCamera camera : cameras) {
            SmartDashboard.putNumber("Vision/" + camera.name + "/AcceptedCount", camera.acceptedCount);
            SmartDashboard.putNumber("Vision/" + camera.name + "/LastDistanceMeters", camera.lastDistance);
            SmartDashboard.putNumber("Vision/" + camera.name + "/LastUsedTagCount", camera.lastTagCount);
            SmartDashboard.putString("Vision/" + camera.name + "/LastDecision", camera.lastDecision);
            SmartDashboard.putNumber("Vision/" + camera.name + "/RejectedCount", camera.rejectedCount);
        }
    }

    private static class VisionCamera {
        final String name;
        final PhotonCamera camera;
        final PhotonPoseEstimator estimator;
        double lastDistance = Double.NaN;
        int lastTagCount = 0;
        String lastDecision = "no-measurement";
        int acceptedCount = 0;
        int rejectedCount = 0;

        VisionCamera(String cameraName, edu.wpi.first.math.geometry.Transform3d robotToCamera, AprilTagFieldLayout layout) {
            name = cameraName;
            camera = new PhotonCamera(cameraName);
            estimator = new PhotonPoseEstimator(layout, robotToCamera);
        }
    }
}
