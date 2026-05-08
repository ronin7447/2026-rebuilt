package frc.robot.commands.compositions;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AimMath;
import frc.robot.util.AutoAimTargeting;
import frc.robot.util.LinearShooterCalibration;
import frc.robot.util.ShooterCalibration;
import frc.robot.util.ShooterKinematics;
import frc.robot.util.sotm.RobustShotModel;
import frc.robot.util.sotm.RobustShotRequest;
import frc.robot.util.sotm.RobustShotSolution;
import frc.robot.util.sotm.ShotMotionState;
import frc.robot.util.sotm.ShotQuality;

public class AutoAimTurretCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final ShooterCalibration shooterCalibration = new LinearShooterCalibration();
    private ChassisSpeeds lastFieldSpeeds = null;
    private double lastMotionSampleTimeSec = -1.0;
    private double filteredFieldAccelXMps2 = 0.0;
    private double filteredFieldAccelYMps2 = 0.0;
    private double filteredAngularAccelRadPerSec2 = 0.0;

    public AutoAimTurretCommand(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem vision,
        TurretSubsystem turret,
        ShooterSubsystem shooter
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.turret = turret;
        this.shooter = shooter;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        var drivetrainState = drivetrain.getState();
        Pose2d robotPose = drivetrainState.Pose;
        ChassisSpeeds robotRelativeSpeeds = drivetrainState.Speeds;
        Translation2d pivotFieldPosition = AimMath.computeTurretPivotFieldPosition(
            robotPose,
            Constants.Turret.ROBOT_TO_TURRET_PIVOT
        );
        boolean poseFresh = vision.getVisionAgeSec() <= Constants.Turret.MAX_VISION_AGE_FOR_SHOT_SEC;
        boolean visionHealthy = vision.isVisionHealthy();
        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

        boolean autoAimAllowedMode = DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled();

        if (!turret.getPositionMode()) return;
        if (!autoAimAllowedMode || turret.isAutoAimSuppressed()) {
            turret.holdCurrentPosition();
            publishAimState(
                AutoAimTargeting.TargetMode.HOLD.name(),
                allianceOpt.map(Enum::name).orElse("Unknown"),
                false,
                false,
                visionHealthy,
                poseFresh,
                false,
                pivotFieldPosition,
                Translation2d.kZero,
                turret.getTurretGoal(),
                Rotation2d.kZero,
                Optional.empty(),
                OptionalDouble.empty(),
                OptionalDouble.empty()
            );
            return;
        }

        if (allianceOpt.isEmpty()) {
            turret.park();
            publishAimState(
                AutoAimTargeting.TargetMode.PARK.name(),
                "Unknown",
                false,
                false,
                visionHealthy,
                poseFresh,
                false,
                pivotFieldPosition,
                Translation2d.kZero,
                Constants.Turret.PARK_ANGLE,
                Rotation2d.kZero,
                Optional.empty(),
                OptionalDouble.empty(),
                OptionalDouble.empty()
            );
            return;
        }

        DriverStation.Alliance alliance = allianceOpt.get();
        AutoAimTargeting.TargetSelection targetSelection = AutoAimTargeting.selectTarget(
            alliance,
            robotPose.getTranslation(),
            robotPose.getY()
        );
        Translation2d selectedTargetPoint = targetSelection.targetPoint();
        Translation3d selectedTargetPoint3d = targetSelection.targetPoint3d();
        Rotation2d stationaryDesiredTurretAngle = AimMath.computeDesiredTurretAngle(
            robotPose,
            Constants.Turret.ROBOT_TO_TURRET_PIVOT,
            Rotation2d.kZero,
            selectedTargetPoint
        );
        Rotation2d stationaryFieldYaw = AimMath.computeDesiredFieldYaw(pivotFieldPosition, selectedTargetPoint);
        boolean inAllianceZone = targetSelection.inAllianceZone();
        boolean inNeutralZone = targetSelection.inNeutralZone();

        if (targetSelection.targetMode() == AutoAimTargeting.TargetMode.HOLD) {
            turret.holdCurrentPosition();
            publishAimState(
                targetSelection.targetMode().name(),
                alliance.name(),
                false,
                false,
                visionHealthy,
                poseFresh,
                false,
                pivotFieldPosition,
                Translation2d.kZero,
                turret.getTurretGoal(),
                Rotation2d.kZero,
                Optional.empty(),
                OptionalDouble.empty(),
                OptionalDouble.empty()
            );
            return;
        }

        ShooterKinematics.MovingShotRequest request = new ShooterKinematics.MovingShotRequest(
            robotPose,
            robotRelativeSpeeds,
            Constants.Turret.ROBOT_TO_TURRET_PIVOT,
            Constants.Shooter.EXIT_HEIGHT_METERS,
            selectedTargetPoint3d,
            Rotation2d.kZero,
            shooterCalibration,
            Constants.Shooter.MIN_HOOD_POSITION_ROT,
            Constants.Shooter.MAX_HOOD_POSITION_ROT,
            Constants.Shooter.FAR_SHOT_DISTANCE_METERS,
            targetSelection.targetMode() == AutoAimTargeting.TargetMode.HUB
                ? Constants.Shooter.FAR_SHOT_MIN_HOOD_POSITION_ROT
                : Constants.Shooter.PASS_BACK_FAR_SHOT_MIN_HOOD_POSITION_ROT,
            Constants.Shooter.MAX_EXIT_SPEED_MPS
        );
        ShooterKinematics.SelectionPreference selectionPreference =
            targetSelection.targetMode() == AutoAimTargeting.TargetMode.HUB
                ? ShooterKinematics.SelectionPreference.MIN_EXIT_SPEED
                : ShooterKinematics.SelectionPreference.FLAT_FAST_SHOT;
        Optional<RobustShotSolution> robustShotSolution = Optional.empty();
        Optional<ShooterKinematics.MovingShotSolution> movingShotSolution = Optional.empty();
        String shotModelName = "Legacy";

        Rotation2d commandedTurretAngle = stationaryDesiredTurretAngle;
        Rotation2d desiredFieldYaw = stationaryFieldYaw;
        OptionalDouble mappedHoodPosition = OptionalDouble.empty();
        OptionalDouble mappedRollerRps = OptionalDouble.empty();

        if (targetSelection.targetMode() == AutoAimTargeting.TargetMode.HUB
            && Constants.ShotMap.USE_ROBUST_SOTM) {
            ShotMotionState motionState = sampleShotMotionState(robotPose, robotRelativeSpeeds);
            robustShotSolution = RobustShotModel.solve(new RobustShotRequest(
                motionState,
                Constants.Turret.ROBOT_TO_TURRET_PIVOT,
                Constants.Shooter.EXIT_HEIGHT_METERS,
                selectedTargetPoint3d,
                Rotation2d.kZero,
                Constants.Shooter.MIN_HOOD_POSITION_ROT,
                Constants.Shooter.MAX_HOOD_POSITION_ROT,
                Constants.Shooter.MAX_EXIT_SPEED_MPS,
                Constants.ShotMap.BASE_RELEASE_LATENCY_SEC
            ));

            if (robustShotSolution.isPresent() && robustShotSolution.get().quality().insideTolerance()) {
                RobustShotSolution solution = robustShotSolution.get();
                commandedTurretAngle = solution.desiredTurretYaw();
                desiredFieldYaw = solution.desiredFieldYaw();
                mappedHoodPosition = OptionalDouble.of(solution.hoodPositionRot());
                mappedRollerRps = OptionalDouble.of(solution.rollerRps());
                mappedHoodPosition.ifPresent(shooter::setTargetPositionRotation);
                mappedRollerRps.ifPresent(shooter::setRollerTargetVelocity);
                shotModelName = "RobustShotMap";
                movingShotSolution = Optional.of(new ShooterKinematics.MovingShotSolution(
                    solution.desiredFieldYaw(),
                    solution.desiredTurretYaw(),
                    solution.recommendedLaunchPitch(),
                    solution.requiredExitSpeedMps(),
                    solution.flightTimeSec(),
                    solution.leadYawDeg()
                ));
            }
        }

        if (movingShotSolution.isEmpty()
            && (targetSelection.targetMode() != AutoAimTargeting.TargetMode.HUB
                || !Constants.ShotMap.USE_ROBUST_SOTM
                || Constants.ShotMap.FALLBACK_TO_LEGACY_SOLVER)) {
            movingShotSolution = ShooterKinematics.solveMovingShot(request, selectionPreference);
            shotModelName = targetSelection.targetMode() == AutoAimTargeting.TargetMode.HUB
                && Constants.ShotMap.USE_ROBUST_SOTM
                    ? "LegacyFallback"
                    : "Legacy";
        }

        if (movingShotSolution.isPresent()) {
            ShooterKinematics.MovingShotSolution solution = movingShotSolution.get();
            commandedTurretAngle = solution.desiredTurretYaw();
            desiredFieldYaw = solution.desiredFieldYaw();
            if (mappedHoodPosition.isEmpty()) {
                mappedHoodPosition = shooterCalibration.hoodPositionForLaunchAngleDeg(
                    solution.recommendedLaunchPitch().getDegrees()
                );
            }
            if (mappedRollerRps.isEmpty()) {
                mappedRollerRps = shooterCalibration.rollerRpsForExitSpeedMps(solution.requiredExitSpeedMps());
            }

            mappedHoodPosition.ifPresent(shooter::setTargetPositionRotation);
            mappedRollerRps.ifPresent(shooter::setRollerTargetVelocity);
        }

        publishRobustShotTelemetry(shotModelName, robustShotSolution);
        turret.setTurretGoal(commandedTurretAngle);
        publishAimState(
            targetSelection.targetMode().name(),
            alliance.name(),
            inAllianceZone,
            inNeutralZone,
            visionHealthy,
            poseFresh,
            true,
            pivotFieldPosition,
            selectedTargetPoint,
            commandedTurretAngle,
            desiredFieldYaw,
            movingShotSolution,
            mappedHoodPosition,
            mappedRollerRps
        );
    }

    private ShotMotionState sampleShotMotionState(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond,
            robotRelativeSpeeds.omegaRadiansPerSecond,
            robotPose.getRotation()
        );
        double timestampSec = Timer.getFPGATimestamp();
        if (lastFieldSpeeds != null && lastMotionSampleTimeSec >= 0.0) {
            double dtSec = timestampSec - lastMotionSampleTimeSec;
            if (dtSec > 1e-6 && dtSec <= Constants.ShotMap.MAX_ACCEL_SAMPLE_DT_SEC) {
                filteredFieldAccelXMps2 = filteredAcceleration(
                    filteredFieldAccelXMps2,
                    (fieldSpeeds.vxMetersPerSecond - lastFieldSpeeds.vxMetersPerSecond) / dtSec,
                    Constants.ShotMap.MAX_FIELD_ACCEL_MPS2
                );
                filteredFieldAccelYMps2 = filteredAcceleration(
                    filteredFieldAccelYMps2,
                    (fieldSpeeds.vyMetersPerSecond - lastFieldSpeeds.vyMetersPerSecond) / dtSec,
                    Constants.ShotMap.MAX_FIELD_ACCEL_MPS2
                );
                filteredAngularAccelRadPerSec2 = filteredAcceleration(
                    filteredAngularAccelRadPerSec2,
                    (fieldSpeeds.omegaRadiansPerSecond - lastFieldSpeeds.omegaRadiansPerSecond) / dtSec,
                    Constants.ShotMap.MAX_ANGULAR_ACCEL_RAD_PER_SEC2
                );
            }
        }
        lastFieldSpeeds = fieldSpeeds;
        lastMotionSampleTimeSec = timestampSec;

        return new ShotMotionState(
            robotPose,
            robotRelativeSpeeds,
            fieldSpeeds,
            filteredFieldAccelXMps2,
            filteredFieldAccelYMps2,
            filteredAngularAccelRadPerSec2,
            drivetrain.getRobotOrientation3d()
        );
    }

    private double filteredAcceleration(double previousValue, double rawValue, double maxMagnitude) {
        double clampedRaw = Math.max(-maxMagnitude, Math.min(maxMagnitude, rawValue));
        return previousValue + (Constants.ShotMap.ACCEL_FILTER_ALPHA * (clampedRaw - previousValue));
    }

    private void publishRobustShotTelemetry(
        String shotModelName,
        Optional<RobustShotSolution> robustShotSolution
    ) {
        if (robustShotSolution.isEmpty()) {
            turret.updateRobustShotState(
                shotModelName,
                "not-running",
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                1.0,
                1.0
            );
            return;
        }

        ShotQuality quality = robustShotSolution.get().quality();
        turret.updateRobustShotState(
            shotModelName,
            quality.reason(),
            quality.distanceMeters(),
            quality.radialVelocityMps(),
            quality.tangentialVelocityMps(),
            quality.robustnessMargin(),
            quality.polynomialFitError(),
            quality.releaseLatencySec(),
            quality.translationScale(),
            quality.rotationScale()
        );
    }

    @Override
    public void end(boolean interrupted) {
        turret.holdCurrentPosition();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void publishAimState(
        String targetMode,
        String allianceName,
        boolean inAllianceZone,
        boolean inNeutralZone,
        boolean visionHealthy,
        boolean poseFresh,
        boolean autoAimActive,
        Translation2d pivotFieldPosition,
        Translation2d selectedTargetPoint,
        Rotation2d desiredTurretAngle,
        Rotation2d desiredFieldYaw,
        Optional<ShooterKinematics.MovingShotSolution> movingShotSolution,
        OptionalDouble mappedHoodPosition,
        OptionalDouble mappedRollerRps
    ) {
        boolean movingShotSolutionValid = movingShotSolution.isPresent();
        double recommendedLaunchPitchDeg = movingShotSolutionValid
            ? movingShotSolution.get().recommendedLaunchPitch().getDegrees()
            : Double.NaN;
        double requiredExitSpeedMps = movingShotSolutionValid
            ? movingShotSolution.get().requiredExitSpeedMps()
            : Double.NaN;
        double flightTimeSec = movingShotSolutionValid
            ? movingShotSolution.get().flightTimeSec()
            : Double.NaN;
        double leadYawDeg = movingShotSolutionValid
            ? movingShotSolution.get().leadYawDeg()
            : Double.NaN;

        turret.updateAimState(
            targetMode,
            allianceName,
            inAllianceZone,
            inNeutralZone,
            visionHealthy,
            poseFresh,
            autoAimActive,
            pivotFieldPosition,
            selectedTargetPoint,
            desiredTurretAngle,
            desiredFieldYaw,
            movingShotSolutionValid,
            recommendedLaunchPitchDeg,
            requiredExitSpeedMps,
            flightTimeSec,
            leadYawDeg,
            mappedHoodPosition.isPresent(),
            mappedHoodPosition.isPresent() ? mappedHoodPosition.getAsDouble() : Double.NaN,
            mappedRollerRps.isPresent(),
            mappedRollerRps.isPresent() ? mappedRollerRps.getAsDouble() : Double.NaN
        );
    }
}
