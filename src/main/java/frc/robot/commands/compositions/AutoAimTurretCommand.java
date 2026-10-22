package frc.robot.commands.compositions;

import java.util.Optional;
import java.util.OptionalDouble;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.util.sotm.ShotTargetConstraints;

public class AutoAimTurretCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final ShooterCalibration shooterCalibration = new LinearShooterCalibration();

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
        var motionEstimate = drivetrain.getShotMotionEstimate();
        double motionAgeSec = Utils.getCurrentTimeSeconds() - motionEstimate.timestampSec();
        if (Constants.ShotMap.USE_ROBUST_SOTM && !Constants.ShotMap.USE_LEGACY_SOLVER_ROLLBACK
            && (motionEstimate.motion() == null || !motionEstimate.derivativeValid()
                || !Double.isFinite(motionAgeSec) || motionAgeSec < 0.0
                || motionAgeSec > Constants.ShotMap.MAX_MOTION_AGE_SEC)) {
            turret.holdCurrentPosition();
            turret.invalidateAimState("invalid-shot-motion");
            return;
        }
        var drivetrainState = drivetrain.getStateCopy();
        Pose2d robotPose = motionEstimate.motion() != null
            ? motionEstimate.motion().robotPose() : drivetrainState.Pose;
        ChassisSpeeds robotRelativeSpeeds = motionEstimate.motion() != null
            ? motionEstimate.motion().robotRelativeSpeeds() : drivetrainState.Speeds;
        Translation2d pivotFieldPosition = AimMath.computeTurretPivotFieldPosition(
            robotPose,
            Constants.Turret.ROBOT_TO_TURRET_PIVOT
        );
        boolean poseFresh = vision.getVisionAgeSec() <= Constants.Turret.MAX_VISION_AGE_FOR_SHOT_SEC;
        boolean visionHealthy = vision.isVisionHealthy();
        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

        boolean autoAimAllowedMode = DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled();

        if (!turret.getPositionMode()) {
            turret.invalidateAimState("turret-position-mode-disabled");
            return;
        }
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

        if (Constants.ShotMap.USE_ROBUST_SOTM
            && !Constants.ShotMap.USE_LEGACY_SOLVER_ROLLBACK) {
            ShotMotionState motionState = motionEstimate.motion();
            ShotTargetConstraints targetConstraints = targetSelection.targetMode() == AutoAimTargeting.TargetMode.HUB
                ? ShotTargetConstraints.hub(selectedTargetPoint3d.getZ())
                : ShotTargetConstraints.passback(selectedTargetPoint3d.getZ());
            robustShotSolution = RobustShotModel.solve(new RobustShotRequest(
                motionState,
                Constants.Turret.ROBOT_TO_TURRET_PIVOT,
                Constants.Shooter.EXIT_HEIGHT_METERS,
                selectedTargetPoint3d,
                Rotation2d.kZero,
                Constants.Shooter.MIN_HOOD_POSITION_ROT,
                Constants.Shooter.MAX_HOOD_POSITION_ROT,
                Constants.Shooter.MAX_EXIT_SPEED_MPS,
                Constants.ShotMap.BASE_RELEASE_LATENCY_SEC + motionAgeSec,
                targetConstraints
            ));

            if (robustShotSolution.isPresent() && robustShotSolution.get().quality().insideTolerance()) {
                RobustShotSolution solution = robustShotSolution.get();
                commandedTurretAngle = solution.desiredTurretYaw();
                desiredFieldYaw = solution.desiredFieldYaw();
                mappedHoodPosition = OptionalDouble.of(solution.hoodPositionRot());
                mappedRollerRps = OptionalDouble.of(solution.rollerRps());
                mappedHoodPosition.ifPresent(shooter::setTargetPositionRotation);
                mappedRollerRps.ifPresent(shooter::setRollerTargetVelocity);
                shotModelName = targetSelection.targetMode() == AutoAimTargeting.TargetMode.HUB
                    ? "RobustShotMap"
                    : "ReferencePassback";
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
            && (!Constants.ShotMap.USE_ROBUST_SOTM
                || Constants.ShotMap.USE_LEGACY_SOLVER_ROLLBACK)) {
            movingShotSolution = ShooterKinematics.solveMovingShot(request, selectionPreference);
            shotModelName = Constants.ShotMap.USE_LEGACY_SOLVER_ROLLBACK ? "LegacyRollback" : "Legacy";
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
                1.0,
                Double.NaN
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
            quality.rotationScale(),
            robustShotSolution.get().solverDurationSec()
        );
    }

    @Override
    public void end(boolean interrupted) {
        turret.holdCurrentPosition();
        turret.invalidateAimState(interrupted ? "command-interrupted" : "command-ended");
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
