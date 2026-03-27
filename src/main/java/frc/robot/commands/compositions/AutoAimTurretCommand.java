package frc.robot.commands.compositions;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AimMath;
import frc.robot.util.PolygonUtil;
import frc.robot.util.ShooterCalibration;
import frc.robot.util.ShooterKinematics;
import frc.robot.util.UncalibratedShooterCalibration;

public class AutoAimTurretCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final TurretSubsystem turret;
    private final ShooterCalibration shooterCalibration = new UncalibratedShooterCalibration();

    public AutoAimTurretCommand(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem vision,
        TurretSubsystem turret
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d pivotFieldPosition = AimMath.computeTurretPivotFieldPosition(
            robotPose,
            Constants.Turret.ROBOT_TO_TURRET_PIVOT
        );
        boolean poseFresh = vision.getVisionAgeSec() <= Constants.Turret.MAX_VISION_AGE_FOR_SHOT_SEC;
        boolean visionHealthy = vision.isVisionHealthy();
        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

        if (!DriverStation.isTeleopEnabled() || turret.isAutoAimSuppressed()) {
            turret.holdCurrentPosition();
            publishAimState(
                allianceOpt.map(Enum::name).orElse("Unknown"),
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
                "Unknown",
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
        Translation2d hubPoint = Constants.FieldTargets.getHubForAlliance(alliance);
        Translation3d hubTargetPoint = Constants.FieldTargets.getHubAimPoint3d(alliance);
        Rotation2d stationaryDesiredTurretAngle = AimMath.computeDesiredTurretAngle(
            robotPose,
            Constants.Turret.ROBOT_TO_TURRET_PIVOT,
            Constants.Turret.TURRET_ZERO_OFFSET,
            hubPoint
        );
        Rotation2d stationaryFieldYaw = AimMath.computeDesiredFieldYaw(pivotFieldPosition, hubPoint);
        boolean inAllianceZone = PolygonUtil.contains(
            Constants.FieldTargets.getAllianceZone(alliance),
            robotPose.getTranslation()
        );

        if (!inAllianceZone) {
            turret.holdCurrentPosition();
            publishAimState(
                alliance.name(),
                false,
                visionHealthy,
                poseFresh,
                false,
                pivotFieldPosition,
                hubPoint,
                stationaryDesiredTurretAngle,
                stationaryFieldYaw,
                Optional.empty(),
                OptionalDouble.empty(),
                OptionalDouble.empty()
            );
            return;
        }

        ShooterKinematics.MovingShotRequest request = new ShooterKinematics.MovingShotRequest(
            robotPose,
            drivetrain.getState().Speeds,
            Constants.Turret.ROBOT_TO_TURRET_PIVOT,
            Constants.Shooter.EXIT_HEIGHT_METERS,
            hubTargetPoint,
            Constants.Turret.TURRET_ZERO_OFFSET,
            Constants.Shooter.MIN_LAUNCH_ANGLE_DEG,
            Constants.Shooter.MAX_LAUNCH_ANGLE_DEG,
            Constants.Shooter.MAX_EXIT_SPEED_MPS
        );
        Optional<ShooterKinematics.MovingShotSolution> movingShotSolution = ShooterKinematics.solveMovingShot(request);

        Rotation2d commandedTurretAngle = stationaryDesiredTurretAngle;
        Rotation2d desiredFieldYaw = stationaryFieldYaw;
        OptionalDouble mappedHoodPosition = OptionalDouble.empty();
        OptionalDouble mappedRollerRps = OptionalDouble.empty();

        if (movingShotSolution.isPresent()) {
            ShooterKinematics.MovingShotSolution solution = movingShotSolution.get();
            commandedTurretAngle = solution.desiredTurretYaw();
            desiredFieldYaw = solution.desiredFieldYaw();
            mappedHoodPosition = shooterCalibration.hoodPositionForLaunchAngleDeg(
                solution.recommendedLaunchPitch().getDegrees()
            );
            mappedRollerRps = shooterCalibration.rollerRpsForExitSpeedMps(solution.requiredExitSpeedMps());
        }

        turret.setTurretGoal(commandedTurretAngle);
        publishAimState(
            alliance.name(),
            true,
            visionHealthy,
            poseFresh,
            true,
            pivotFieldPosition,
            hubPoint,
            commandedTurretAngle,
            desiredFieldYaw,
            movingShotSolution,
            mappedHoodPosition,
            mappedRollerRps
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
        String allianceName,
        boolean inAllianceZone,
        boolean visionHealthy,
        boolean poseFresh,
        boolean autoAimActive,
        Translation2d pivotFieldPosition,
        Translation2d hubPoint,
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
            allianceName,
            inAllianceZone,
            visionHealthy,
            poseFresh,
            autoAimActive,
            pivotFieldPosition,
            hubPoint,
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
