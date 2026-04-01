package frc.robot.commands.autopilot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ShortRangeAutoAlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d goal;
    private final PIDController xController = new PIDController(
        Constants.Drivebase.SHORT_ALIGN_TRANSLATION_KP,
        Constants.Drivebase.SHORT_ALIGN_TRANSLATION_KI,
        Constants.Drivebase.SHORT_ALIGN_TRANSLATION_KD
    );
    private final PIDController yController = new PIDController(
        Constants.Drivebase.SHORT_ALIGN_TRANSLATION_KP,
        Constants.Drivebase.SHORT_ALIGN_TRANSLATION_KI,
        Constants.Drivebase.SHORT_ALIGN_TRANSLATION_KD
    );
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.Drivebase.SHORT_ALIGN_ROTATION_KP,
        Constants.Drivebase.SHORT_ALIGN_ROTATION_KI,
        Constants.Drivebase.SHORT_ALIGN_ROTATION_KD,
        new TrapezoidProfile.Constraints(
            Constants.Drivebase.SHORT_ALIGN_MAX_ANGULAR_SPEED_RAD_PER_SEC,
            Constants.Drivebase.SHORT_ALIGN_MAX_ANGULAR_SPEED_RAD_PER_SEC
        )
    );
    private final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withDeadband(0.0)
        .withRotationalDeadband(0.0);
    private final Timer settleTimer = new Timer();

    public ShortRangeAutoAlignCommand(CommandSwerveDrivetrain drivetrain, Pose2d goal) {
        this.drivetrain = drivetrain;
        this.goal = goal;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drivetrain.getState().Pose;

        xController.reset();
        yController.reset();
        thetaController.reset(currentPose.getRotation().getRadians());

        settleTimer.stop();
        settleTimer.reset();

        publishTelemetry(currentPose);
        SmartDashboard.putString("Autopilot/Mode", "SHORT_ALIGN");
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;

        double vxCmd = MathUtil.clamp(
            xController.calculate(currentPose.getX(), goal.getX()),
            -Constants.Drivebase.SHORT_ALIGN_MAX_LINEAR_SPEED_MPS,
            Constants.Drivebase.SHORT_ALIGN_MAX_LINEAR_SPEED_MPS
        );
        double vyCmd = MathUtil.clamp(
            yController.calculate(currentPose.getY(), goal.getY()),
            -Constants.Drivebase.SHORT_ALIGN_MAX_LINEAR_SPEED_MPS,
            Constants.Drivebase.SHORT_ALIGN_MAX_LINEAR_SPEED_MPS
        );
        double omegaCmd = MathUtil.clamp(
            thetaController.calculate(currentPose.getRotation().getRadians(), goal.getRotation().getRadians()),
            -Constants.Drivebase.SHORT_ALIGN_MAX_ANGULAR_SPEED_RAD_PER_SEC,
            Constants.Drivebase.SHORT_ALIGN_MAX_ANGULAR_SPEED_RAD_PER_SEC
        );

        drivetrain.setControl(
            alignRequest.withVelocityX(vxCmd)
                .withVelocityY(vyCmd)
                .withRotationalRate(omegaCmd)
        );

        boolean atPositionTolerance = currentPose.getTranslation().getDistance(goal.getTranslation())
            <= Constants.Drivebase.SHORT_ALIGN_POSITION_TOLERANCE_METERS;
        boolean atHeadingTolerance = Math.abs(currentPose.getRotation().minus(goal.getRotation()).getDegrees())
            <= Constants.Drivebase.SHORT_ALIGN_HEADING_TOLERANCE_DEG;

        if (atPositionTolerance && atHeadingTolerance) {
            if (!settleTimer.isRunning()) {
                settleTimer.reset();
                settleTimer.start();
            }
        } else {
            settleTimer.stop();
            settleTimer.reset();
        }

        publishTelemetry(currentPose);
        SmartDashboard.putBoolean("Autopilot/AtPositionTolerance", atPositionTolerance);
        SmartDashboard.putBoolean("Autopilot/AtHeadingTolerance", atHeadingTolerance);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            alignRequest.withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
        SmartDashboard.putString("Autopilot/Mode", "IDLE");
    }

    @Override
    public boolean isFinished() {
        return settleTimer.hasElapsed(Constants.Drivebase.SHORT_ALIGN_SETTLE_TIME_SEC);
    }

    private void publishTelemetry(Pose2d currentPose) {
        SmartDashboard.putNumber("Autopilot/GoalX", goal.getX());
        SmartDashboard.putNumber("Autopilot/GoalY", goal.getY());
        SmartDashboard.putNumber("Autopilot/GoalHeadingDeg", goal.getRotation().getDegrees());
        SmartDashboard.putNumber(
            "Autopilot/DistanceToGoal",
            currentPose.getTranslation().getDistance(goal.getTranslation())
        );
    }
}
