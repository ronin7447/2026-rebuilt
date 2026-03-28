// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.compositions.AutoAimTurretCommand;
import frc.robot.commands.compositions.ShootCommand;
import frc.robot.commands.intake.ManipulateIntake;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = Constants.Drivebase.MAX_SPEED; // 1.0 * kSpeedAt12Volts desired top speed
    private double MaxAngularRate = Constants.Drivebase.MAX_ANGULAR_RATE;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drivebase.DRIVE_DEADBAND).withRotationalDeadband(Constants.Drivebase.ROTATIONAL_DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // this is the closed-loop velocity control request used for tuning the drivetrain's velocity PID constants in SysId.
    private final SwerveRequest.RobotCentric tuneVel = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withDeadband(0.0)
        .withRotationalDeadband(0.0);





    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final VisionSubsystem  m_vision = new VisionSubsystem(drivetrain);
    public final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();

    public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

    public final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
    public final OuttakeSubsystem m_OuttakeSubsystem = new OuttakeSubsystem();

    public RobotContainer() {
        m_TurretSubsystem.setDefaultCommand(new AutoAimTurretCommand(
            drivetrain,
            m_vision,
            m_TurretSubsystem,
            m_ShooterSubsystem
        ));
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on POV LEFT.
        joystick.povLeft().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); 
        joystick.povRight().onTrue((Commands.sequence((Commands.runOnce(() -> {
            m_TurretSubsystem.zeroEncoder();
        })))));
        /* Right Bumper: Enable Intake */
        joystick.rightBumper().onTrue(new ManipulateIntake(true, m_IntakeSubsystem));

        /* Left Bumper: Disable Intake */
        joystick.leftBumper().onTrue(new ManipulateIntake(false, m_IntakeSubsystem));

        joystick.rightTrigger().whileTrue(new ShootCommand(
            m_ShooterSubsystem,
            m_IndexerSubsystem,
            m_OuttakeSubsystem
        ));

        joystick.leftTrigger().onTrue((Commands.sequence((Commands.runOnce(() -> {
            m_IndexerSubsystem.disableRoller();
            m_IndexerSubsystem.setRollerTargetVelocity(0);
        })))));

        // joystick.a().onTrue((Commands.sequence((Commands.runOnce(() -> {
        //     m_OuttakeSubsystem.enableRoller();
        //     m_OuttakeSubsystem.setRollerTargetVelocity(Constants.Outtake.ROLLER_VELOCITY);
        // })))));

        // joystick.b().onTrue((Commands.sequence((Commands.runOnce(() -> {
        //     m_OuttakeSubsystem.disableRoller();
        //     m_OuttakeSubsystem.setRollerTargetVelocity(0);
        // })))));

        // joystick.povUp().onTrue((Commands.sequence((Commands.runOnce(() -> {
        //     m_ShooterSubsystem.enableRoller();
        //     m_ShooterSubsystem.setRollerTargetVelocity(Constants.Shooter.ROLLER_VELOCITY);
        // })))));

        // joystick.povRight().onTrue((Commands.sequence((Commands.runOnce(() -> {
        //     m_ShooterSubsystem.disableRoller();
        //     m_ShooterSubsystem.setRollerTargetVelocity(0);
        // })))));

        joystick.povDown().onTrue((Commands.sequence((Commands.runOnce(() -> {
            m_TurretSubsystem.setAutoAimSuppressed(false);
            DriverStation.reportError(String.valueOf(m_TurretSubsystem.getTurretPositionRotations()), false);
            DriverStation.reportError(String.valueOf(m_TurretSubsystem.getKraken().getPosition().getValueAsDouble()), false);
        })))));
        joystick.povUp().onTrue((Commands.sequence((Commands.runOnce(() -> {
            m_TurretSubsystem.setAutoAimSuppressed(true);
        })))));

        /* Right Trigger (while holdiing): Starts Shooter + Outtake + Indexer */
        // joystick.a().whileTrue(new ShootCommand(m_ShooterSubsystem, m_IndexerSubsystem, m_OuttakeSubsystem));

        // for tuning: run a constant velocity sweep for SysId when right bumper is pressed.
        // joystick.rightBumper().onTrue(constantVelocitySweep());

        // joystick.rightStick().onTrue(Commands.runOnce(() -> m_TurretSubsystem.setAutoAimSuppressed(true)));
        // joystick.rightStick().onFalse(Commands.runOnce(() -> m_TurretSubsystem.setAutoAimSuppressed(false)));

        joystick.x().onTrue((Commands.runOnce(() -> {
            m_IntakeSubsystem.zeroEncoder();
        })));

        joystick.y().onTrue((Commands.runOnce(() -> {
            DriverStation.reportError(String.valueOf(m_IntakeSubsystem.getIntakePosition()), false);
        })));

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    /* Robot Functions Begins */

    /**
     * holdVx(double vxMetersPerSec, double seconds)
     * returns a command that holds the robot at a specified velocity for a specified duration. Used for SysId tuning.
     */
    // private Command holdVx(double vxMetersPerSec, double seconds) {
    //     return drivetrain.applyRequest(() ->
    //         tuneVel.withVelocityX(vxMetersPerSec)
    //                 .withVelocityY(0.0)
    //                 .withRotationalRate(0.0)
    //     ).withTimeout(seconds);
    // }

    /**
     * constantVelocitySweep()
     * returns a command that performs a constant velocity sweep for SysId tuning.
     */
    // private Command constantVelocitySweep() {
    //     return Commands.sequence(
    //         holdVx(0.5, 2.0),
    //         holdVx(1.0, 2.0),
    //         holdVx(0.0, 1.0)
    //     );
    // }
    
    /* Robot Functions Ends */

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }

    
}
