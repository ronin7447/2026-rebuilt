package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.LinearShooterCalibration;

/** Latched manual trial. Uses the shared feed-readiness switch, with no feed timers. */
public final class LaunchSpeedCalibrationCommand extends Command {
    public static final String DASHBOARD_PREFIX = "LaunchSpeedCalibration/";
    private final Mechanisms mechanisms;
    private final DoubleSupplier requestedSpeedRps;
    private boolean validTrial;

    public LaunchSpeedCalibrationCommand(
        ShooterSubsystem shooter, TurretSubsystem turret,
        OuttakeSubsystem outtake, IndexerSubsystem spindexer
    ) {
        this(new Mechanisms() {
            @Override
            public void start(double signedRps, double hoodPositionRot) {
                turret.setAutoAimSuppressed(true);
                turret.invalidateAimState("launch-speed-calibration");
                turret.setTurretGoal(Rotation2d.kZero);
                shooter.setHoodEnabled(true);
                shooter.setTargetPositionRotation(hoodPositionRot);
                shooter.runRoller(signedRps);
                outtake.setRollerTargetVelocity(Constants.Outtake.ROLLER_VELOCITY);
                spindexer.setRollerTargetVelocity(Constants.Indexer.ROLLER_VELOCITY);
            }

            @Override
            public String feedBlocker() {
                if (!DriverStation.isTeleopEnabled()) return "not-teleop-enabled";
                if (!shooter.hasFiniteBoundedSetpoints() || !turret.hasFiniteBoundedSetpoint()) {
                    return "invalid-setpoints";
                }
                if (!shooter.isRollerEnabled()) return "shooter-disabled";
                return readinessBlocker(Constants.ShotMap.FEED_INTERLOCK_ENABLED,
                    shooter.atTargetSpeed(), shooter.isHoodAtTarget(),
                    turret.getPositionMode() && turret.isTurretAtAimGoal() && !turret.isOnSoftwareLimit());
            }

            @Override
            public void setFeed(boolean enabled) {
                if (enabled) {
                    outtake.enableRoller();
                    spindexer.enableRoller();
                } else {
                    outtake.disableRoller();
                    spindexer.disableRoller();
                }
            }

            @Override
            public double actualRps() {
                return shooter.getRollerVelocity();
            }

            @Override
            public void stop() {
                shooter.disableRoller();
                shooter.setRollerTargetVelocity(0.0);
                outtake.disableRoller();
                outtake.setRollerTargetVelocity(0.0);
                spindexer.disableRoller();
                spindexer.setRollerTargetVelocity(0.0);
                shooter.setHoodEnabled(false);
                turret.setPositionMode(false);
                turret.setAutoAimSuppressed(true);
                turret.invalidateAimState("calibration-stopped");
            }
        }, LaunchSpeedCalibrationCommand::readRequestedSpeedRps, shooter, turret, outtake, spindexer);

        SmartDashboard.setDefaultBoolean(DASHBOARD_PREFIX + "UseDashboardRPS",
            Constants.LaunchSpeedCalibration.USE_DASHBOARD_RPS);
        SmartDashboard.setDefaultNumber(DASHBOARD_PREFIX + "RequestedRPS",
            Constants.LaunchSpeedCalibration.ROLLER_SPEED_RPS);
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Running", false);
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Feeding", false);
        SmartDashboard.putString(DASHBOARD_PREFIX + "Status", "stopped");
    }

    // Hardware boundary allows command lifecycle and readiness tests without CAN devices.
    interface Mechanisms {
        void start(double signedRps, double hoodPositionRot);
        String feedBlocker();
        void setFeed(boolean enabled);
        double actualRps();
        void stop();
    }

    LaunchSpeedCalibrationCommand(
        Mechanisms mechanisms, DoubleSupplier requestedSpeedRps, Subsystem... requirements
    ) {
        this.mechanisms = mechanisms;
        this.requestedSpeedRps = requestedSpeedRps;
        addRequirements(requirements);
    }

    static double readRequestedSpeedRps() {
        boolean useDashboard = SmartDashboard.getBoolean(DASHBOARD_PREFIX + "UseDashboardRPS",
            Constants.LaunchSpeedCalibration.USE_DASHBOARD_RPS);
        SmartDashboard.putString(DASHBOARD_PREFIX + "ActiveSource", useDashboard ? "dashboard" : "code");
        return useDashboard
            ? SmartDashboard.getNumber(DASHBOARD_PREFIX + "RequestedRPS", Double.NaN)
            : Constants.LaunchSpeedCalibration.ROLLER_SPEED_RPS;
    }

    static String readinessBlocker(boolean interlockEnabled, boolean shooterAtSpeed,
        boolean hoodAtTarget, boolean turretAtGoal) {
        if (!interlockEnabled) return "";
        if (!shooterAtSpeed) return "waiting-for-shooter-speed";
        if (!hoodAtTarget) return "waiting-for-hood";
        if (!turretAtGoal) return "waiting-for-turret";
        return "";
    }

    @Override
    public void initialize() {
        mechanisms.stop();
        double speedRps = requestedSpeedRps.getAsDouble();
        double hoodPositionRot = new LinearShooterCalibration()
            .hoodPositionForLaunchAngleDeg(Constants.LaunchSpeedCalibration.LAUNCH_ANGLE_DEG)
            .orElse(Double.NaN);
        validTrial = Double.isFinite(speedRps) && speedRps > 0.0
            && speedRps <= Constants.Shooter.MAX_ROLLER_RPS
            && Double.isFinite(hoodPositionRot)
            && hoodPositionRot >= Constants.Shooter.MIN_HOOD_POSITION_ROT
            && hoodPositionRot <= Constants.Shooter.MAX_HOOD_POSITION_ROT;
        SmartDashboard.putNumber(DASHBOARD_PREFIX + "ActiveRPS", speedRps);
        SmartDashboard.putNumber(DASHBOARD_PREFIX + "HoodTargetRot", hoodPositionRot);
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Running", validTrial);
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Feeding", false);
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "ReadinessChecksEnabled",
            Constants.ShotMap.FEED_INTERLOCK_ENABLED);
        SmartDashboard.putString(DASHBOARD_PREFIX + "Status",
            !validTrial ? "invalid-speed-or-hood-target"
                : (Constants.ShotMap.FEED_INTERLOCK_ENABLED ? "waiting-for-readiness" : "starting"));
        if (validTrial) {
            mechanisms.start(Constants.Shooter.ROLLER_DIRECTION_SIGN * speedRps, hoodPositionRot);
        }
    }

    @Override
    public void execute() {
        String blocker = validTrial ? mechanisms.feedBlocker() : "invalid-speed-or-hood-target";
        boolean feed = blocker.isEmpty();
        mechanisms.setFeed(feed);
        SmartDashboard.putNumber(DASHBOARD_PREFIX + "ActualRPS", Math.abs(mechanisms.actualRps()));
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Feeding", feed);
        SmartDashboard.putString(DASHBOARD_PREFIX + "Status", feed ? "feeding" : blocker);
    }

    @Override
    public void end(boolean interrupted) {
        mechanisms.stop();
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Running", false);
        SmartDashboard.putBoolean(DASHBOARD_PREFIX + "Feeding", false);
        if (validTrial) SmartDashboard.putString(DASHBOARD_PREFIX + "Status", "stopped");
    }

    @Override
    public boolean isFinished() {
        return !validTrial;
    }
}
