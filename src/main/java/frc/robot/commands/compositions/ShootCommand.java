package frc.robot.commands.compositions;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.sotm.ShotReadinessSnapshot;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private final Supplier<ShotReadinessSnapshot> readinessSupplier;
    private final BooleanSupplier manualOverrideSupplier;
    private final DoubleSupplier clockSec;
    private boolean feeding;

    public boolean isFeeding() {
        return feeding;
    }

    private final FeedSequencer sequencer = new FeedSequencer(
        Constants.ShotMap.FEED_RECOVERY_READY_TIME_SEC,
        Constants.ShotMap.OUTTAKE_DELAY_AFTER_READY_SEC,
        Constants.ShotMap.INDEXER_DELAY_AFTER_OUTTAKE_SEC,
        Constants.ShotMap.CONTINUE_FEED_AFTER_INITIAL_READY
    );

    public ShootCommand(
        ShooterSubsystem shooterSubsystem,
        IndexerSubsystem indexerSubsystem,
        OuttakeSubsystem outtakeSubsystem
    ) {
        this(
            shooterSubsystem,
            indexerSubsystem,
            outtakeSubsystem,
            () -> ShotReadinessSnapshot.evaluate(
                Timer.getFPGATimestamp(),
                DriverStation.isEnabled(),
                true,
                0.0,
                shooterSubsystem.isRollerEnabled(),
                shooterSubsystem.atTargetSpeed(),
                shooterSubsystem.isHoodEnabled(),
                shooterSubsystem.isHoodAtTarget(),
                true,
                true,
                true,
                shooterSubsystem.hasFiniteBoundedSetpoints(),
                Double.POSITIVE_INFINITY,
                Constants.ShotMap.MAX_SOLUTION_AGE_SEC
            ),
            () -> false,
            Timer::getFPGATimestamp
        );
    }

    public ShootCommand(
        ShooterSubsystem shooterSubsystem,
        IndexerSubsystem indexerSubsystem,
        OuttakeSubsystem outtakeSubsystem,
        Supplier<ShotReadinessSnapshot> readinessSupplier,
        BooleanSupplier manualOverrideSupplier,
        DoubleSupplier clockSec
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.readinessSupplier = readinessSupplier;
        this.manualOverrideSupplier = manualOverrideSupplier;
        this.clockSec = clockSec;

        addRequirements(shooterSubsystem, indexerSubsystem, outtakeSubsystem);
    }

    @Override
    public void initialize() {
        feeding = false;
        sequencer.reset();
        shooterSubsystem.setShootingHoodRequested(true);
        SmartDashboard.putBoolean("Feed/ShootRequested", true);

        shooterSubsystem.enableRoller();
        outtakeSubsystem.disableRoller();
        indexerSubsystem.disableRoller();

        outtakeSubsystem.setRollerTargetVelocity(Constants.Outtake.ROLLER_VELOCITY);
        indexerSubsystem.setRollerTargetVelocity(Constants.Indexer.ROLLER_VELOCITY);
    }

    @Override
    public void execute() {
        ShotReadinessSnapshot readiness = readinessSupplier.get();
        FeedSequencer.State state = sequencer.update(
            clockSec.getAsDouble(),
            readiness,
            manualOverrideSupplier.getAsBoolean(),
            Constants.ShotMap.FEED_INTERLOCK_ENABLED
        );
        feeding = state.outtakeEnabled() && state.indexerEnabled();
        if (state.outtakeEnabled()) outtakeSubsystem.enableRoller();
        else outtakeSubsystem.disableRoller();
        if (state.indexerEnabled()) indexerSubsystem.enableRoller();
        else indexerSubsystem.disableRoller();
        SmartDashboard.putBoolean("Feed/Permitted", state.permitted());
        SmartDashboard.putBoolean("Feed/ReadinessChecksEnabled", Constants.ShotMap.FEED_INTERLOCK_ENABLED);
        SmartDashboard.putBoolean("Feed/OverrideActive", state.overrideActive());
        SmartDashboard.putString("Feed/Reason", state.reason());
        SmartDashboard.putNumber("Feed/TransitionTimestampSec", state.transitionTimestampSec());
        SmartDashboard.putString("Feed/Blockers", state.permitted() ? "" : state.reason());
        SmartDashboard.putString("Feed/ReadinessWarnings", readiness.ready() ? "" : readiness.blockerSummary());
        SmartDashboard.putBoolean(
            "Feed/VisionDegradedWarning",
            readiness.visionDegraded(Constants.Turret.MAX_VISION_AGE_FOR_SHOT_SEC)
        );
    }

    @Override
    public void end(boolean interrupted) {
        feeding = false;
        shooterSubsystem.setShootingHoodRequested(false);
        shooterSubsystem.disableRoller();
        outtakeSubsystem.disableRoller();
        indexerSubsystem.disableRoller();
        SmartDashboard.putBoolean("Feed/ShootRequested", false);
        SmartDashboard.putBoolean("Feed/Permitted", false);
        SmartDashboard.putBoolean("Feed/OverrideActive", false);
        SmartDashboard.putString("Feed/Reason", interrupted ? "command-interrupted" : "command-ended");
        SmartDashboard.putString("Feed/Blockers", interrupted ? "command-interrupted" : "command-ended");
        SmartDashboard.putNumber("Feed/TransitionTimestampSec", clockSec.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
