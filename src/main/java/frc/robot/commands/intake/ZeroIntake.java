package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ZeroIntake extends Command {

    IntakeSubsystem intake;
    private final DoubleSupplier clockSec;
    private double startTimeSec;
    private boolean confirmedStall;
    private boolean timedOut;
    private boolean unavailable;
    private final HomingStateMachine stateMachine = new HomingStateMachine(Constants.Intake.HOMING_TIMEOUT_SEC);

    public ZeroIntake(IntakeSubsystem intake) {
        this(intake, Timer::getFPGATimestamp);
    }

    ZeroIntake(IntakeSubsystem intake, DoubleSupplier clockSec) {

        this.intake = intake;
        this.clockSec = clockSec;

        addRequirements(intake);
    }

    @Override
    public void initialize() {

        confirmedStall = false;
        timedOut = false;
        startTimeSec = clockSec.getAsDouble();
        stateMachine.begin(startTimeSec);
        unavailable = !intake.beginHoming();
        if (unavailable) stateMachine.end(true);
    }

    @Override
    public boolean isFinished() {

        if (unavailable) return true;
        confirmedStall = intake.isStallDetected();
        timedOut = stateMachine.update(clockSec.getAsDouble(), confirmedStall)
            && stateMachine.outcome() == HomingStateMachine.Outcome.TIMEOUT;
        return stateMachine.outcome() == HomingStateMachine.Outcome.SUCCESS || timedOut;
    }

    @Override
    public void end(boolean interrupted) {

        HomingStateMachine.Outcome outcome = stateMachine.end(interrupted);
        IntakeSubsystem.HomingStatus status = switch (outcome) {
            case SUCCESS -> IntakeSubsystem.HomingStatus.SUCCESS;
            case TIMEOUT -> IntakeSubsystem.HomingStatus.TIMEOUT;
            default -> IntakeSubsystem.HomingStatus.INTERRUPTED;
        };
        intake.finishHoming(status);
    }
}
