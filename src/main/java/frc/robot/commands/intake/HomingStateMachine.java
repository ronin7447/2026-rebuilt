package frc.robot.commands.intake;

/** Pure timeout/interruption policy for hard-stop homing. */
public final class HomingStateMachine {
    private final double timeoutSec;
    private double startSec;
    private Outcome outcome = Outcome.NOT_STARTED;

    public HomingStateMachine(double timeoutSec) {
        this.timeoutSec = timeoutSec;
    }

    public void begin(double nowSec) {
        startSec = nowSec;
        outcome = Outcome.RUNNING;
    }

    public boolean update(double nowSec, boolean confirmedStall) {
        if (outcome != Outcome.RUNNING) return outcome != Outcome.NOT_STARTED;
        if (confirmedStall) outcome = Outcome.SUCCESS;
        else if (nowSec - startSec >= timeoutSec) outcome = Outcome.TIMEOUT;
        return outcome != Outcome.RUNNING;
    }

    public Outcome end(boolean interrupted) {
        if (interrupted && outcome == Outcome.RUNNING) outcome = Outcome.INTERRUPTED;
        return outcome;
    }

    public Outcome outcome() { return outcome; }

    public enum Outcome { NOT_STARTED, RUNNING, SUCCESS, TIMEOUT, INTERRUPTED }
}
