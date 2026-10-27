package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/** Timed pivot pulses while feeding; never waits for the pivot to arrive. */
public class AntiJamPulse extends Command {
    private final IntakeSubsystem intake;
    private final BooleanSupplier feeding;
    private final BooleanSupplier inhibited;
    private boolean originallyExtended;
    private boolean pulsing;
    private boolean retracted;
    private double feedingSince;
    private double stepSince;

    public AntiJamPulse(IntakeSubsystem intake, BooleanSupplier feeding, BooleanSupplier inhibited) {
        this.intake = intake;
        this.feeding = feeding;
        this.inhibited = inhibited;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        originallyExtended = Math.abs(intake.getTargetRot() - Constants.Intake.INTAKE_EXTENDED_POS)
            <= Constants.Intake.INTAKE_TARGET_ROT_TOLERANCE;
        pulsing = false;
        feedingSince = Double.NaN;
    }

    @Override
    public void execute() {
        if (!originallyExtended || Constants.Intake.ANTI_JAM_PULSE_CYCLES <= 0) return;
        double now = Timer.getFPGATimestamp();
        if (inhibited.getAsBoolean() || !feeding.getAsBoolean()) {
            restoreExtended();
            feedingSince = Double.NaN;
            return;
        }
        if (Double.isNaN(feedingSince)) feedingSince = now;
        if (now - feedingSince < Constants.Intake.ANTI_JAM_FEED_DELAY_SEC) return;
        if (!pulsing) {
            pulsing = true;
            retracted = true;
            stepSince = now;
            intake.commandExtensionPosition(Constants.Intake.ANTI_JAM_PULSE_RETRACT_POS);
        } else if (now - stepSince >= Constants.Intake.ANTI_JAM_PULSE_DWELL_SEC) {
            retracted = !retracted;
            stepSince = now;
            intake.commandExtensionPosition(retracted
                ? Constants.Intake.ANTI_JAM_PULSE_RETRACT_POS : Constants.Intake.INTAKE_EXTENDED_POS);
        }
    }

    private void restoreExtended() {
        if (pulsing) intake.commandExtensionPosition(Constants.Intake.INTAKE_EXTENDED_POS);
        pulsing = false;
    }

    @Override
    public void end(boolean interrupted) {
        restoreExtended();
    }
}
