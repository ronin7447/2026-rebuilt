package frc.robot.commands.intake;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class AntiJamPulse extends SequentialCommandGroup {
    private final IntakeSubsystem intake;

    public AntiJamPulse(IntakeSubsystem intake) {
        this.intake = intake;

        if (Constants.Intake.ANTI_JAM_PULSE_CYCLES <= 0) {
            addCommands(Commands.none());
            return;
        }

        List<Command> pulseSequence = new ArrayList<>();
        pulseSequence.add(moveTo(Constants.Intake.INTAKE_EXTENDED_POS, false));

        for (int cycle = 0; cycle < Constants.Intake.ANTI_JAM_PULSE_CYCLES; cycle++) {
            pulseSequence.add(moveTo(Constants.Intake.ANTI_JAM_PULSE_RETRACT_POS, true));
            pulseSequence.add(moveTo(Constants.Intake.INTAKE_EXTENDED_POS, false));
        }

        addCommands(
            Commands.sequence(pulseSequence.toArray(Command[]::new))
                .finallyDo(interrupted -> {
                    if (interrupted) {
                        intake.commandExtensionPosition(Constants.Intake.INTAKE_EXTENDED_POS);
                    }
                })
        );
        addRequirements(intake);
    }

    private Command moveTo(double targetRot, boolean stopOnStall) {
        return Commands.sequence(
            Commands.runOnce(() -> intake.commandExtensionPosition(targetRot), intake),
            Commands.waitUntil(() -> intake.isExtensionAtTarget() || (stopOnStall && intake.isStallDetected()))
                .withTimeout(Constants.Intake.ANTI_JAM_PULSE_STEP_TIMEOUT_SEC),
            Commands.waitSeconds(Constants.Intake.ANTI_JAM_PULSE_DWELL_SEC)
        );
    }
}
