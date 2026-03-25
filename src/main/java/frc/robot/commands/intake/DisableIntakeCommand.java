package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class DisableIntakeCommand extends InstantCommand {
    public DisableIntakeCommand(IntakeSubsystem intakeSubsystem) {
        super(() -> {
            intakeSubsystem.enableExtension();
            intakeSubsystem.setIntakePosition(Constants.Intake.INTAKE_COLLAPSED_POS);
            intakeSubsystem.disableRoller();
        }, intakeSubsystem);
    }
}
