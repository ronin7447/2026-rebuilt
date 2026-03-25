package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class EnableIntakeCommand extends InstantCommand {
    public EnableIntakeCommand(IntakeSubsystem intakeSubsystem) {
        super(() -> {
            intakeSubsystem.enableExtension();
            intakeSubsystem.setIntakePosition(Constants.Intake.INTAKE_EXTENDED_POS);
            intakeSubsystem.enableRoller();
        }, intakeSubsystem);
    }
}
