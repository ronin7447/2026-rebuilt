package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ManipulateIntake extends InstantCommand {
    public ManipulateIntake(boolean extend, IntakeSubsystem intakeSubsystem) {
        super(
            extend ? () -> {
                intakeSubsystem.enableExtension();
                intakeSubsystem.setIntakePosition(Constants.Intake.INTAKE_EXTENDED_POS);
                intakeSubsystem.enableRoller();
            } : () -> {
                intakeSubsystem.disableExtension();
                intakeSubsystem.setIntakePosition(Constants.Intake.INTAKE_COLLAPSED_POS);
                intakeSubsystem.disableRoller();
            },
            intakeSubsystem
        );
    }
}
