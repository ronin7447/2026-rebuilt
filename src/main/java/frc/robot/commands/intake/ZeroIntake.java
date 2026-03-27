package frc.robot.commands.intake;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ZeroIntake extends Command {

    IntakeSubsystem intake;

    public ZeroIntake(IntakeSubsystem intake) {

        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {

        intake.disableExtension();
        intake.disableRoller();

        intake.getExtensionKrakenMaster().setControl(new VoltageOut(-1));
    }

    @Override
    public boolean isFinished() {

            return intake.isStallDetected();
    }

    @Override
    public void end(boolean interrupted) {

        intake.zeroEncoder();
        intake.getExtensionKrakenMaster().setControl(new VoltageOut(0));
    }
}
