package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ManipulateIntake extends Command {

    boolean extend;
    IntakeSubsystem intake;

    public ManipulateIntake(boolean extend, IntakeSubsystem intake) {

        this.extend = extend;
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {


        System.out.println("intake");
        if (extend) {

            intake.setIntakePosition(Constants.Intake.INTAKE_EXTENDED_POS);
            intake.enableExtension();
    
        } else {
            intake.disableRoller();
            intake.setIntakePosition(Constants.Intake.INTAKE_COLLAPSED_POS);
            intake.enableExtension();
           
        }
    }

    @Override
    public void execute() {

        // if (intake.isStallDetected()) {

        //     cancel();
        // }
    }

    @Override
    public boolean isFinished() {

            // DriverStation.reportError(String.valueOf(intake.getExtensionKrakenMaster().getClosedLoopError().getValueAsDouble()), false);
            
            return Math.abs(intake.getIntakePosition() - intake.getTargetRot()) <= Constants.Intake.INTAKE_TARGET_ROT_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {

        if (interrupted) {

            if (extend) {

                new ManipulateIntake(false, intake);
                
            } else {

                intake.setIntakePosition(intake.getIntakePosition());
            }

            return;
        }

        if (!extend) {

            // intake.disableExtension();
        }

        if (extend) {
            intake.enableRoller();
            intake.disableExtension();
        }
    }
}
