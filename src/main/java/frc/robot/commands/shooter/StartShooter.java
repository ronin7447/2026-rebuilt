package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends Command {

    ShooterSubsystem shooter;

    public StartShooter(ShooterSubsystem shooter) {

        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {

        shooter.enableRoller();
        shooter.setRollerTargetVelocity(Constants.Shooter.ROLLER_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        
        shooter.disableRoller();
    }

    @Override
    public boolean isFinished() {

        return shooter.atTargetSpeed();
    }
}
