package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    
    public ShootCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, OuttakeSubsystem outtakeSubsystem) {
        addCommands(
            new InstantCommand(shooterSubsystem::enableRoller, shooterSubsystem),
            new WaitCommand(0.2),
            new InstantCommand(outtakeSubsystem::enableRoller, outtakeSubsystem),
            new WaitCommand(0.2),
            new InstantCommand(indexerSubsystem::enableRoller, indexerSubsystem)
        );
    }

}
