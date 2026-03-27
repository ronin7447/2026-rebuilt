package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(
        ShooterSubsystem shooterSubsystem,
        IndexerSubsystem indexerSubsystem,
        OuttakeSubsystem outtakeSubsystem
    ) {
        addCommands(
            new StartShooter(shooterSubsystem),
            new InstantCommand(outtakeSubsystem::enableRoller, outtakeSubsystem),
            new WaitCommand(0.2),
            new InstantCommand(indexerSubsystem::enableRoller, indexerSubsystem)
        );
    }

}
