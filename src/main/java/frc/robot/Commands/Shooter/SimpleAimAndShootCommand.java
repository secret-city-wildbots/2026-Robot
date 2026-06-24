package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import frc.robot.Actors.Subsystems.Indexer.Indexer;
import frc.robot.Actors.Subsystems.Indexer.Transfer;
import frc.robot.Commands.Indexer.SpinAndFeedCommand;

public class SimpleAimAndShootCommand extends ParallelCommandGroup {
    public SimpleAimAndShootCommand(Indexer indexer, Transfer transfer, Shooter shooter, Turret turret) {
        addCommands(
            new SimpleShootCommand(shooter, turret),
            new SpinAndFeedCommand(transfer, indexer, IndexerConstants.transferRPS, IndexerConstants.indexerRPS)
        );
        addRequirements(shooter, turret);
    }
}