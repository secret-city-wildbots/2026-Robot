package frc.robot.Commands.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import frc.robot.Actors.Subsystems.Indexer.Indexer;
import frc.robot.Actors.Subsystems.Indexer.Transfer;
import frc.robot.Commands.Indexer.SpinAndFeedCommand;

public class SimpleAimAndShootCommand extends ParallelCommandGroup {
    public SimpleAimAndShootCommand(Indexer indexer, Transfer transfer, Shooter shooter, Turret turret, double hoodAngle, double rps,Rotation2d turretAngle) {
        addCommands(
            new SimpleShootCommand(shooter, turret, hoodAngle, rps, turretAngle),
            new SpinAndFeedCommand(transfer, indexer, IndexerConstants.transferRPS, IndexerConstants.indexerRPS, turret::isLocked)
        );
        addRequirements(shooter, turret);
    }
}