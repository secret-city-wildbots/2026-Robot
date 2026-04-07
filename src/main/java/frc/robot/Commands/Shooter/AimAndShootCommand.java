package frc.robot.Commands.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Actors.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import frc.robot.Actors.Subsystems.Indexer.Indexer;
import frc.robot.Actors.Subsystems.Indexer.Transfer;
import frc.robot.Commands.Indexer.ClearTransferCommand;
import frc.robot.Commands.Indexer.SpinAndFeedCommand;

public class AimAndShootCommand extends ParallelCommandGroup {
    public AimAndShootCommand(
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelSupplier,Indexer indexer, Transfer transfer, Shooter shooter, Turret turret) {
        addCommands(
            new AimAtHubCommand(shooter, turret, robotPoseSupplier, robotVelSupplier).andThen(Commands.waitSeconds(0.6)),
            
            // new SequentialCommandGroup(
            //     new ParallelRaceGroup( //?                                   // Should not be need as we are letting it clear out rest of balls
            //         new ClearTransferCommand(transfer, indexer),
            //         new WaitCommand(0.6)
            // ),

                new SpinAndFeedCommand(transfer, indexer, IndexerConstants.transferRPS, IndexerConstants.indexerRPS)
            );
        addRequirements(shooter, turret);
    }
}
