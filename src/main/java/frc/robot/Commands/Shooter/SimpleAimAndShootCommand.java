package frc.robot.Commands.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Actors.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;
import frc.robot.Commands.Spindexer.ClearTransferCommand;
import frc.robot.Commands.Spindexer.SpinAndFeedCommand;

public class SimpleAimAndShootCommand extends ParallelCommandGroup {
    public SimpleAimAndShootCommand(
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelSupplier,Spindexer spindexer, Transfer transfer, Shooter shooter, Turret turret) {
        addCommands(
            new SimpleShootCommand(shooter, turret, robotPoseSupplier, robotVelSupplier),
            
            new SequentialCommandGroup(
                new ParallelRaceGroup( //?
                    new ClearTransferCommand(transfer, spindexer),
                    new WaitCommand(0.6)
                ),
                new SpinAndFeedCommand(transfer, spindexer, SpindexerConstants.transferRPS, SpindexerConstants.spindexerRPS)
            )
        );
        addRequirements(shooter, turret);
    }
}