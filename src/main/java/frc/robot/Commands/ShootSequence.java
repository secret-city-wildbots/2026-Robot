package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;
import frc.robot.Commands.Spindexer.SpinAndFeedCommand;
public class ShootSequence extends SequentialCommandGroup {

    public ShootSequence(
        // SpinAndFeedCommand
        Transfer transfer,
        Spindexer spindexer,
        double transferRPS,
        double spindexerRPS,
        double spinupTime
        ) {

        addCommands(

            // 1. Shoot
            // Spin up Shooter

            // 2. Spindexer (transfer)
            new SpinAndFeedCommand(transfer, spindexer, transferRPS, spindexerRPS, spinupTime)

        );
    }
}
