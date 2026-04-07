package frc.robot.Commands.Indexer;

// Import WPILib Commands
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Indexer.Indexer;
import frc.robot.Actors.Subsystems.Indexer.Transfer;
import frc.robot.Constants.IndexerConstants;

public class SpinAndFeedCommand extends Command {

    // Initialize the subsystems
    private final Transfer transfer;
    private final Indexer indexer;

    // Initialize the speeds
    private final double transferRPS;
    private final double indexerRPS;

    /**
     * Creates and sets up the SpinFuelCommand
     * 
     * @param transfer The subsystem to be controlled by the command ({@link Transfer})
     * @param indexer The subsystem to be controlled by the command ({@link Indexer})
     * @param transferRPS The rps for the transfer
     * @param indexerRPS The rps for the indexer and roller bed
     */
    public SpinAndFeedCommand(
        Transfer transfer,
        Indexer indexer,
        double transferRPS,
        double indexerRPS
        ) {
        // Set the subystems
        this.transfer = transfer;
        this.indexer = indexer;

        // Set the speeds
        this.transferRPS = transferRPS;
        this.indexerRPS = indexerRPS;

        // Add subsystem requirements
        addRequirements(transfer, indexer);
    }

    @Override
    public void initialize() {
        // Start the transfer motor and reset the timer
        transfer.setRPS(transferRPS);
        indexer.setRPS(indexerRPS, IndexerConstants.rollerRPS);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        // Turn off all motors
        transfer.setRPS(0.0);
        indexer.setRPS(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
