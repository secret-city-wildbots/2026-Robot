package frc.robot.Commands.Indexer;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Indexer.Transfer;

public class TransferFuelCommand extends Command {
    // Real Variables
    private final Transfer indexerTransfer;
    private final double motorRPS;

    /**
     * Creates and sets up the TransferFuelCommand
     * 
     * @param indexerTransfer The subsystem to be controlled by the command ({@link Transfer})
     * @param motorRPS The rps for the transfer
     */
    public TransferFuelCommand(Transfer indexerTransfer, double motorRPS) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.indexerTransfer = indexerTransfer;
        this.motorRPS = motorRPS;
        addRequirements(indexerTransfer);
    }

    @Override
    public void initialize() {
        // Call the indexerTransfer subsystem start function
        indexerTransfer.setRPS(this.motorRPS);
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the indexerTransfer subsystem
        indexerTransfer.setRPS(0.0);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}
