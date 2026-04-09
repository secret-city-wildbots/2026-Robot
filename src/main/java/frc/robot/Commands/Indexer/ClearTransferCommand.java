package frc.robot.Commands.Indexer;

// Import WPILib Commands
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Indexer.Indexer;
import frc.robot.Actors.Subsystems.Indexer.Transfer;

public class ClearTransferCommand extends Command {

    // Initialize the subsystems
    private final Transfer transfer;
    private final Indexer indexer;

    /**
     * Creates and sets up the SpinFuelCommand
     * 
     * @param transfer The subsystem to be controlled by the command ({@link Transfer})
     * @param indexer The subsystem to be controlled by the command ({@link Indexer})
     * @param transferRPS The rps for the transfer
     * @param indexerRPS The rps for the indexer
     */
    public ClearTransferCommand(
        Transfer transfer,
        Indexer indexer
        ) {
        // Set the subystems
        this.transfer = transfer;
        this.indexer = indexer;

        // Add subsystem requirements
        addRequirements(transfer, indexer);
    }

    @Override
    public void initialize() {
        // Start the transfer motor and reset the timer
        //?
        transfer.motor.dc(-0.3);
        indexer.motor.dc(-0.1);
        indexer.motor2.dc(-0.1);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        transfer.setRPS(0.0);
        indexer.setRPS(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
