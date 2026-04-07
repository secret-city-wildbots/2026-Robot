package frc.robot.Commands.Indexer;

// Import WPILib Commands
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Indexer.Indexer;
import frc.robot.Actors.Subsystems.Indexer.Transfer;

public class AutoStopIndexCommand extends Command {

    // Initialize the subsystems
    private final Transfer transfer;
    private final Indexer indexer;


    /**
     * Creates and sets up the SpinFuelCommand
     * 
     * @param transfer The subsystem to be controlled by the command ({@link Transfer})
     * @param indexer The subsystem to be controlled by the command ({@link Indexer})
     */
    public AutoStopIndexCommand(Transfer transfer, Indexer indexer) 
{
        // Set the subystems
        this.transfer = transfer;
        this.indexer = indexer;

        // Add subsystem requirements
        addRequirements(transfer, indexer);
    }

    @Override
    public void initialize() {
        // Sets transfer and indexer to what they are in Constants
        transfer.setRPS(0);
        indexer.setRPS(0, 0);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
