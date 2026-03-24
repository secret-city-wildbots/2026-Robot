package frc.robot.Commands.Spindexer;

// Import WPILib Commands
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;

public class AutoStopIndexCommand extends Command {

    // Initialize the subsystems
    private final Transfer transfer;
    private final Spindexer spindexer;


    /**
     * Creates and sets up the SpinFuelCommand
     * 
     * @param transfer The subsystem to be controlled by the command ({@link Transfer})
     * @param spindexer The subsystem to be controlled by the command ({@link Spindexer})
     */
    public AutoStopIndexCommand(Transfer transfer, Spindexer spindexer) 
{
        // Set the subystems
        this.transfer = transfer;
        this.spindexer = spindexer;

        // Add subsystem requirements
        addRequirements(transfer, spindexer);
    }

    @Override
    public void initialize() {
        // Sets transfer and spindexer to what they are in Constants
        transfer.setRPS(0);
        spindexer.setRPS(0);
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
