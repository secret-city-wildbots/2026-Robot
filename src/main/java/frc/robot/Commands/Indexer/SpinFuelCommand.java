package frc.robot.Commands.Indexer;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Indexer.Indexer;

public class SpinFuelCommand extends Command {
    // Real Variables
    private final Indexer indexer;
    private final double indexerRPS;
    private final double rollerRPS;

    /**
     * Creates and sets up the SpinFuelCommand
     * 
     * @param indexer The subsystem to be controlled by the command ({@link Indexer})
     * @param indexerRPS The rps for the indexer
     * @param rollerRPS The rps for the roller bed
     */
    public SpinFuelCommand(Indexer indexer, double indexerRPS, double rollerRPS) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.indexer = indexer;
        this.indexerRPS = indexerRPS;
        this.rollerRPS = rollerRPS;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        // Call the indexer subsystem start function
        indexer.setRPS(this.indexerRPS, this.rollerRPS);
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the indexer subsystem
        indexer.setRPS(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}
