package frc.robot.Commands.Indexer;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems (Indexer)
import frc.robot.Actors.Subsystems.Indexer;

public class ShootCommand extends Command {
    // Real Variables
    private final Indexer indexer;

    /**
     * Creates and sets up the ShootCommand
     * 
     * @param indexer The subsystem to be controlled by the command ({@link Indexer})
     * @param motorRpS The rotations per second required for the motor
     */
    public ShootCommand(Indexer indexer) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        // Call the indexer subsystem setRPS function
        indexer.shoot();
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the spindexer subsystem
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}
