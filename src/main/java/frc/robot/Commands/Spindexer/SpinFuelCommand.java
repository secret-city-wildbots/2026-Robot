package frc.robot.Commands.Spindexer;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;

public class SpinFuelCommand extends Command {
    // Real Variables
    private final Spindexer spindexer;
    private final double motorSpeedPercentage;

    /**
     * Creates and sets up the SpinFuelCommand
     * 
     * @param spindexer The subsystem to be controlled by the command ({@link Spindexer})
     */
    public SpinFuelCommand(Spindexer spindexer, double motorSpeedPercentage) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.spindexer = spindexer;
        this.motorSpeedPercentage = motorSpeedPercentage;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        // Call the spindexer subsystem start function
        spindexer.set(this.motorSpeedPercentage);
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the spindexer subsystem
        spindexer.set(0.0);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}
