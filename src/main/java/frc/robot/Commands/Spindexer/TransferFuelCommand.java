package frc.robot.Commands.Spindexer;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Spindexer.Transfer;

public class TransferFuelCommand extends Command {
    // Real Variables
    private final Transfer spindexerTransfer;
    private final double motorRPS;

    /**
     * Creates and sets up the TransferFuelCommand
     * 
     * @param spindexerTransfer The subsystem to be controlled by the command ({@link Transfer})
     * @param motorRPS The rps for the transfer
     */
    public TransferFuelCommand(Transfer spindexerTransfer, double motorRPS) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.spindexerTransfer = spindexerTransfer;
        this.motorRPS = motorRPS;
        addRequirements(spindexerTransfer);
    }

    @Override
    public void initialize() {
        // Call the spindexerTransfer subsystem start function
        spindexerTransfer.setRPS(this.motorRPS);
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the spindexerTransfer subsystem
        spindexerTransfer.setRPS(0.0);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}
