package frc.robot.Commands.Spindexer;

// Import WPILib Commands
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;

public class SpinAndFeedCommand extends Command {

    // Initialize the subsystems
    private final Transfer transfer;
    private final Spindexer spindexer;

    // Initialize the speeds
    private final double transferRPS;
    private final double spindexerRPS;

    /**
     * Creates and sets up the SpinFuelCommand
     * 
     * @param transfer The subsystem to be controlled by the command ({@link Transfer})
     * @param spindexer The subsystem to be controlled by the command ({@link Spindexer})
     * @param transferRPS The rps for the transfer
     * @param spindexerRPS The rps for the spindexer
     */
    public SpinAndFeedCommand(
        Transfer transfer,
        Spindexer spindexer,
        double transferRPS,
        double spindexerRPS
        ) {
        // Set the subystems
        this.transfer = transfer;
        this.spindexer = spindexer;

        // Set the speeds
        this.transferRPS = transferRPS;
        this.spindexerRPS = spindexerRPS;

        // Add subsystem requirements
        addRequirements(transfer, spindexer);
    }

    @Override
    public void initialize() {
        // Start the transfer motor and reset the timer
        transfer.setRPS(transferRPS);
        spindexer.setRPS(spindexerRPS);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        // Turn off all motors
        transfer.setRPS(0.0);
        spindexer.setRPS(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
