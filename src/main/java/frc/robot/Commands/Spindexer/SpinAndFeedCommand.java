package frc.robot.Commands.Spindexer;

// Import WPILib Commands
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Import Spindexer Subsystems (Spindexer and Transfer)
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;

public class SpinAndFeedCommand extends Command {

    // Initialize the subsystems
    private final Transfer transfer;
    private final Spindexer spindexer;

    // Initialize the speeds
    private final double transferRPS;
    private final double spindexerRPS;

    // Initialize the spinup time and timer
    private final double spinupTime;
    private final Timer timer = new Timer();

    public SpinAndFeedCommand(
        Transfer transfer,
        Spindexer spindexer,
        double transferRPS,
        double spindexerRPS,
        double spinupTime
    ) {
        // Set the subystems
        this.transfer = transfer;
        this.spindexer = spindexer;

        // Set the speeds
        this.transferRPS = transferRPS;
        this.spindexerRPS = spindexerRPS;

        // Set the spinup time
        this.spinupTime = spinupTime;

        // Add subsystem requirements
        addRequirements(transfer, spindexer);
    }

    @Override
    public void initialize() {
        // Start the transfer motor and reset the timer
        transfer.setRPS(transferRPS);
        timer.restart();
    }

    @Override
    public void execute() {
        // Check to see if the startup time has elapsed
        if (timer.hasElapsed(spinupTime)) {
            // Once the time has passed, start the spindexer
            spindexer.setRPS(spindexerRPS);
        }
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
