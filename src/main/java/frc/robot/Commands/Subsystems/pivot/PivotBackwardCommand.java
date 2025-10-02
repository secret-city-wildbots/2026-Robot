package frc.robot.Commands.Subsystems.pivot;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems
import frc.robot.Actors.Subsystems.Pivot;

public class PivotBackwardCommand extends Command {
    // Real Variables
    private final Pivot pivot;

    /**
     * Creates and sets up the PivotBackwardCommand
     * 
     * @param pivot2 The subsystem to be controlled by the command ({@link Pivot})
     */
    public PivotBackwardCommand(Pivot pivot) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        // Call the pivot subsystem backward function
        pivot.backward();
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the pivot subsystem
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        // TODO: add logic to end the command when the encoder reaches a specific value or a limit is reached?
        return false;
    }
}
