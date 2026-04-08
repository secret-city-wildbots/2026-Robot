package frc.robot.Commands.Intake;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Intake.Intake;
// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;

public class AutoIntakeStop extends Command {
    // Real Variables
    private final Intake intake;

    /**
     * Creates and sets up the ExtensionCommand
     * 
     * @param intakeExtension The subsystem to be controlled by the command ({@link IntakeExtension})
     */
    public AutoIntakeStop(Intake intake) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Call the intakeExtension subsystem setIntakePos function
        intake.set(0);
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return true;
    }
}