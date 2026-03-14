package frc.robot.Commands.Intake;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;

public class ExtensionCommand extends Command {
    // Define Variables
    private final IntakeExtension intakeExtension;
    private final double motorSpeedPercentage;

    /**
     * Creates and sets up the ExtensionCommand
     * 
     * @param intakeExtension The subsystem to be controlled by the command ({@link IntakeExtension})
     * @param motorSpeedPercentage The the input to control intake extension speed
     */
    public ExtensionCommand(IntakeExtension intakeExtension, double motorSpeedPercentage) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.intakeExtension = intakeExtension;
        this.motorSpeedPercentage = motorSpeedPercentage;
        addRequirements(this.intakeExtension);
    }

    @Override
    public void initialize() {
        // Call the intake subsystem start function
        intakeExtension.set(this.motorSpeedPercentage);
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the IntakeExtension subsystem
        intakeExtension.set(0.0);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}