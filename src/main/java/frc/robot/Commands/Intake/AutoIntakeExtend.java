package frc.robot.Commands.Intake;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Intake.Intake;
// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;

public class AutoIntakeExtend extends Command {
    // Real Variables
    private final IntakeExtension intakeExtension;
    private final Intake intake;

    /**
     * Creates and sets up the ExtensionCommand
     * 
     * @param intakeExtension The subsystem to be controlled by the command ({@link IntakeExtension})
     * @param intakeExtensionPos The pos for the motor
     */
    public AutoIntakeExtend(Intake intake, IntakeExtension intakeExtension) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.intakeExtension = intakeExtension;
        this.intake = intake;
        addRequirements(intakeExtension);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Call the intakeExtension subsystem setIntakePos function
        intakeExtension.setIntakePos(86.0);
        intake.set(0.6);
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