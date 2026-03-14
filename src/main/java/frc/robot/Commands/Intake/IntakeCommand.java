package frc.robot.Commands.Intake;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Intake.Intake;

public class IntakeCommand extends Command {
    // Real Variables
    private final Intake intake;
    private final double motorSpeedPercentage;

    /**
     * Creates and sets up the IntakeCommand
     * 
     * @param intake The subsystem to be controlled by the command ({@link Intake})
     * @param motorSpeedPercentage The the input to control intake motor speed
     */
    public IntakeCommand(Intake intake, double motorSpeedPercentage) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.intake = intake;
        this.motorSpeedPercentage = motorSpeedPercentage;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        // Call the intake subsystem set function
        intake.set(this.motorSpeedPercentage);
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the IntakeExtension subsystem
        intake.set(0.0);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}