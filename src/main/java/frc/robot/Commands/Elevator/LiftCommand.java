package frc.robot.Commands.Elevator;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Elevator.ElevatorLift;

public class LiftCommand extends Command {
    // Real Variables
    private final ElevatorLift elevatorLift;
    private final CommandXboxController joystick;

    /**
     * Creates and sets up the LiftCommand
     * 
     * @param elevatorLift The subsystem to be controlled by the command ({@link ElevatorLift})
     * @param joystick input to control the elevator ({@link CommandXboxController})
     */
    public LiftCommand(ElevatorLift elevatorLift, CommandXboxController joystick) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.elevatorLift = elevatorLift;
        this.joystick = joystick;
        addRequirements(this.elevatorLift);
    }

    @Override
    public void initialize() {
        // Call the ElevatorLift subsystem start function
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
        elevatorLift.set(this.joystick.getLeftY() * 0.8);
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the ElevatorLift subsystem
        elevatorLift.set(0.0);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}