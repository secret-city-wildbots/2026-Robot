package frc.robot.Commands.Elevator;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Elevator.ElevatorHook;

public class HookCommand extends Command {
    // Real Variables
    private final ElevatorHook elevatorHook;
    private final CommandXboxController joystick;

    /**
     * Creates and sets up the HookCommand
     * 
     * @param elevatorHook The subsystem to be controlled by the command ({@link ElevatorHook})
     * @param joystick input to control the elevator ({@link CommandXboxController})
     */
    public HookCommand(ElevatorHook elevatorHook, CommandXboxController joystick) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.elevatorHook = elevatorHook;
        this.joystick = joystick;
        addRequirements(this.elevatorHook);
    }

    @Override
    public void initialize() {
        // Call the ElevatorHook subsystem start function
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
        elevatorHook.set(this.joystick.getLeftY() * 0.2);
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the ElevatorLift subsystem
        elevatorHook.set(0.0);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}