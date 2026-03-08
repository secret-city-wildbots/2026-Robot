package frc.robot.Commands.Elevator;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Elevator.ElevatorHook;
import frc.robot.Constants.ElevatorConstants;

public class RotateHookToPositionCommand extends Command {
    // Real Variables
    private final ElevatorHook elevatorHook;
    private final double targetAngle;
    private static final double ANGLE_TOLERANCE = ElevatorConstants.angleTolerance; // degrees

    /**
     * Creates and sets up the RotateHookToPositionCommand
     * 
     * @param ElevatorHook The subsystem to be controlled by the command ({@link ElevatorHook})
     */
    public RotateHookToPositionCommand(ElevatorHook elevatorHook, double targetAngle) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.elevatorHook = elevatorHook;
        this.targetAngle = targetAngle;
        addRequirements(this.elevatorHook);
    }

    @Override
    public void initialize() {
        // Call the ElevatorHook subsystem set target function
        elevatorHook.setTargetAngle(this.targetAngle);
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
        double current = elevatorHook.getCurrentAngle();

        if (current < targetAngle) {
            elevatorHook.set(1.0);  // rotate outward
        } else {
            elevatorHook.set(-1.0); // rotate inward
        }
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the ElevatorHook subsystem
        elevatorHook.set(0.0);
    }

    @Override
    public boolean isFinished() {
        // check to see where to end the command

        return Math.abs(elevatorHook.getCurrentAngle() - targetAngle) <= ANGLE_TOLERANCE;
    }
}