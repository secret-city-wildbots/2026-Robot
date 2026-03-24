package frc.robot.Commands.Elevator;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Elevator.ElevatorLift;
import frc.robot.Constants.ElevatorConstants;

public class ClimbAfterTopLimitSwitch extends Command {
    
    // Define variables
    private final ElevatorLift elevatorLift;

    /**
     * Creates and sets up the ClimbAfterTopLimitSwitch
     * 
     * @param elevatorLift The subsystem to be controlled by the command ({@link ElevatorLift})
     */
    public ClimbAfterTopLimitSwitch(ElevatorLift elevatorLift) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.elevatorLift = elevatorLift;
        addRequirements(this.elevatorLift);
    }

    @Override
    public void initialize() {
        // Call the ElevatorLift subsystem set function
        elevatorLift.set(-ElevatorConstants.maxSpeedPercentage);
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the ElevatorLift subsystem
        elevatorLift.set(0.0);
    }

    @Override
    public boolean isFinished() {
        // End the command when we the set rotations after the limit switch
        return elevatorLift.climbAfterTopLimitSwitch();
    }
}
