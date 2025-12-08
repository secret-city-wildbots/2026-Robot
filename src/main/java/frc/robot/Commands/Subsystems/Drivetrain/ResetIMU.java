package frc.robot.Commands.Subsystems.Drivetrain;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems
import frc.robot.Actors.Subsystems.Drivetrain;

public class ResetIMU extends Command {
    // Real Variables
    private final Drivetrain drivetrain;

    /**
     * Creates and sets up the ResetIMU command. Resets the IMU and thus the heading angle
     * 
     * @param drivetrain The subsystem to be controlled by the command ({@link Drivetrain})
     */
    public ResetIMU(Drivetrain drivetrain) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.resetIMU();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("reset IMU");
    }

    @Override
    public boolean isFinished() {
        // Do end the command
        return true;
    }
}