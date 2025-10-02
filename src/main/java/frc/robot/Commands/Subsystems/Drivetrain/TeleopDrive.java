package frc.robot.Commands.Subsystems.Drivetrain;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Import Subsystems
import frc.robot.Actors.Subsystems.Drivetrain;
import frc.robot.Utils.JoystickScaler;

public class TeleopDrive extends Command {
    // Real Variables
    private final Drivetrain drivetrain;
    private final CommandXboxController driverController;

    /**
     * Creates and sets up the TeleopDriveCommand
     * 
     * @param drivetrain The subsystem to be controlled by the command ({@link Drivetrain})
     * @param driverController The controller giving the drivebase inputs
     */
    public TeleopDrive(Drivetrain drivetrain, CommandXboxController driverController) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
    drivetrain.drive(
        // Multiply by max speed to map the joystick unitless inputs to actual units.
        // This will map the [-1, 1] to [max speed backwards, max speed forwards],
        // converting them to actual units.
        JoystickScaler.scale(-driverController.getLeftY())*0.6,
        JoystickScaler.scale(driverController.getLeftX())*0.6,
        JoystickScaler.scale(-driverController.getRightX())*0.8
    );
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // Do end the command
        return true;
    }
}