package frc.robot.Commands.Turret;

// Import WPILib Libraries
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// Import Subsystems
import frc.robot.Actors.Subsystems.Shooter.Turret;

public class JoystickAimCommand extends Command {

    // Real Variables
    private final Turret turret;
    private final CommandXboxController joystick;

    /**
     * Creates and sets up the TrackHubCommand
     * 
     * @param turret            The subsystem to be controlled by the command
     *                          ({@link Turret})
     * @param robotPoseSupplier The pose of the robot (continuous supplier)
     * @param robotVelSupplier  The velocity of the robot (continuous supplier) (robot relative)
     */
    public JoystickAimCommand(Turret turret, CommandXboxController joystick) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.turret = turret;
        this.joystick = joystick;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Only use this for constants
    }

    @Override
    public void execute() {
        // calculate the angle needed for the turret
        //adjustedRobotPos is a pos, despite being the chassisspeed type lol
        double angle = Math.atan2(joystick.getLeftY(), joystick.getLeftX()) * 180.0 / Math.PI; // Convert radians to degrees
        Rotation2d turretTarget = Rotation2d.fromDegrees(angle);

        // System.out.println(turretTarget);
        if (Math.abs(joystick.getLeftY()) + Math.abs(joystick.getLeftX()) < 0.2) {
            // If the joystick is near the center, do not update the turret target
            return;
        }
        turret.setTargetAngle(turretTarget);
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will...
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }

}