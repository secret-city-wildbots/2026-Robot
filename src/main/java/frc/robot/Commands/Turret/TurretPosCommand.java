package frc.robot.Commands.Turret;

// Import Java Utils
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Turret;

public class TurretPosCommand extends Command {

    // Real Variables
    private final Turret turret;
    private final Rotation2d yaw;

    /**
     * Creates and sets up the TrackHubCommand
     * 
     * @param turret            The subsystem to be controlled by the command
     *                          ({@link Turret})
     * @param robotPoseSupplier The pose of the robot (continuous supplier)
     * @param robotVelSupplier  The velocity of the robot (continuous supplier) (robot relative)
     */
    public TurretPosCommand(Turret turret, Rotation2d yaw) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.turret = turret;
        this.yaw = yaw;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Only use this for constants
    }

    @Override
    public void execute() {
        turret.setTargetAngle(yaw);
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will...
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return true;
    }

}
