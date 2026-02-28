package frc.robot.Commands.Turret;

// Import WPILib Libraries
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
// Import Subsystems
import frc.robot.Actors.Subsystems.Turret;

public class Zero extends Command {

    // Real Variables
    private final Turret turret;

    /**
     * Creates and sets up the TrackHubCommand
     * 
     * @param turret            The subsystem to be controlled by the command
     *                          ({@link Turret})
     * @param robotPoseSupplier The pose of the robot (continuous supplier)
     * @param robotVelSupplier  The velocity of the robot (continuous supplier) (robot relative)
     */
    public Zero(Turret turret) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        this.turret.dc(0.03);
        this.turret.setForwardLimit(true);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        this.turret.zero();
        this.turret.setTargetAngle(new Rotation2d(0.0));
        this.turret.setForwardLimit(false);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return this.turret.beambreakActive();
    }

}
