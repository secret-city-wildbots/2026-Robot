package frc.robot.Commands.Turret;


// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import frc.robot.Utils.ShotPredictor.Shot;

public class AimAtHubTurret extends Command {
    // Real Variables
    private final Turret turret;

    /**
     * Creates and sets up the ShootCommand
     * 
     * @param shooter The subsystem to be controlled by the command ({@link Shooter})
     * @param turret The subsystem to be controlled by the command ({@link Turret})
     * @param robotPoseSupplier The pose supplier of the robot drivetrain to get its current position live
     * @param robotVelSupplier the vel supplier of the robot drivetrain to get its current vel live
     */
    public AimAtHubTurret(
        Turret turret
    ) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Shot shot = Robot.shot;
        //System.out.println("Shooter Hood Angle (degrees): " + shot.tilt.getDegrees());
        //System.out.println("Shooter Speed (mPs): " + shot.velocity_mPs / (ShooterConstants.wheelRadius_m * Math.PI * 2));
        //System.out.println("Turret Angle (?): " + shot.yaw);
        //System.out.println("Turret Angle (?): " + shot.yaw.getDegrees());
        //System.out.println("Turret Motor (?): " + shot.yaw.getRotations());

        // trench auto-stow //?
        // if (((x > 3.7 && x < 5.5) || //?
        // (x > 16-3.7 && x < 16-5.5)) &&
        // (y > 6.8 || y < 1.2)) {
        //     this.shooter.setHoodAngle(0);
        //     this.shooter.setRPS(0);
        // } else {
        this.turret.setTargetAngle(shot.yaw);
       // }
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}