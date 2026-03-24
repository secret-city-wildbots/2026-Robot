package frc.robot.Commands.Shooter;


// Import WPILib Libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import java.util.function.Supplier;
import frc.robot.Utils.ShotPredictor;
import frc.robot.Utils.ShotPredictor.Shot;
import frc.robot.Constants.ShooterConstants;

public class SimpleShootCommand extends Command {
    // Real Variables
    private final Shooter shooter;
    private final Turret turret;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelSupplier;

    /**
     * Creates and sets up the ShootCommand
     * 
     * @param shooter The subsystem to be controlled by the command ({@link Shooter})
     * @param turret The subsystem to be controlled by the command ({@link Turret})
     * @param robotPoseSupplier The pose supplier of the robot drivetrain to get its current position live
     * @param robotVelSupplier the vel supplier of the robot drivetrain to get its current vel live
     */
    public SimpleShootCommand(
        Shooter shooter,
        Turret turret,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelSupplier
    ) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelSupplier = robotVelSupplier;
        this.shooter = shooter;
        this.turret = turret;
        addRequirements(shooter);
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //Shot shot = ShotPredictor.predict(this.robotPoseSupplier, this.robotVelSupplier);
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
            this.shooter.setHoodAngle(15.0);
            this.shooter.setRPS(30.0);
            this.turret.setTargetAngle(new Rotation2d());
       // }
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.setHoodAngle(0);
        this.shooter.setRPS(0);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}