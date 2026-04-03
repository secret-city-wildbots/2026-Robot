package frc.robot.Commands.Shooter;


// Import WPILib Libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import java.util.function.Supplier;

public class FreeAim extends Command {
    // Real Variables
    private final Shooter shooter;
    private final Turret turret;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final CommandXboxController sadstick2;

    /**
     * Creates and sets up the ShootCommand
     * 
     * @param shooter The subsystem to be controlled by the command ({@link Shooter})
     * @param turret The subsystem to be controlled by the command ({@link Turret})
     * @param robotPoseSupplier The pose supplier of the robot drivetrain to get its current position live
     * @param robotVelSupplier the vel supplier of the robot drivetrain to get its current vel live
     */
    public FreeAim(
        Shooter shooter,
        Turret turret,
        Supplier<Pose2d> robotPoseSupplier,
        CommandXboxController sadstick2
    ) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.robotPoseSupplier = robotPoseSupplier;
        this.shooter = shooter;
        this.turret = turret;
        this.sadstick2 = sadstick2;
        addRequirements(shooter);
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (Robot.sad) {
            this.shooter.setHoodAngle(Math.abs(sadstick2.getRightY())*40.0+5.0);
            
            if (sadstick2.getLeftTriggerAxis() > 0.2) {
                this.shooter.setRPS(sadstick2.getLeftTriggerAxis()*30.0);
            } else if (sadstick2.getRightTriggerAxis() > 0.2) {
                this.shooter.setRPS(sadstick2.getRightTriggerAxis()*80.0);
            } else {
                this.shooter.setRPS(0.0);
            }

            Rotation2d robotRot = robotPoseSupplier.get().getRotation();

            double angle = 90.0-Math.atan2(sadstick2.getLeftY(), sadstick2.getLeftX()) * 180.0 / Math.PI; // Convert radians to degrees
            Rotation2d turretTarget = Rotation2d.fromDegrees(angle).minus(robotRot);

            // System.out.println(turretTarget);
            if (Math.abs(sadstick2.getLeftY()) + Math.abs(sadstick2.getLeftX()) < 0.2) {
                // If the joystick is near the center, do not update the turret target
                return;
            }
            turret.setTargetAngle(turretTarget);
        } else {
            this.shooter.setHoodAngle(0);
            this.shooter.setRPS(0);
        }
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