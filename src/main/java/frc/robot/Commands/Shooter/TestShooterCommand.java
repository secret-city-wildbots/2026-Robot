package frc.robot.Commands.Shooter;


// Import WPILib Libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import java.util.function.Supplier;
import frc.robot.Utils.ShotPredictor;
import frc.robot.Utils.ShotPredictor.Shot;
import frc.robot.Constants.ShooterConstants;

public class TestShooterCommand extends Command {
    // Real Variables
    private final Shooter shooter;

    /**
     * Creates and sets up the ShootCommand
     * 
     * @param shooter The subsystem to be controlled by the command ({@link Shooter})
     * @param turret The subsystem to be controlled by the command ({@link Turret})
     * @param robotPoseSupplier The pose supplier of the robot drivetrain to get its current position live
     * @param robotVelSupplier the vel supplier of the robot drivetrain to get its current vel live
     */
    public TestShooterCommand(
        Shooter shooter
    ) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.shooter.setHoodAngle(30);
        this.shooter.setRPS(50);
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.setHoodAngle(7);
        this.shooter.setRPS(0);
        // When the command is interrupted or cancelled, we will stop the spindexer
        // subsystem
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}