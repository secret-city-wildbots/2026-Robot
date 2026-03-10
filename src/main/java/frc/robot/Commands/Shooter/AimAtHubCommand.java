package frc.robot.Commands.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utils.ShotPredictor;
import frc.robot.Utils.ShotPredictor.Shot;

public class AimAtHubCommand extends Command {
    // Real Variables
    private final Shooter shooter;
    private final Turret turret;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelSupplier;

    /**
     * Creates and sets up the ShootCommand
     * 
     * @param shooter  The subsystem to be controlled by the command
     *                 ({@link Shooter})
     * @param motorRpS The rotations per second required for the motor
     */
    public AimAtHubCommand(Shooter shooter, Turret turret, Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelSupplier) {
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
        Shot shot = ShotPredictor.predict(this.robotPoseSupplier, this.robotVelSupplier);

        this.shooter.setHoodPos(shot.tilt.getDegrees());
        this.shooter.setRPS(shot.velocity_mPs / (ShooterConstants.wheelRadius_m * Math.PI * 2));
        this.turret.setTargetAngle(shot.yaw);
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the spindexer
        // subsystem
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}