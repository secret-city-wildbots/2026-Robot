package frc.robot.Commands.Turret;

// Import Java Utils
import java.util.function.Supplier;

// Import WPILib Libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems
import frc.robot.Actors.Subsystems.Turret;
import frc.robot.Constants.TurretConstants;

public class TrackHubCommand extends Command {

    // Real Variables
    private final Turret turret;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelSupplier;
    private final Translation2d hubPosition = new Translation2d(4.625594, 4.02336);

    /**
     * Creates and sets up the TrackHubCommand
     * 
     * @param turret            The subsystem to be controlled by the command
     *                          ({@link Turret})
     * @param robotPoseSupplier The pose of the robot (continuous supplier)
     * @param robotVelSupplier  The velocity of the robot (continuous supplier) (robot relative)
     */
    public TrackHubCommand(Turret turret, Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelSupplier) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.turret = turret;
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelSupplier = robotVelSupplier;
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
        ChassisSpeeds adjustedRobotPos = robotVelSupplier.get().times(TurretConstants.turretBaseAirtime_s + TurretConstants.turretDistAirtime_sPm
                        * hubPosition.getDistance(robotPoseSupplier.get().getTranslation()));

        Translation2d adjustedHubPosition = hubPosition.minus(
                    new Translation2d(adjustedRobotPos.vxMetersPerSecond, adjustedRobotPos.vyMetersPerSecond)
                );
        Translation2d robotPos = robotPoseSupplier.get().getTranslation();
        Translation2d delta = adjustedHubPosition.minus(robotPos);
        Rotation2d fieldAngleToTarget = delta.getAngle();
        Rotation2d turretTarget = fieldAngleToTarget.minus(robotPoseSupplier.get().getRotation());

        // System.out.println(turretTarget);
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
