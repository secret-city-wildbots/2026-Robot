package frc.robot.Commands.Turret;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems
import frc.robot.Actors.Subsystems.Turret;

public class TrackHubCommand extends Command {

    // Real Variables
    private final Turret turret;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Translation2d hubPosition;

    /**
     * Creates and sets up the TrackHubCommand
     * 
     * @param intake The subsystem to be controlled by the command ({@link Turret})
     */
    public TrackHubCommand(Turret turret, Supplier<Pose2d> robotPoseSupplier) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.turret = turret;
        this.robotPoseSupplier = robotPoseSupplier;
        this.hubPosition = new Translation2d(4.625594,4.02336);
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // calculate the angle needed for the turret
        Translation2d robotPos = robotPoseSupplier.get().getTranslation();
        Translation2d delta = hubPosition.minus(robotPos);
        Rotation2d fieldAngleToTarget = delta.getAngle();
        Rotation2d turretTarget = fieldAngleToTarget.minus(robotPoseSupplier.get().getRotation());
        System.out.println(turretTarget);
        turret.setTargetAngle(turretTarget);
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the Intake subsystem
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        // TODO: add logic to end the command when a beam break is broken? We can use the pivot.get_hasPiece()
        return false;
    }
    
}
