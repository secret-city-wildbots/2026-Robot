package frc.robot.Commands.Shooter;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems (Shooter)
import frc.robot.Actors.Subsystems.Shooter;

public class ShootCommand extends Command {
    // Real Variables
    private final Shooter shooter;
    private final double motorRpS;
    private final double hoodAngleDeg;

    /**
     * Creates and sets up the ShootCommand
     * 
     * @param shooter The subsystem to be controlled by the command ({@link Shooter})
     * @param motorRpS The rotations per second required for the motor
     */
    public ShootCommand(Shooter shooter, double motorRpS, double hoodAngleDeg) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.shooter = shooter;
        this.motorRpS = motorRpS;
        this.hoodAngleDeg = hoodAngleDeg;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Call the shooter subsystem setRPS function
        shooter.setRPS(this.motorRpS);
        shooter.setHoodAngle(this.hoodAngleDeg);
    }

    @Override
    public void execute() {
        // Only use execute if we have dynamically changing speeds. This is called each loop (~20ms).
        // So if we have just a constant speed, use initialize to avoid spamming the canbus network.
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the spindexer subsystem
        shooter.setRPS(0.0);
        shooter.setHoodAngle(5.0);
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}
