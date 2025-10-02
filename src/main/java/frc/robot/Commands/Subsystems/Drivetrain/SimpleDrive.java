package frc.robot.Commands.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Timer;
// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Import Subsystems
import frc.robot.Actors.Subsystems.Drivetrain;
import frc.robot.Utils.JoystickScaler;

public class SimpleDrive extends Command {
    // Real Variables
    private final Drivetrain drivetrain;
    private final double xpow;
    private final double ypow;
    private final double hpow;
    private final double waitTime_s;
    private final Timer waitTimer = new Timer();

    /**
     * Creates and sets up the TeleopDriveCommand
     * 
     * @param drivetrain The subsystem to be controlled by the command ({@link Drivetrain})
     * @param driverController The controller giving the drivebase inputs
     */
    public SimpleDrive(Drivetrain drivetrain, double xpow, double ypow, double hpow, double waitTime_s) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.drivetrain = drivetrain;
        this.xpow = xpow;
        this.ypow = ypow;
        this.hpow = hpow;
        this.waitTime_s = waitTime_s;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        waitTimer.reset();
        waitTimer.start();
    }

    @Override
    public void execute() {
    drivetrain.drive(
        // Multiply by max speed to map the joystick unitless inputs to actual units.
        // This will map the [-1, 1] to [max speed backwards, max speed forwards],
        // converting them to actual units.
        -ypow,
        xpow,
        -hpow
    );
    }

    @Override
    public void end(boolean interrupted) {
        waitTimer.stop();
    }

    @Override
    public boolean isFinished() {
        // Do end the command
        return waitTimer.hasElapsed(waitTime_s);
    }
}