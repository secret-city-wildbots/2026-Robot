package frc.robot.Commands.Subsystems.Drivetrain;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Drivetrain;

public class UnlockAzimuths extends Command {
    // Real Variables
    private final Drivetrain drivetrain;

    /**
     * Creates and sets up the IntakeAlgaeCommand
     * 
     * @param intake The subsystem to be controlled by the command ({@link Intake})
     */
    public UnlockAzimuths(Drivetrain drivetrain) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.unlockAzimuths();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("unlocked azimuths");
    }

    @Override
    public boolean isFinished() {
        // Do end the command
        return true;
    }
}