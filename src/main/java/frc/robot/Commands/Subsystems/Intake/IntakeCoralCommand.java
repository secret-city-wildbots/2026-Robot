package frc.robot.Commands.Subsystems.Intake;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;

// Import Subsystems
import frc.robot.Actors.Subsystems.Intake;

public class IntakeCoralCommand extends Command {
    // Real Variables
    private final Intake intake;

    /**
     * Creates and sets up the IntakeCoralCommand
     * 
     * @param intake The subsystem to be controlled by the command ({@link Intake})
     */
    public IntakeCoralCommand(Intake intake) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        System.out.println("IN HERE 2");
        // Call the Intake subsystem acquireCoral Function
        intake.intakeCoral();
    }

    @Override
    public void end(boolean interrupted) {
        // When the command is interrupted or cancelled, we will stop the Intake subsystem
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        // TODO: add logic to end the command when a beam break is broken? We can use the pivot.get_hasPiece()
        return false;
    }
}