
package frc.robot.Commands.Intake;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;

public class IntakeSequence extends ParallelCommandGroup {

    public IntakeSequence(Intake intake, IntakeExtension extender) {

        addCommands(

            // 1. extend out intake
            new ExtensionCommand(extender, 90.0),

            // 2. turn on intake
            new IntakeCommand(intake, 0.6)

        );
    }
    
}
