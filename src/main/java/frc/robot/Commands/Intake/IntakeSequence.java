
package frc.robot.Commands.Intake;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;

public class IntakeSequence extends SequentialCommandGroup {

    public IntakeSequence(Intake intake, IntakeExtension extender) {

        addCommands(

            // 1. extend out intake
            // new ExtensionCommand(extender, 0.2),

            // 2. turn on intake
            new IntakeCommand(intake, 0.6)

        );
    }
    
}
