
package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;

public class IntakeSequence extends ParallelCommandGroup {

    public IntakeSequence(Intake intake, IntakeExtension extender) {

        /*Command runafter = new ParallelRaceGroup(
                    new ExtensionCommand(extender, 90.0),
                    new IntakeCommandDC(intake, -20),
                    new WaitCommand(0.2)
                );*/

        addCommands(

            // 1. extend out intake
            new ExtensionCommand(extender, IntakeConstants.maxDegree),

            new SequentialCommandGroup(
                new WaitCommand(0.5),
                // 2. turn on intake
                new IntakeCommand(intake, 0.6)
            )

        );
        //this.andThen(runafter);
    }
    
}
