
package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Commands.Intake.IntakeCommand;

public class IntakeSequence extends SequentialCommandGroup {

    public IntakeSequence(Intake intake, IntakeExtension extender) {

        addCommands(

            // 1. extend out intake
            new ExtensionCommand(extender, 0.2),

            // 2. turn on intake
            new IntakeCommand(intake, 0.8)

        );
    }
    
}
