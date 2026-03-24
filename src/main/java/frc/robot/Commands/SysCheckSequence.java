
package frc.robot.Commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;
import frc.robot.Commands.Intake.ExtensionCommand;
import frc.robot.Commands.Intake.IntakeCommand;
import frc.robot.Commands.Intake.IntakeSequence;
import frc.robot.Commands.Shooter.TestShooterCommand;
import frc.robot.Commands.Spindexer.SpinFuelCommand;
import frc.robot.Commands.Spindexer.TransferFuelCommand;
import frc.robot.Commands.Turret.Zero;

public class SysCheckSequence extends SequentialCommandGroup {

    public SysCheckSequence(Intake intake, Transfer transfer, Spindexer spindexer, IntakeExtension extender, Turret turret, Shooter shooter) {

        addCommands(
            // Drive in Square
            new PathPlannerAuto("SysCheck"),
            //Intake and Extend
            new ParallelRaceGroup(
                new ExtensionCommand(extender, 86.0),
                new WaitCommand(2.0)
            ),
            //Stop Intake and Retract
            new ParallelRaceGroup(
                new IntakeCommand(intake, 0.6),
                new WaitCommand(3.0)
            ),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new IntakeCommand(intake, 0.0),
                    new ExtensionCommand(extender, 0.0)
                ),
                new WaitCommand(2.0)
            ),
            //Retract
            // Set Turret to 0
            new Zero(turret),
            // Spin Shooter
            new ParallelRaceGroup(
                new TestShooterCommand(shooter),
                new WaitCommand(3.0)
            ),
            
            // Spin Transfer
            new ParallelRaceGroup(
                new TransferFuelCommand(transfer, 10),
                new WaitCommand(2.0)
            ),

            // Spin Spindexer
            new ParallelRaceGroup( 
                new SpinFuelCommand(spindexer, 10),
                new WaitCommand(2.0)
            )
        );
    }
    
}
