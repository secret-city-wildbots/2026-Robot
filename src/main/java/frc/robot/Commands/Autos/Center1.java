package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Actors.Subsystems.Drivetrain;
import frc.robot.Actors.Subsystems.Intake;
import frc.robot.Actors.Subsystems.Pivot;
import frc.robot.Commands.Subsystems.Drivetrain.SimpleDrive;
import frc.robot.Commands.Subsystems.Intake.IntakeAlgaeCommand;
import frc.robot.Commands.Subsystems.Intake.OuttakeCoralCommand;
import frc.robot.Commands.pivot.PivotToPositionCommand;

public class Center1 extends SequentialCommandGroup {
    // Real Variables
    public Center1(Drivetrain drivetrain, Intake intake, Pivot pivot) {
        System.out.println("IN AUTO");
        // Reset IMU
        //drivetrain.resetIMU();
        // Assign the variables and add the subsystem as a requirement to the command
        addRequirements(drivetrain, intake, pivot);
        addCommands(
            new ParallelRaceGroup(
                new PivotToPositionCommand(pivot, 140.0),
                new WaitCommand(2.0)
            ),

            new SimpleDrive(drivetrain, 0, -0.5, 0, 0.75),

            new SimpleDrive(drivetrain, 0, 0.3, 0, 2),

            new SimpleDrive(drivetrain, 0, 0, 0, 3),

            new ParallelRaceGroup(
                new OuttakeCoralCommand(intake),
                new WaitCommand(1)
            ),

            new ParallelRaceGroup(
                new IntakeAlgaeCommand(intake),
                new WaitCommand(1)
            ),

            new ParallelCommandGroup(
                 new SimpleDrive(drivetrain, 0, -0.1, 0, 0.5),
                 new IntakeAlgaeCommand(intake)
            )
        );
    }
}