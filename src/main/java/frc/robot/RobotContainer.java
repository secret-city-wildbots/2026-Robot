
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import Constants
import frc.robot.Constants.*;

// Import Subsystems
import frc.robot.Actors.Subsystems.Intake;
import frc.robot.Actors.Subsystems.Pivot;
import frc.robot.Actors.Subsystems.Drivetrain;

// Import Commands
import frc.robot.Commands.Autos.Center1;
import frc.robot.Commands.Drivetrain.ResetIMU;
import frc.robot.Commands.Drivetrain.TeleopDrive;
import frc.robot.Commands.Intake.IntakeAlgaeCommand;
import frc.robot.Commands.Intake.IntakeCoralCommand;
import frc.robot.Commands.Intake.OuttakeAlgaeCommand;
import frc.robot.Commands.Intake.OuttakeCoralCommand;
import frc.robot.Commands.pivot.PivotToPositionCommand;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake(0);
  public static final Pivot pivot = new Pivot(1, 0);
  private final Drivetrain drivetrain = new Drivetrain();

  // Instantiate drive and manipulator Xbox Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DriverControllerPort);
  private final CommandXboxController manipulatorController = new CommandXboxController(OperatorConstants.ManipulatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    drivetrain.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new TeleopDrive(drivetrain, driverController)
    );
  }

  /**e this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}
   * /{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule the `ResetIMUCommand` when holding the down D-Pad of the driver controller for 1 second
    driverController.pov(180).debounce(1).onTrue(new ResetIMU(drivetrain));

    // Schedule `IntakeCoralCommand` when the driver controller's leftBumper button is pressed, cancel on release
    // Schedule `OuttakeCoralCommand` when the driver controller's leftTrigger button is pressed, cancel on release
    driverController.leftBumper().whileTrue(new IntakeCoralCommand(intake));
    driverController.leftTrigger().whileTrue(new OuttakeCoralCommand(intake));

    // Schedule `IntakeAlgaeCommand` when the driver controller's rightBumper button is pressed, continues on press
    // Schedule `OuttakeAlgaeCommand` when the driver controller's rightTrigger button is pressed, cancel on release, also cancels intake
    driverController.rightBumper().onTrue(new IntakeAlgaeCommand(intake));
    driverController.rightTrigger().whileTrue(new OuttakeAlgaeCommand(intake));

    // TODO: Comment these commands so we know which positions the pivot is in? Coral pickup, algae pickup? etc.
    manipulatorController.button(7).onTrue(new PivotToPositionCommand(pivot, 194.0));
    manipulatorController.button(8).onTrue(new PivotToPositionCommand(pivot, 140.0));
    manipulatorController.a().onTrue(new PivotToPositionCommand(pivot, 5.0));
    manipulatorController.b().onTrue(new PivotToPositionCommand(pivot, 20.0));
    manipulatorController.y().onTrue(new PivotToPositionCommand(pivot, 138.0));
    manipulatorController.leftBumper().onTrue(new PivotToPositionCommand(pivot, 110.0));
  }

  public Command getAutonomousCommand() {
    return new Center1(drivetrain, intake, pivot);
  }
}