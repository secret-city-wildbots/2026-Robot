
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import Constants
import frc.robot.Constants.*;

// Import Subsystems
import frc.robot.Actors.Subsystems.Drivetrain;

//Import Commands
import frc.robot.Commands.Subsystems.Drivetrain.TeleopDrive;
import frc.robot.Commands.Subsystems.Drivetrain.UnlockAzimuths;
import frc.robot.Commands.Subsystems.Drivetrain.ResetIMU;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  // Instantiate drive and manipulator Xbox Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DriverControllerPort);

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
    driverController.pov(90).debounce(1).onTrue(new UnlockAzimuths(drivetrain));
  }

  public Command getAutonomousCommand() {
    return Commands.print("no auto selected");
  }
}