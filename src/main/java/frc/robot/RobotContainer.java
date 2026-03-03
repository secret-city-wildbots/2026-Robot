
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import Phoenix6 Libraries
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// Import Path Planner Libraries
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

// Import WPILib Librarires
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// Import Custom TunerConstants
import frc.robot.generated.TunerConstants;

// Import subystems
import frc.robot.Actors.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Actors.Subsystems.FlashLightTurret;
import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;
import frc.robot.Actors.Subsystems.Elevator.ElevatorLift;
import frc.robot.Actors.Subsystems.Elevator.ElevatorHook;
import frc.robot.Commands.Elevator.RetractLiftCommand;
import frc.robot.Commands.Elevator.ClimbSequenceL1;
import frc.robot.Commands.Elevator.ExtendLiftCommand;
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;
// Import Commands
import frc.robot.Commands.Spindexer.SpinAndFeedCommand;
import frc.robot.Commands.Spindexer.SpinFuelCommand;
import frc.robot.Commands.Spindexer.TransferFuelCommand;

// Import Custom Commands
import frc.robot.Commands.Intake.IntakeCommand;
import frc.robot.Commands.Intake.ExtensionCommand;
import frc.robot.Commands.FlashLightTurret.TrackHubCommand;
import frc.robot.Commands.Intake.IntakeSequence;
public class RobotContainer {
    // TODO: Set max speed back to normal
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxSpeed = 1.0; // kSpeedAt12Volts desired top speed

    // TODO: Set max rotation back to normal
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Spindexer spindexer = new Spindexer();
    public final Transfer transfer = new Transfer();

    private final FlashLightTurret flturret = new FlashLightTurret(44, 0);
    public final Intake intake = new Intake();
    public final IntakeExtension intakeExtension = new IntakeExtension();
    private final ElevatorLift elevatorLift = new ElevatorLift();
    private final ElevatorHook elevatorHook = new ElevatorHook();

      /* Path follower */
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        // TODO: Set default auto
        autoChooser = AutoBuilder.buildAutoChooser("Reverse 9");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // drivetrain.resetPose(new Pose2d( new Translation2d(2,2), new Rotation2d()));

        configureBindings();

         // Register Named Commands within Pathplanner
        NamedCommands.registerCommand("IntakeFuel", new IntakeCommand(intake, 0.2));
        NamedCommands.registerCommand("OuttakeFuel", new IntakeCommand(intake, -0.2));
        NamedCommands.registerCommand("IntakeExtend", new ExtensionCommand(intakeExtension, 0.2));
        NamedCommands.registerCommand("IntakeRetract", new ExtensionCommand(intakeExtension, -0.2));

        // Register Named Commands within Pathplanner
        NamedCommands.registerCommand("L1Climb", new ClimbSequenceL1(elevatorLift));

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();


    }

    private void configureBindings() {
        /*************************************************
         * Commands for Drivetrain
         *************************************************/

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Set the default command for the turret
        flturret.setDefaultCommand(new TrackHubCommand(flturret, drivetrain::getPose));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Break when pressing A
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Orientate wheels when pressing B and and moving left and right joysticks
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // TODO: Enable logger
        // drivetrain.registerTelemetry(logger::telemeterize);
        
        joystick.leftBumper().toggleOnTrue(new IntakeSequence(intake, intakeExtension));
        drivetrain.registerTelemetry(logger::telemeterize);

        // Descend from Auto L1 + Retract Lift down
        joystick.y().whileTrue(new ExtendLiftCommand(elevatorLift));
        joystick.a().whileTrue(new RetractLiftCommand(elevatorLift, false));
       
        /*************************************************
         * Commands for Spindexer
         *************************************************/

        joystick.x().whileTrue(new SpinAndFeedCommand(
            transfer, spindexer, 30, 10, 0.5
        ));

        joystick.y().whileTrue(new SpinFuelCommand(spindexer, 10));
    }


    public Command getAutonomousCommand() {
         /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
        // /* Run the path selected from the auto chooser */
        // return Commands.print("No autonomous command configured");
    }
}
