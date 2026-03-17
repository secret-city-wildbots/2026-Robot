
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
import com.pathplanner.lib.events.EventTrigger;

// Import WPILib Librarires
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.filter.SlewRateLimiter;

// Import Custom TunerConstants
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Utils.JoystickScaler;
// Import subystems
import frc.robot.Actors.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;
import frc.robot.Actors.Subsystems.Elevator.ElevatorLift;
import frc.robot.Actors.Subsystems.Elevator.ElevatorHook;
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;

// Import Custom Commands
import frc.robot.Commands.Intake.IntakeSequence;
import frc.robot.Commands.Spindexer.SpinAndFeedCommand;
import frc.robot.Commands.Elevator.ClimbSequenceL1;
import frc.robot.Commands.Elevator.ClimbSequenceL3;
import frc.robot.Commands.Elevator.HookCommand;
import frc.robot.Commands.Elevator.LiftCommand;
import frc.robot.Commands.Shooter.AimAtHubCommand;



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

    private double xVelAvg = 0.0;
    private double yVelAvg = 0.0;
    private double hVelAvg = 0.0;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Spindexer spindexer = new Spindexer();
    public final Transfer transfer = new Transfer();

    public final Intake intake = new Intake();
    public final IntakeExtension intakeExtension = new IntakeExtension();
    private final ElevatorLift elevatorLift = new ElevatorLift();
    private final ElevatorHook elevatorHook = new ElevatorHook();
    private final Shooter shooter = new Shooter();
    private final Turret turret = new Turret();

      /* Path follower */
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        //TODO: Make sure values for Commands are correct
         // Register Named Commands within Pathplanner
        NamedCommands.registerCommand("ShootName",
            new SpinAndFeedCommand(
                transfer, spindexer, SpindexerConstants.transferRPS, SpindexerConstants.spindexerRPS , SpindexerConstants.spinupTime
            ).alongWith(Commands.print("Shooting")));
        //NamedCommands.registerCommand("Intake", new IntakeSequence(intake, intakeExtension).alongWith(Commands.print("Intaking (Named)")));
        NamedCommands.registerCommand("L1Climb", new ClimbSequenceL1(elevatorLift).alongWith(Commands.print("Climbing")));
        NamedCommands.registerCommand("Intake", new IntakeSequence(intake, intakeExtension));

        // Register Event Triggers within Pathplanner
        new EventTrigger("Intake").onTrue(new IntakeSequence(intake, intakeExtension).alongWith(Commands.print("Intaking (Trigger)")));
        //new EventTrigger("Intake").onTrue(Commands.print("Intaking (Trigger)"));
        new EventTrigger("Shoot").onTrue(
            new SpinAndFeedCommand(
                transfer, spindexer, SpindexerConstants.transferRPS, SpindexerConstants.spindexerRPS, SpindexerConstants.spinupTime
            ).alongWith(Commands.print("Shooting (Trigger)")));
        new EventTrigger("L1Climb").onTrue(new ClimbSequenceL1(elevatorLift));
        
        // TODO: Set default auto
        autoChooser = AutoBuilder.buildAutoChooser("Awesome");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

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
            drivetrain.applyRequest(new Supplier<SwerveRequest>() {
                public SwerveRequest get() {
                    double inputX = joystick.getLeftY();
                    double inputY = joystick.getLeftX();
                    double inputH = joystick.getRightX();
                    xVelAvg = xVelAvg+(inputX*0.1)/1.1;
                    yVelAvg = yVelAvg+(inputY*0.1)/1.1;
                    hVelAvg = hVelAvg+(inputH*0.1)/1.1;
                    if (drivetrain.getPose().getX() < 4.25 && joystick.getRightTriggerAxis() > 0.4) {
                        inputX = xVelAvg;
                        inputY = yVelAvg;
                        inputH = hVelAvg;
                    }
                    return drive.withVelocityX(-JoystickScaler.scaleStrafe(inputX) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-JoystickScaler.scaleStrafe(inputY) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-JoystickScaler.scaleRotate(inputH) * MaxAngularRate); // Drive counterclockwise with negative X (left)

                }
            })
        );

        joystick.rightTrigger(0.4).onTrue(new FunctionalCommand(null, () -> {
            xVelAvg = joystick.getLeftY();
            yVelAvg = joystick.getLeftX();
            hVelAvg = joystick.getRightX();
        }, null, null, new Subsystem[0]));

        shooter.setDefaultCommand(new AimAtHubCommand(shooter, turret, drivetrain::getPose, () -> {
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                state.Speeds,
                state.Pose.getRotation()
            );
        }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Break when pressing A
        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Orientate wheels when pressing B and and moving left and right joysticks
        /*joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.povLeft().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // TODO: Enable logger
        // drivetrain.registerTelemetry(logger::telemeterize);
        
        joystick.leftBumper().toggleOnTrue(new IntakeSequence(intake, intakeExtension));
        drivetrain.registerTelemetry(logger::telemeterize);
        joystick.rightTrigger(0.4).whileTrue(new SpinAndFeedCommand(transfer, spindexer, SpindexerConstants.transferRPS, SpindexerConstants.spindexerRPS , SpindexerConstants.spinupTime));  
        // Descend from Auto L1 + Retract Lift down
        joystick.x().whileTrue(new ClimbSequenceL3(elevatorLift, elevatorHook));
        joystick.y().whileTrue(new LiftCommand(elevatorLift, joystick));
        joystick.a().whileTrue(new HookCommand(elevatorHook, joystick));
        joystick.b().whileTrue(new ClimbSequenceL1(elevatorLift));
        
       
        /*************************************************
         * Commands for Spindexer Testing
         *************************************************/

    //     joystick.x().whileTrue(new SpinAndFeedCommand(
    //         transfer, spindexer, 30, 10, 0.5
    //     ));

    //     joystick.y().whileTrue(new SpinFuelCommand(spindexer, 10));
    }


    public Command getAutonomousCommand() {
         /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
        // /* Run the path selected from the auto chooser */
        // return Commands.print("No autonomous command configured");
    }
}