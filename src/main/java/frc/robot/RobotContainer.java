
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import Phoenix6 Libraries
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
// Import Path Planner Libraries
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

// Import WPILib Librarires
import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

// Import Custom TunerConstants
import frc.robot.generated.TunerConstants;
import frc.robot.Utils.JoystickScaler;
// Import subystems
import frc.robot.Actors.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;
import frc.robot.Actors.Subsystems.Indexer.Indexer;
import frc.robot.Actors.Subsystems.Indexer.Transfer;
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import frc.robot.Commands.Intake.AutoIntakeExtend;
import frc.robot.Commands.Intake.AutoIntakeRetract;
import frc.robot.Commands.Intake.AutoIntakeStop;
import frc.robot.Commands.Intake.ExtensionCommand;
// Import Custom Commands
import frc.robot.Commands.Intake.IntakeSequence;
import frc.robot.Commands.Indexer.AutoStartIndexCommand;
import frc.robot.Commands.Indexer.AutoStopIndexCommand;
import frc.robot.Commands.Indexer.ClearTransferCommand;
import frc.robot.Commands.Shooter.AimAndShootCommand;
import frc.robot.Commands.Shooter.AimAtHubCommand;
import frc.robot.Commands.Shooter.SimpleAimAndShootCommand;
import frc.robot.Commands.Turret.AimAtHubTurret;
import frc.robot.Commands.Turret.JoystickAimCommand;



public class RobotContainer {
    // TODO: Set max speed back to normal
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double topSpeed = (Robot.test) ? 1.0:TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed //?
    private double MaxSpeed = topSpeed;

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
    public final Indexer indexer = new Indexer();
    public final Transfer transfer = new Transfer();

    public final Intake intake = new Intake();
    public final IntakeExtension intakeExtension = new IntakeExtension();
    private final Shooter shooter = new Shooter(); 
    private final Turret turret = new Turret();

    private final PowerDistribution pdh = new PowerDistribution();

    public final Dashboard dashboard;

    public SlewRateLimiter shotSmoothingx;
    public SlewRateLimiter shotSmoothingy;
    public SlewRateLimiter shotSmoothingh;

      /* Path follower */
    private Command auto;
    private final Consumer<Command> autoChosen = (Command newAuto) -> {this.auto = newAuto;};
    
    public RobotContainer() {
        dashboard = new Dashboard(drivetrain, shooter, indexer, transfer, turret, intake, intakeExtension, pdh, autoChosen);

        shotSmoothingx = new SlewRateLimiter(1.0);
        shotSmoothingy = new SlewRateLimiter(1.0);
        shotSmoothingh = new SlewRateLimiter(2.0);
        //TODO: Make sure values for Commands are correct
         //Register Named Commands within Pathplanner
        // NamedCommands.registerCommand("Shoot",
        //     new AutoStartIndexCommand(
        //         transfer, indexer
        //     ).alongWith(Commands.print("Shooting Start (Named)")));
        // NamedCommands.registerCommand("ShootStop",
        //     new AutoStopIndexCommand(
        //         transfer, indexer
        //     ).alongWith(Commands.print("Shooting Stop (Named)")));
        // NamedCommands.registerCommand("Intake", new IntakeSequence(intake, intakeExtension).alongWith(Commands.print("Intaking (Named)")));
        
        // NamedCommands.registerCommand("Intake", new AutoIntakeExtend(intake, intakeExtension));
        // NamedCommands.registerCommand("NamedAimAndShoot", new AimAndShootCommand(drivetrain::getPose, () -> { //?
        //     var state = drivetrain.getState();
        //     return ChassisSpeeds.fromRobotRelativeSpeeds(
        //         state.Speeds,
        //         state.Pose.getRotation()
        //     );
        // }, indexer, transfer, shooter));
        auto = new WaitCommand(5.0); //?

        // Register Event Triggers within Pathplanner
        new EventTrigger("StopIntake").onTrue( new AutoIntakeStop(intake));
        new EventTrigger("AimAndShoot").toggleOnTrue(new AimAndShootCommand(drivetrain::getPose, () -> { //?
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                state.Speeds,
                state.Pose.getRotation()
            );
        }, indexer, transfer, shooter));
        new EventTrigger("Intake").onTrue(new AutoIntakeExtend(intake, intakeExtension));
        new EventTrigger("IntakeRetract").onTrue(new AutoIntakeRetract(intake, intakeExtension));
        new EventTrigger("Intake").onTrue(Commands.print("Intaking (Trigger)"));
        new EventTrigger("Shoot").onTrue(
        new AutoStartIndexCommand(transfer, indexer).alongWith(
        Commands.print("Shooting Start (Trigger)")));
        new EventTrigger("ShootStop").onTrue(
        new AutoStopIndexCommand(transfer, indexer).alongWith(
        Commands.print("Shooting Stop (Trigger)")));

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
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
                    if (joystick.getRightTriggerAxis() > 0.4 && dashboard.shotSmoothing) {
                        System.out.println("shot smoothing active");
                        return drive.withVelocityX(shotSmoothingx.calculate(-JoystickScaler.scaleStrafe(inputX) * MaxSpeed * 0.3)) // Drive forward with negative Y (forward)
                        .withVelocityY(shotSmoothingy.calculate(-JoystickScaler.scaleStrafe(inputY) * MaxSpeed * 0.3)) // Drive left with negative X (left)
                        .withRotationalRate(shotSmoothingh.calculate(-JoystickScaler.scaleStrafe(inputH) * MaxAngularRate * 0.3)); // Drive counterclockwise with negative X (left)
                    } else {
                        return drive.withVelocityX(-JoystickScaler.scaleStrafe(inputX) * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-JoystickScaler.scaleStrafe(inputY) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-JoystickScaler.scaleRotate(inputH) * MaxAngularRate); // Drive counterclockwise with negative X (left)
                    }
                }
            })
        );

        // joystick.rightTrigger(0.4).onTrue(Commands.runOnce(() -> {
        //     xVelAvg = joystick.getLeftY();
        //     yVelAvg = joystick.getLeftX();
        //     hVelAvg = joystick.getRightX();
        // }));

        //joystick.a().whileTrue(new Zero(turret));

        /*joystick.rightBumper().whileTrue(new AimAtHubCommand(shooter, turret, drivetrain::getPose, () -> { //?
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                state.Speeds,
                state.Pose.getRotation()
            );
        }));*/

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
        /*joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */
        // reset the field-centric heading on left bumper press
        joystick.povLeft().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //joystick.povUp().toggleOnTrue(new LockTurret(turret));

        //joystick.rightBumper().whileTrue(Commands.startEnd(() -> {MaxSpeed/=2.0;}, () -> {MaxSpeed*=2.0;}));

        // TODO: Enable logger
        //drivetrain.registerTelemetry(logger::telemeterize);
        
        joystick.leftBumper().toggleOnTrue(new IntakeSequence(intake, intakeExtension));

        joystick.rightTrigger(0.4).onTrue(Commands.runOnce(() -> {
            Robot.shooterEnabled = true;
        }));
        joystick.rightTrigger(0.4).onFalse(Commands.runOnce(() -> {
            Robot.shooterEnabled = false;
        }));
        joystick.rightTrigger(0.4).whileTrue(new AimAndShootCommand(drivetrain::getPose, () -> { //?
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                state.Speeds,
                state.Pose.getRotation()
            );
        }, indexer, transfer, shooter));  

        joystick.rightBumper().whileTrue(new SimpleAimAndShootCommand(indexer, transfer, shooter, turret));
        
        joystick.a().whileTrue(
            new ParallelCommandGroup(
                new ClearTransferCommand(transfer, indexer, intake, shooter),
                new ExtensionCommand(intakeExtension, 90.0) //?
            ));

        turret.setDefaultCommand(new AimAtHubTurret(turret));
        //joystick.x().whileTrue(new JoystickAimCommand(turret, joystick));
        /*joystick.rightTrigger(0.4).whileFalse(new ParallelRaceGroup( //?
            new ClearTransferCommand(transfer, indexer),
            new WaitCommand(0.5)
        ));*/

        //turret.setDefaultCommand(new JoystickAimCommand(turret, joystick));
       
        /*************************************************
         * Commands for Indexer Testing
         *************************************************/

    //     joystick.x().whileTrue(new SpinAndFeedCommand(
    //         transfer, indexer, 30, 10, 0.5
    //     ));

    //     joystick.y().whileTrue(new SpinFuelCommand(indexer, 10));
        auto = AutoBuilder.buildAutoChooser("Shoot 8 + Plow").getSelected(); //?
    }


    public Command getAutonomousCommand() {
         /* Run the path selected from the auto chooser */
        return auto;
        // /* Run the path selected from the auto chooser */
        //return Commands.print("No autonomous command configured");
    }
}