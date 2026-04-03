
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import Phoenix6 Libraries
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// Import Path Planner Libraries
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

// Import WPILib Librarires
import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

// Import Custom TunerConstants
import frc.robot.generated.TunerConstants;
import frc.robot.Utils.JoystickScaler;
// Import subystems
import frc.robot.Actors.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import frc.robot.Commands.Intake.AutoIntakeExtend;
import frc.robot.Commands.Intake.AutoIntakeRetract;
import frc.robot.Commands.Intake.ExpandHopperCommand;
// Import Custom Commands
import frc.robot.Commands.Intake.IntakeSequence;
import frc.robot.Commands.Spindexer.AutoStartIndexCommand;
import frc.robot.Commands.Spindexer.AutoStopIndexCommand;
import frc.robot.Commands.Shooter.AimAndShootCommand;
import frc.robot.Commands.Shooter.AimAtHubCommand;
import frc.robot.Commands.Shooter.FreeAim;

public class RobotContainer {
    // TODO: Set max speed back to normal
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxSpeed = (Robot.test) ? 1.0:TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed //?

    // TODO: Set max rotation back to normal
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController sadstick1 = new CommandXboxController(1);
    private final CommandXboxController sadstick2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Spindexer spindexer = new Spindexer();
    public final Transfer transfer = new Transfer();

    public final Intake intake = new Intake();
    public final IntakeExtension intakeExtension = new IntakeExtension();
    //private final ElevatorLift elevatorLift = new ElevatorLift();
    //private final ElevatorHook elevatorHook = new ElevatorHook();
    private final Shooter shooter = new Shooter(); 
    private final Turret turret = new Turret();

    private final PowerDistribution pdh = new PowerDistribution();

    public final Dashboard dashboard;

      /* Path follower */
    private Command auto;
    private final Consumer<Command> autoChosen = (Command newAuto) -> {this.auto = newAuto;};
    
    public RobotContainer() {
        dashboard = new Dashboard(drivetrain, shooter, spindexer, transfer, turret, intake, intakeExtension, pdh, autoChosen);

        //TODO: Make sure values for Commands are correct
         //Register Named Commands within Pathplanner
        NamedCommands.registerCommand("Shoot",
            new AutoStartIndexCommand(
                transfer, spindexer
            ).alongWith(Commands.print("Shooting Start (Named)")));
        NamedCommands.registerCommand("ShootStop",
            new AutoStopIndexCommand(
                transfer, spindexer
            ).alongWith(Commands.print("Shooting Stop (Named)")));
        NamedCommands.registerCommand("Intake", new IntakeSequence(intake, intakeExtension).alongWith(Commands.print("Intaking (Named)")));
        
        //NamedCommands.registerCommand("L1Climb", new ClimbSequenceL1(elevatorLift).alongWith(Commands.print("Climbing")));
        NamedCommands.registerCommand("Intake", new AutoIntakeExtend(intake, intakeExtension));
        NamedCommands.registerCommand("AutoAim", new AimAtHubCommand(shooter, turret, drivetrain::getPose, () -> {
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                state.Speeds,
                state.Pose.getRotation()
            );
        }));
        NamedCommands.registerCommand("AimAndShoot", new AimAndShootCommand(drivetrain::getPose, () -> { //?
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                state.Speeds,
                state.Pose.getRotation()
            );
        }, spindexer, transfer, shooter, turret));
        auto = new WaitCommand(5.0); //?

        // Register Event Triggers within Pathplanner
        new EventTrigger("ExpandHopper").onTrue( new ExpandHopperCommand(intake, intakeExtension));
        new EventTrigger("AutoAim").onTrue( new AimAtHubCommand(shooter, turret, drivetrain::getPose, () -> {
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                state.Speeds,
                state.Pose.getRotation()
            );
        }));
        new EventTrigger("AimAndShoot").onTrue(new AimAndShootCommand(drivetrain::getPose, () -> { //?
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                state.Speeds,
                state.Pose.getRotation()
            );
        }, spindexer, transfer, shooter, turret));
        new EventTrigger("Intake").onTrue(new AutoIntakeExtend(intake, intakeExtension));
        new EventTrigger("IntakeRetract").onTrue(new AutoIntakeRetract(intake, intakeExtension));
        //new EventTrigger("Intake").onTrue(Commands.print("Intaking (Trigger)"));
        new EventTrigger("Shoot").onTrue(
        new AutoStartIndexCommand(transfer, spindexer).alongWith(
        Commands.print("Shooting Start (Trigger)")));
        new EventTrigger("ShootStop").onTrue(
        new AutoStopIndexCommand(transfer, spindexer).alongWith(
        Commands.print("Shooting Stop (Trigger)")));

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(new Supplier<SwerveRequest>() {
                public SwerveRequest get() {
                    CommandXboxController stick = (Robot.sad) ? sadstick1:joystick;
                    double inputX = stick.getLeftY();
                    double inputY = stick.getLeftX();
                    double inputH = stick.getRightX();
                    return drive.withVelocityX(-JoystickScaler.scaleStrafe(inputX) * ((Robot.sad) ? 1.0:MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(-JoystickScaler.scaleStrafe(inputY) * ((Robot.sad) ? 1.0:MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(-JoystickScaler.scaleRotate(inputH) * ((Robot.sad) ? 2.0:MaxAngularRate)); // Drive counterclockwise with negative X (left)
                }
            })
        );

        shooter.setDefaultCommand(new FreeAim(shooter, turret, drivetrain::getPose, sadstick2));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.button(7).onTrue(Commands.runOnce(() -> {
            Robot.sad = !Robot.sad;
        }));

        joystick.povLeft().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        joystick.leftBumper().toggleOnTrue(new IntakeSequence(intake, intakeExtension));
    }


    public Command getAutonomousCommand() {
         /* Run the path selected from the auto chooser */
        return auto;
        // /* Run the path selected from the auto chooser */
        //return Commands.print("No autonomous command configured");
    }
}