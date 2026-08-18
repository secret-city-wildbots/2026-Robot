
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
// Import Phoenix6 Libraries
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
// Import Path Planner Libraries
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

// Import WPILib Librarires
import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// Import Custom TunerConstants
import frc.robot.generated.TunerConstants;
import frc.robot.Utils.JoystickScaler;
import frc.robot.Constants.IntakeConstants;
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
import frc.robot.Commands.Shooter.SimpleShootCommand;
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
    private final Consumer<Command> autoChosen = (Command newAuto) -> {
        this.auto = newAuto;
        this.dashboardArmed = true;
    };

    /*
     *=========================================================================
     * AUTO BYPASS  —  currently INACTIVE
     *=========================================================================
     * Normally the auto is armed from the dashboard's "Autos" tab, which
     * validates that it will actually load before letting you pick it. If the
     * dashboard is unavailable - browser closed, laptop dead, WildBoard not
     * serving - there is otherwise no way to choose an auto and the robot sits
     * still for the whole autonomous period.
     *
     * The bypass publishes a plain SendableChooser to NetworkTables as
     * "Auto Bypass", so Glass / Elastic / SmartDashboard can select one.
     *
     *     >>> TO ENABLE: uncomment the ONE line marked BYPASS  <<<
     *     >>> in getAutonomousCommand() at the bottom of this file. <<<
     *
     * Everything else here is already live. The chooser is built and published
     * at startup so it is visible and selectable in NetworkTables even while
     * the bypass is inactive - it has to be, or it would not appear in time to
     * pick from. Until you uncomment that line its selection is simply
     * ignored, and the dashboard stays the only source of the auto.
     *
     * Once enabled the dashboard still wins whenever it has armed something;
     * the chooser is consulted only when it has not.
     *
     * NOTE: buildAutoChooser() lists every auto in the deploy folder,
     * including the ones the analysis flags as unloadable. Selecting one of
     * those will throw when autonomous starts - the Autos tab refuses them, a
     * plain SendableChooser cannot. Check the Autos tab first.
     */
    private SendableChooser<Command> autoBypass = null;
    private boolean dashboardArmed = false;
    
    public RobotContainer() {
        dashboard = new Dashboard(drivetrain, shooter, indexer, transfer, turret, intake, intakeExtension, pdh, autoChosen);

        shotSmoothingx = new SlewRateLimiter(4.0);
        shotSmoothingy = new SlewRateLimiter(4.0);
        shotSmoothingh = new SlewRateLimiter(7.0);
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
        }, indexer, transfer, shooter, turret));
        new EventTrigger("Intake").onTrue(new AutoIntakeExtend(intake, intakeExtension));
        new EventTrigger("IntakeRetract").onTrue(new AutoIntakeRetract(intake, intakeExtension));
        //new EventTrigger("Intake").onTrue(Commands.print("Intaking (Trigger)"));
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
                        .withRotationalRate(shotSmoothingh.calculate(-JoystickScaler.scaleStrafe(inputH) * MaxAngularRate * 0.5)); // Drive counterclockwise with negative X (left)
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
        /*joystick.povUp().onTrue(Commands.runOnce(SignalLogger::start));
        joystick.povDown().onFalse(Commands.runOnce(SignalLogger::stop));
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/
        
        // reset the field-centric heading on left bumper press
        joystick.povLeft().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //joystick.povUp().toggleOnTrue(new LockTurret(turret));

        //joystick.rightBumper().whileTrue(Commands.startEnd(() -> {MaxSpeed/=2.0;}, () -> {MaxSpeed*=2.0;}));

        // TODO: Enable logger
        //drivetrain.registerTelemetry(logger::telemeterize);
        joystick.leftBumper().toggleOnTrue(new IntakeSequence(intake, intakeExtension));

        joystick.leftTrigger(0.4).whileTrue(Commands.runEnd(
            () -> {
                if (intakeExtension.getPos() > 50) {
                    intakeExtension.setIntakePos(IntakeConstants.jostleDegree);
                }
            },
            () -> {
                if (intakeExtension.getPos() > 50) {
                    intakeExtension.setIntakePos(IntakeConstants.maxDegree);
                }
            }
        ));

        joystick.rightTrigger(0.4).onTrue(Commands.runOnce(() -> {
            Robot.shooterEnabled = true;
        }));
        joystick.rightTrigger(0.4).onFalse(Commands.runOnce(() -> {
            Robot.shooterEnabled = false;
        }));
        joystick.rightTrigger(0.4).whileTrue(new ParallelCommandGroup(new AimAndShootCommand(drivetrain::getPose, () -> { //?
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                state.Speeds,
                state.Pose.getRotation()
            );
        }, indexer, transfer, shooter, turret)
        /*,new RepeatCommand(
            new SequentialCommandGroup(
                Commands.runOnce(() -> intakeExtension.setIntakePos(IntakeConstants.maxDegree)),
                new WaitCommand(1),
                Commands.runOnce(() -> intakeExtension.setIntakePos(IntakeConstants.minDegree)),
                new WaitCommand(1)
            )
        )*/));  

    //3,14
    //5.14

        joystick.rightBumper().and(joystick.y()).whileTrue(new SimpleAimAndShootCommand(indexer, transfer, shooter, turret,
            90-(Math.pow(0.475086, 1-4.67884)+62+(-1.37205*1)),
            (1.456*(1-2.0) + 50),
            new Rotation2d()
        ));

        joystick.rightBumper().and(joystick.a()).whileTrue(new SimpleAimAndShootCommand(indexer, transfer, shooter, turret,
            90-(Math.pow(0.475086, 3.14-4.67884)+62+(-1.37205*3.14)),
            Math.pow(Math.E,0.565613*(3.14-0.832416)) + 48.5,
            new Rotation2d()
        ));

        joystick.rightBumper().and(joystick.b()).whileTrue(new SimpleAimAndShootCommand(indexer, transfer, shooter, turret,
            90-(Math.pow(0.475086, 5.14-4.67884)+62+(-1.37205*5.14)),
            Math.pow(Math.E,0.565613*(5.14-0.832416)) + 48.8,
            new Rotation2d(Math.PI * 0.23)
        ));

        joystick.rightBumper().and(joystick.x()).whileTrue(new SimpleAimAndShootCommand(indexer, transfer, shooter, turret,
            90-(Math.pow(0.475086, 5.14-4.67884)+62+(-1.37205*5.14)),
            Math.pow(Math.E,0.565613*(5.14-0.832416)) + 48.8,
            new Rotation2d(-Math.PI * 0.23)
        ));
        
        joystick.a().and(joystick.rightBumper().negate()).whileTrue(
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
        // Was: auto = AutoBuilder.buildAutoChooser("L Trench 2 Dip").getSelected();
        // Removed because it did not do what it looks like it does:
        //   - the SendableChooser it built was never published anywhere, so
        //     nothing could ever select from it;
        //   - it called getSelected() immediately, one statement after
        //     construction, so it only ever returned the default;
        //   - "L Trench 2 Dip" no longer exists (renamed to "LT-2Dip"), so
        //     even the default did not match and it returned Commands.none();
        //   - running last in configureBindings(), it silently overwrote the
        //     WaitCommand(5.0) fallback set earlier in the constructor.
        // The auto now comes from the dashboard's Autos tab, or from the
        // bypass chooser below if you enable it.

        // Build and publish the bypass chooser. This happens whether or not the
        // bypass is active, because a chooser that appears only after you flip
        // it on would not be selectable in time to matter. See the AUTO BYPASS
        // block near the top of this file.
        //
        // buildAutoChooser() constructs every auto in the deploy folder, so one
        // unloadable auto would otherwise take out the whole robot program at
        // startup. Losing the bypass is survivable; failing to boot is not.
        try {
            autoBypass = AutoBuilder.buildAutoChooser("LT-2Dip");
            SmartDashboard.putData("Auto Bypass", autoBypass);
        } catch (Exception e) {
            autoBypass = null;
            DriverStation.reportError(
                    "Auto Bypass chooser could not be built (the Autos tab still works): " + e,
                    false);
        }
    }


    public Command getAutonomousCommand() {
        /*
         * Run the auto armed in the dashboard's "Autos" tab.
         *
         * If nothing has been armed, and no bypass below is enabled, this
         * returns the WaitCommand(5.0) set in the constructor — the robot does
         * nothing for 5 seconds.
         *
         *=====================================================================
         * BYPASS — two ways to choose an auto without the dashboard.
         * Uncomment ONE. Either only applies when the dashboard has NOT armed
         * anything, so the Autos tab always wins when it is available.
         *=====================================================================
         *
         * A) TYPE THE NAME HERE. Use this if you are not running Glass or
         *    Elastic. Put the auto name between the quotes. It must match a
         *    file in src/main/deploy/pathplanner/autos/ exactly, without the
         *    .auto extension — for example "LT-2Dip" or "RT-2Dip-Outpost".
         *    The Autos tab lists them, or run tools\audit.bat.
         */
         // if (!dashboardArmed) return new PathPlannerAuto("LT-T-B"); TODO: AUTO BYPASS

        /*
         * B) PICK IT FROM A DROPDOWN. Nothing to type. The chooser is already
         *    published to NetworkTables at startup as "Auto Bypass", so it
         *    shows up in Glass / Elastic / SmartDashboard whether or not this
         *    line is uncommented. Open one of those, find "Auto Bypass", and
         *    select from the list.
         *
         *    In Glass:    NetworkTables → SmartDashboard → Auto Bypass
         *    In Elastic:  add a "ComboBox Chooser" widget bound to Auto Bypass
         */
        if (Robot.noTags) {
            return new SequentialCommandGroup(
                Commands.runOnce(() -> drivetrain.resetPose(new Pose2d(3.59, 7.61, new Rotation2d(0))), drivetrain),
                new PathPlannerAuto("Houston")
            );
        } else {
            if (!dashboardArmed) return bypassAuto();

            return auto;
        }
    }

    /**
     * The auto selected in the NetworkTables "Auto Bypass" chooser, or the
     * normal auto if the chooser is unavailable or has nothing selected.
     *
     * <p>Only reached when the BYPASS line in {@link #getAutonomousCommand()}
     * is uncommented.
     */
    private Command bypassAuto() {
        if (autoBypass == null) {
            DriverStation.reportError(
                    "Auto bypass is enabled but the chooser failed to build; "
                            + "falling back to the dashboard auto", false);
            return auto;
        }
        Command picked = autoBypass.getSelected();
        if (picked == null) {
            DriverStation.reportWarning(
                    "Auto bypass is enabled but nothing is selected in \"Auto Bypass\"", false);
            return auto;
        }
        DriverStation.reportWarning(
                "Auto came from the BYPASS chooser, not the dashboard", false);
        return picked;
    }
}