package frc.robot;

import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Actors.Subsystems.Elevator.ElevatorHook;
import frc.robot.Actors.Subsystems.Elevator.ElevatorLift;
import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Intake.IntakeExtension;
import frc.robot.Actors.Subsystems.Shooter.Shooter;
import frc.robot.Actors.Subsystems.Shooter.Turret;
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;
import frc.robot.WildBoard.WildBoard;
import frc.robot.WildBoard.Panels.*;

public class Dashboard {
    public WildBoard dashboard;

    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Spindexer spindexer;
    private Transfer transfer;
    private Turret turret;
    private Intake intake;
    private IntakeExtension intakeExtension;
    private ElevatorHook elevatorHook;
    private ElevatorLift elevatorLift;
    private PowerDistribution pdh;

    final VelocitySimpleSubsystem WBshooter;
    final SimpleSubsystem WBintake;
    final VelocitySimpleSubsystem WBspindexer;
    final VelocitySimpleSubsystem WBtransfer;
    final SimpleSubsystem WBturret;
    final SimpleSubsystem WBhood;
    final SwerveModules WBswerveModules;
    final MasterAlarms WBalarms;

    public double battAvg = 12.0;
    public double currAvg = 50.0;

    private Consumer<Command> autoChosen;

    public Dashboard(CommandSwerveDrivetrain drivetrain, ElevatorHook elevatorHook, ElevatorLift elevatorLift, Shooter shooter, Spindexer spindexer, Transfer transfer, Turret turret, Intake intake, IntakeExtension intakeExtension, PowerDistribution pdh, Consumer<Command> autoChoosen) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.transfer = transfer;
        this.turret = turret;
        this.intake = intake;
        this.intakeExtension = intakeExtension;
        this.elevatorHook = elevatorHook;
        this.elevatorLift = elevatorLift;
        this.pdh = pdh;
        this.autoChosen = autoChoosen;
        dashboard = new WildBoard(5804);

        // Checklist
        dashboard.addTab(new Tab()
                .addChild(new Checklist())
                .setTitle("Checklist"));

        // TeleOp
        dashboard.addTab(new Tab()
                .setTitle("TeleOp")
                .addChild(new Col(2).addChild(
                        new FieldMap()))
                .addChild(new Col(6).addChild(
                        new Row().addChild(
                                new CameraFeed(5800)).addChild(
                                        new CameraFeed(5801)))
                        .addChild(
                                new Row().addChild(
                                        new CameraFeed(5802)).addChild(
                                                new CameraFeed(5803))))
                .addChild(new Col(4).addChild(
                        new AutoChooser(new String[] { "Go Forward", "Move Fast" }).onChange((String choice) -> {
                            System.out.println("Auto Chosen: "+choice);
                            autoChosen.accept(new PathPlannerAuto(choice));
                        })).addChild(
                                new Overrides(new String[] { "Limelight PowerSaver", "Disable Camera Feeds", "CompMode",
                                        "Disable Shot Smoothing", "Always Aim at Hub", "Disable Shoot Safeties" },
                                        2))));

        // Subsystems
        WBshooter = new VelocitySimpleSubsystem("Shooter");
        WBturret = new SimpleSubsystem("Turret", false);
        WBhood = new SimpleSubsystem("Turret Hood", true);
        WBintake = new SimpleSubsystem("Intake", true, "rps");
        WBtransfer = new VelocitySimpleSubsystem("Transfer");
        WBspindexer = new VelocitySimpleSubsystem("Spindexer");

        WBswerveModules = new SwerveModules();
        WBalarms = new MasterAlarms(
                new String[] { "SWOH", "SSOH", "CNBS", "CURR", "JOYS", "PING", "LOOP", "BATT", "blank", "blank" },
                new String[] { "Swerve Overheat", "Subsystem Overheat", "Canbus Error", "Current High",
                        "Joystick Disconnect", "Ping High/Failed", "Loop Time too High", "Battery Voltage Low",
                    "Placeholder for future issues", "Placeholder for future issues" },
                2);

        dashboard.addTab(new Tab()
                .addChild(
                        new Col(4).addChild(
                                WBswerveModules).addChild(
                                        new SystemsCheck().onTest(() -> {
                                            //TODO
                                            // drive in square

                                            // full climber squence

                                            // deploy intake
                                            // intake
                                            // retract intake
                                            // aim turret to 0
                                            // aim hood to 0
                                            // spin up shooter
                                            // spin up transfer
                                            // spin spindexer to shoot
                                            System.out.println("blah");
                                        })))
                .addChild(
                        new Col(3).addChild(
                                new Placeholder("Climb", 20)))
                .addChild(
                        new Col(5).addChild(
                                new Row().addChild(
                                        WBshooter).addChild(
                                                WBintake))
                                .addChild(
                                        new Row().addChild(
                                                WBtransfer).addChild(
                                                        WBspindexer))
                                .addChild(
                                        new Row().addChild(
                                                WBturret).addChild(
                                                        WBhood)))
                .setTitle("Subsystems"));

        dashboard.addPanel(new LooptimeMonitor());
        dashboard.addPanel(new PingMonitor());
        dashboard.addPanel(new FPSMonitor());
        dashboard.addPanel(WBalarms);
        dashboard.start();

        WBintake.onUnlock((Boolean locked) -> {
            System.out.println("intake " + (locked ? "locked" : "unlocked"));
            intakeExtension.setBrake(locked);
        });

        WBhood.onUnlock((Boolean locked) -> {
            System.out.println("hood " + (locked ? "locked" : "unlocked"));
            shooter.setBrake(locked);
        });

        WBturret.onUnlock((Boolean locked) -> {
            System.out.println("turret " + (locked ? "locked" : "unlocked"));
            turret.setBrake(locked);
        });

        WBturret.onCalib((Boolean pressed) -> {
            System.out.println("turret calib " + (pressed ? "pressed" : "released"));
            turret.zero();
        });
    }

    public void update() {
        WBshooter.updateVals(shooter.getRPS(), (shooter.getLeadTemp()+shooter.getFollowTemp())/2.0);
        WBturret.updateVals(turret.getTurretDegrees(), turret.getTemp());
        WBhood.updateVals(shooter.getPos(), shooter.getHoodTemp());
        WBintake.updateVals(intake.getVel(), intake.getTemp());
        WBspindexer.updateVals(spindexer.getRPS(), spindexer.getTemp());
        WBtransfer.updateVals(transfer.getRPS(), transfer.getTemp());

        double[] swerveAngles = new double[4];
        double[] swerveTemps = new double[4];
        double[] swerveVels = new double[4];
        SwerveModuleState[] states = drivetrain.getState().ModuleStates;
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = drivetrain.getModules();

        for (int i = 0; i < states.length; i++) {
            swerveAngles[i] = states[i].angle.getDegrees();
            swerveTemps[i] = drivetrain.getModules()[i].getDriveMotor().getDeviceTemp().getValueAsDouble();
            swerveVels[i] = states[i].speedMetersPerSecond;
        }

        WBswerveModules.updateVals(swerveAngles, swerveTemps, swerveVels);

        // Master Alarms update

        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : modules) {
            if (module.getDriveMotor().getDeviceTemp().getValueAsDouble() > 80) {
                WBalarms.triggerAlarm(0);
            }
        }

        double maxHeat = 70.0;
        if (
            shooter.getHoodTemp() > maxHeat || shooter.getLeadTemp() > maxHeat ||
            shooter.getFollowTemp() > maxHeat || turret.getTemp() > maxHeat ||
            intake.getTemp() > maxHeat || intakeExtension.getTemp() > maxHeat ||
            elevatorHook.getTemp() > maxHeat || elevatorLift.getTemp() > maxHeat ||
            spindexer.getTemp() > maxHeat || transfer.getTemp() > maxHeat
        ) {
            WBalarms.triggerAlarm(1);
        }

        CANStatus can = RobotController.getCANStatus();
        if (can.transmitErrorCount > 0 || can.receiveErrorCount > 0 || can.percentBusUtilization > 0.9) {
            WBalarms.triggerAlarm(2);
        }

        currAvg = (currAvg + (pdh.getTotalCurrent() * 0.1)) / 1.1;
        if (currAvg > 150.0) {
            WBalarms.triggerAlarm(3);
        }

        if (!DriverStation.getJoystickIsXbox(0)) {
            WBalarms.triggerAlarm(4);
        }

        // 5, ping, is handled by frontend

        if (WildBoard.loopTime_ms > 100) {
            WBalarms.triggerAlarm(6);
        }

        battAvg = (battAvg + (RobotController.getBatteryVoltage() * 0.1)) / 1.1;
        if (battAvg < 10.0) {
            WBalarms.triggerAlarm(7);
        }

        dashboard.update();
    }
}