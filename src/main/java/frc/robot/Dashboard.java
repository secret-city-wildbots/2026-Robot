package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Actors.Subsystems.CommandSwerveDrivetrain;
import frc.robot.WildBoard.WildBoard;
import frc.robot.WildBoard.Panels.AutoChooser;
import frc.robot.WildBoard.Panels.CameraFeed;
import frc.robot.WildBoard.Panels.Checklist;
import frc.robot.WildBoard.Panels.Col;
import frc.robot.WildBoard.Panels.FPSMonitor;
import frc.robot.WildBoard.Panels.FieldMap;
import frc.robot.WildBoard.Panels.LooptimeMonitor;
import frc.robot.WildBoard.Panels.MasterAlarms;
import frc.robot.WildBoard.Panels.Overrides;
import frc.robot.WildBoard.Panels.PingMonitor;
import frc.robot.WildBoard.Panels.Placeholder;
import frc.robot.WildBoard.Panels.Row;
import frc.robot.WildBoard.Panels.SimpleSubsystem;
import frc.robot.WildBoard.Panels.SwerveModules;
import frc.robot.WildBoard.Panels.SystemsCheck;
import frc.robot.WildBoard.Panels.Tab;
import frc.robot.WildBoard.Panels.VelocitySimpleSubsystem;

public class Dashboard {
    public WildBoard dashboard;

    private CommandSwerveDrivetrain drivetrain;
    private PowerDistribution pdh;

    final VelocitySimpleSubsystem WBshooter;
    final VelocitySimpleSubsystem WBintake;
    final VelocitySimpleSubsystem WBspindexer;
    final VelocitySimpleSubsystem WBindexer;
    final SimpleSubsystem WBturret;
    final SimpleSubsystem WBhood;
    final SwerveModules WBswerveModules;
    final MasterAlarms WBalarms;

    public double[] temps = new double[] { 40, 40, 40, 40, 40, 40 };
    public double[] tempDirs = new double[] { 1, 1, 1, 1, 1, 1 };
    public double[] swervePoses = new double[] { 0, 0, 0, 0 };
    public double[] swervePosesDirs = new double[] { 0, 0, 0, 0 };
    public double[] swerveVels = new double[] { 0, 0, 0, 0 };
    public double[] swerveTemps = new double[] { 40, 40, 40, 40 };
    public double[] swerveTempsDirs = new double[] { 1, 1, 1, 1 };

    public double battAvg = 12.0;
    public double currAvg = 50.0;

    public Dashboard(CommandSwerveDrivetrain drivetrain, PowerDistribution pdh) {
        this.drivetrain = drivetrain;
        this.pdh = pdh;

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
                            System.out.println(choice);
                        })).addChild(
                                new Overrides(new String[] { "Limelight PowerSaver", "Disable Camera Feeds", "CompMode",
                                        "Disable Shot Smoothing", "Always Aim at Hub", "Disable Shoot Safeties" },
                                        2))));

        // Subsystems
        WBshooter = new VelocitySimpleSubsystem("Shooter");
        WBintake = new VelocitySimpleSubsystem("Intake");
        WBindexer = new VelocitySimpleSubsystem("Indexer");
        WBspindexer = new VelocitySimpleSubsystem("Spindexer");

        WBturret = new SimpleSubsystem("Turret", false);
        WBhood = new SimpleSubsystem("Turret Hood", true);

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
                                                WBindexer).addChild(
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
    }

    public void update() {
        WBshooter.updateVals(10 + Math.round(Math.random() * 5), temps[0]);
        WBintake.updateVals(8 + Math.round(Math.random() * 0.55), temps[1]);
        WBspindexer.updateVals(4 + Math.round(Math.random() * 0.6), temps[2]);
        WBindexer.updateVals(0, temps[3]);

        // fake temp vals
        for (int i = 0; i < temps.length; i++) {
            temps[i] += (Math.random() - 0.5) * 0.8 + tempDirs[i] * 0.2;
            if (temps[i] < 20) {
                temps[i] += 0.5;
                tempDirs[i] = 1;
            }
            if (temps[i] > 80) {
                temps[i] -= 0.5;
                tempDirs[i] = -1;
            }

            if (Math.random() < 0.1) {
                temps[i] += (Math.random() - 0.5) * 1.5;
            }

            if (Math.random() < 0.015) {
                tempDirs[i] = Math.random() * 2 - 1;
            }
        }

        WBturret.updateVals(289 + Math.round(Math.random() * 0.6) * 10, temps[4]);
        WBhood.updateVals(21 + Math.round(Math.random() * 0.6) * 2, temps[5]);

        for (int i = 0; i < swervePoses.length; i++) {
            swervePoses[i] += (Math.random() - 0.5) * 5 + swervePosesDirs[i] * 2;

            if (Math.random() < 0.05) {
                swervePosesDirs[i] = Math.random() * 2 - 1;
            }
            if (swervePoses[i] < 0) {
                swervePoses[i] += 360;
            }
            if (swervePoses[i] > 360) {
                swervePoses[i] -= 360;
            }

            swerveVels[i] += (Math.random() - 0.5) * 0.1;
            if (swerveVels[i] < 4) {
                swerveVels[i] += 0.02;
            }
            if (swerveVels[i] < 0) {
                swerveVels[i] = 0;
            }
            if (swerveVels[i] > 6) {
                swerveVels[i] -= 0.03;
            }
            if (Math.random() < 0.01 && swerveVels[i] > 2) {
                swerveVels[i] -= (Math.random()) * 2;
            }

            swerveTemps[i] += (Math.random() - 0.5) * 0.8 + swerveTempsDirs[i] * 0.2;
            if (swerveTemps[i] < 20) {
                swerveTemps[i] += 0.5;
                swerveTempsDirs[i] = 1;
            }
            if (swerveTemps[i] > 80) {
                swerveTemps[i] -= 0.5;
                swerveTempsDirs[i] = -1;
            }
            if (Math.random() < 0.1) {
                swerveTemps[i] += (Math.random() - 0.5) * 1.5;
            }
            if (Math.random() < 0.015) {
                swerveTempsDirs[i] = Math.random() * 2 - 1;
            }
        }

        WBswerveModules.updateVals(swervePoses, swerveTemps, swerveVels);

        WBturret.onUnlock((Boolean locked) -> {
            System.out.println("turret " + (locked ? "locked" : "unlocked"));
        });

        WBturret.onCalib((Boolean pressed) -> {
            System.out.println("turret calib " + (pressed ? "pressed" : "released"));
        });

        // Master Alarms update
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : drivetrain.getModules()) {
            if (module.getDriveMotor().getDeviceTemp().getValueAsDouble() > 80) {
                WBalarms.triggerAlarm(0);
            }
        }

        //TODO 1, need to detect subsystem overheats

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