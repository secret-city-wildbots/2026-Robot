package frc.robot.Actors.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

public class Indexer extends SubsystemBase {

    private Motor spindexer;
    private Motor transfer;

    public Indexer() {
        this.spindexer = new Motor(42, MotorType.TFX);
        this.transfer = new Motor(43, MotorType.TFX);

        this.spindexer.motorConfig.direction = RotationDir.CounterClockwise;
        this.transfer.motorConfig.direction = RotationDir.CounterClockwise;

        this.spindexer.motorConfig.brake = false;
        this.transfer.motorConfig.brake = false;

        this.spindexer.motorConfig.dutyCycleOpenLoopRampPeriod = 0.6;

        this.transfer.pid(0.01, 0.0, 0.0);

        this.spindexer.applyConfig();
        this.transfer.applyConfig();
    }

    public void shoot() {
        this.spindexer.dc(0.3);
        this.transfer.vel(30);
    }

    public void stop() {
        this.spindexer.dc(0.0);
        this.transfer.vel(0.0);
    }
}