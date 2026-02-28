package frc.robot.Actors.Subsystems;

import frc.robot.Actors.Motor;
import frc.robot.Constants.TurretConstants;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    // Real Motor Variables
    private Motor motor;
    private CANcoder absEncoder;

    public Turret() {
        this.motor = new Motor(TurretConstants.turretMotorID, MotorType.TFX);
        this.motor.motorConfig.direction = RotationDir.CounterClockwise;

        this.motor.configTFX.Feedback.FeedbackRemoteSensorID = TurretConstants.encoderID;
        this.motor.configTFX.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        this.motor.configTFX.Feedback.RotorToSensorRatio = TurretConstants.turretGearRatio;

        this.motor.applyConfig();
        this.motor.pid(0.0005, 0.0, 0.0);

        this.absEncoder = new CANcoder(TurretConstants.encoderID);
    }

    public double getTurretDegrees() {
        return absEncoder.getPosition().getValueAsDouble() * 360.0; // Convert rotations to degrees
    }

    public Rotation2d getTurretAngle() {
        return Rotation2d.fromDegrees(getTurretDegrees());
    }

    public void setTargetAngle(Rotation2d angle) {
        this.motor.pos(angle.getRotations());
    }
}
