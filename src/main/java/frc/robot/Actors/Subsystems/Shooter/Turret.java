package frc.robot.Actors.Subsystems.Shooter;

import frc.robot.Actors.Motor;
import frc.robot.Constants.TurretConstants;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    // Real Motor Variables
    private Motor motor;
    private CANcoder absEncoder;

    public Turret() {
        this.motor = new Motor(TurretConstants.turretMotorID, MotorType.TFX);
        this.motor.motorConfig.direction = RotationDir.CounterClockwise;
        this.motor.motorConfig.peakForwardDC = 0.25;
        this.motor.motorConfig.peakReverseDC = -0.25;

        this.motor.configTFX.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANifier;
        this.motor.configTFX.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 40;
        this.motor.configTFX.HardwareLimitSwitch.ForwardLimitEnable = true;
        this.motor.configTFX.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;

        // this.motor.configTFX.ClosedLoopGeneral.ContinuousWrap = true;
        // this.motor.configTFX.Feedback.FeedbackRemoteSensorID =
        // TurretConstants.encoderID;
        // this.motor.configTFX.Feedback.FeedbackSensorSource =
        // FeedbackSensorSourceValue.RemoteCANcoder;
        // this.motor.configTFX.Feedback.RotorToSensorRatio =
        // TurretConstants.turretGearRatio;
        // this.motor.configTFX.Feedback.SensorToMechanismRatio =
        // TurretConstants.turretGearRatio;

        this.motor.applyConfig();
        this.motor.motionMagic(0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 4.0);
    }

    public void dc(double dc) {
        this.motor.dc(dc);
    }

    public void zero() {
        this.motor.motorTFX.setPosition(0);
    }

    public boolean beambreakActive() {
        return this.motor.motorTFX.getForwardLimit().getValue().equals(ForwardLimitValue.Open);
    }

    public void setForwardLimit(boolean state) {
        this.motor.configTFX.HardwareLimitSwitch.ForwardLimitEnable = state;
        this.motor.applyConfig();
    } 

    public double getTurretDegrees() {
        return absEncoder.getPosition().getValueAsDouble() * 360.0; // Convert rotations to degrees
    }

    public Rotation2d getTurretAngle() {
        return Rotation2d.fromDegrees(getTurretDegrees());
    }

    public void setTargetAngle(Rotation2d angle) {
        System.out.println((Math.round(angle.getRotations() * 100) / 100.0) + " : "
                + (Math.round(motor.motorTFX.getPosition().getValueAsDouble() / TurretConstants.turretGearRatio * 100)
                        / 100.0)
                + " : " +
                (Math.round(motor.motorTFX.getClosedLoopOutput().getValueAsDouble() * 100) / 100.0));
        this.motor.posMM(angle.getRotations() * TurretConstants.turretGearRatio);
        SmartDashboard.putNumber("diff", Math.abs((angle.minus(new Rotation2d(
                motor.motorTFX.getPosition().getValueAsDouble() * 2 * Math.PI / TurretConstants.turretGearRatio)))
                .getDegrees()));
    }
}