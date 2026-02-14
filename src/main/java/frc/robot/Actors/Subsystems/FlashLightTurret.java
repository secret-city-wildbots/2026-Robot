package frc.robot.Actors.Subsystems;

import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlashLightTurret extends SubsystemBase {

    // Real Motor Variables
    private Motor motor;
    private int moduleNumber;

    // Setting up Encoder
    private DutyCycleEncoder absEncoder;
    private int encoderChannel;

    // PID
    private final PIDController pid = new PIDController(0.02, 0.0005, 0.0);

    // State
    private Rotation2d targetAngle = Rotation2d.fromDegrees(20.0);

    public FlashLightTurret(int moduleNumber, int encoderChannel) {
        //Intialize module number+motor
        this.moduleNumber = moduleNumber;
        this.motor = new Motor(this.moduleNumber, MotorType.TFX, "drivetrain");
        this.motor.motorConfig.direction = RotationDir.CounterClockwise;
        this.motor.applyConfig();

        // Initialize the encoder
        this.encoderChannel = encoderChannel;
        this.absEncoder = new DutyCycleEncoder(this.encoderChannel, 1.0, 0.8847);

        // Initialize the PID
        pid.enableContinuousInput(0.0, 360.0);
        pid.setTolerance(1.0); // degrees
    }

    public double getTurretDegrees() {
        return absEncoder.get()*360;
    }

    public Rotation2d getTurretAngle() {
        return Rotation2d.fromDegrees(getTurretDegrees());
    }

    public void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }

    public void calculateTargetAngle() {

    }

    @Override
    public void periodic() {

        double currentDeg = getTurretDegrees();
        double targetDeg  = targetAngle.getDegrees();
        // double targetDeg;
        // if (targetAngle.getDegrees() < 0) {
        //     targetDeg = MathUtil.clamp(360.0 + targetAngle.getDegrees(), 0.0, 359.99999
        //     );
        // } else {
        //     targetDeg = MathUtil.clamp(targetAngle.getDegrees(), 0.0, 359.99999);
        // }

        double output = pid.calculate(currentDeg, targetDeg);


        if (pid.atSetpoint()) {
            motor.dc(0.0);
            // System.out.println(
            //     "Current=" + currentDeg +
            //     " Target=" + targetDeg +
            //     " Error=" + pid.getPositionError() +
            //     " Output=" + output
            // );
            return;
        }

        // Minimum output to overcome friction
        if (Math.abs(output) < 0.1) {
            output = Math.copySign(0.1, output);
        }

        // System.out.println(
        //     "Current=" + currentDeg +
        //     " Target=" + targetDeg +
        //     " Error=" + pid.getPositionError() +
        //     " Output=" + output
        // );

        output = MathUtil.clamp(output, -1, 1);
        motor.dc(output);
    }
}
