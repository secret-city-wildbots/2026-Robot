package frc.robot.Actors.Subsystems.Intake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;

public class Intake extends SubsystemBase {

    // Define variables
    public Motor motor; // Motor to control the intake position
    public Motor motorFollow; // Motor to control the intake position

    public Intake() {
        // Configure the intake motor
        this.motor = new Motor(IntakeConstants.intakeMotorID, MotorType.TFX, "rio");
        this.motorFollow = new Motor(60, MotorType.TFX, "rio");
        this.motor.motorConfig.direction = RotationDir.Clockwise;
        this.motor.motorConfig.brake = false;
        this.motor.applyConfig();
        this.motor.slot0TFX.kV = 0.012;
        this.motor.pid(0.05, 0.0, 0.0);
        this.motor.curlim.SupplyCurrentLimit = 25;
        this.motor.curlim.SupplyCurrentLimitEnable = true;
        this.motor.motorTFX.getConfigurator().apply(this.motor.curlim);
        this.motorFollow.motorTFX.setControl(new Follower(IntakeConstants.intakeMotorID, MotorAlignmentValue.Opposed));
        //0.05, 0.012
    }

    public double getTemp() {
        return this.motor.getTemp();
    }

    public double getVel() {
        return this.motor.vel();
    }

    // Motor Controls

    /**
     * Sets Intake motor output (-1.0 to 1.0)
     * @param percent
     */
    public void set(double percent) {
        // Send the output to the motor
        motor.dc(percent);
    }

    public void intake() {
        motor.vel(IntakeConstants.intake_rps);
    }
    public void stop() {
        motor.vel(0.0);
    }
}