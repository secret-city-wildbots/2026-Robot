package frc.robot.Actors.Subsystems.Intake;

import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class IntakeExtension extends SubsystemBase {

     private Motor motor;

    public IntakeExtension() {
        this.motor = new Motor(IntakeConstants.extensionMotorID, MotorType.TFX, "rio");
    }

    // Motor Controls

    /**
     * Sets IntakeExtensionMotor motor output (-1.0 to 1.0)
     */

    public void set(double percent) {
        motor.dc(percent);
    }
}