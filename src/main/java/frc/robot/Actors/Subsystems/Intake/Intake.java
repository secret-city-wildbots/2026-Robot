package frc.robot.Actors.Subsystems.Intake;

import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class Intake extends SubsystemBase {

     private Motor motor;

    public Intake() {
        this.motor = new Motor(IntakeConstants.intakeMotorID, MotorType.TFX, "rio");
    }

    // Motor Controls

    /**
     * Sets IntakeMotor motor output (-1.0 to 1.0)
     */

    public void set(double percent) {
        motor.dc(percent);
    }
}