package frc.robot.Actors.Subsystems.Elevator;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorHook extends SubsystemBase {
    
    private Motor motor;
    private CANcoder encoder;
    private double targetAngle;

    public ElevatorHook() {
        this.motor = new Motor(ElevatorConstants.hookMotorID, MotorType.TFX, "rio");
        this.encoder = new CANcoder(ElevatorConstants.hookMotorCancoderID);

        this.targetAngle = 0.0;
    }

    // Motor Controls

    /**
     * Sets ElevatorHook motor output (-1.0 to 1.0)
     */

    public void set(double percent) {
        // Clamp input percentage to proper range
        percent = MathUtil.clamp(percent, -1.0, 1.0);

        // Check to make sure the hooks are safe to extend out
        if (percent > 0.0 && getCurrentAngle() >= 90.0) {
            // if it is not safe, dont allow the motor to move
            motor.dc(0.0);
            return;
        }

        // check to make sure the hooks are safe to retract in
        if (percent < 0.0 && getCurrentAngle() <= 0.0) {
            // if it is not safe, dont allow the motor to move
            motor.dc(0.0);
            return;
        }
        
        motor.dc(percent);
    }

    /**
     * Sets the target angle for the hooks to travel to
     * @param angle
     */
    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }

    // Sensor Controls

    /**
     * Get the encoder position in degrees
     */
    public double getCurrentAngle() {
        return this.encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }
}
