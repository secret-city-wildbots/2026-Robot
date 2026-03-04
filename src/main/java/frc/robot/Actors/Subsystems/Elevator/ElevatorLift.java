package frc.robot.Actors.Subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.CANifier;

public class ElevatorLift extends SubsystemBase {
    
    private Motor motor;
    private DigitalInput lowerLimitMagneticSwitch;
    // private DigitalInput handoffLimitMagneticSwitch;
    private CANifier handoffLimitSwitch;
    private DigitalInput topLimitMagneticSwitch;

    public ElevatorLift() {
        this.motor = new Motor(ElevatorConstants.liftMotorID, MotorType.TFX, "rio");
        this.lowerLimitMagneticSwitch = new DigitalInput(ElevatorConstants.lowerLimitMagneticSensorPort);
        // this.handoffLimitMagneticSwitch = new DigitalInput(ElevatorConstants.handoffMagneticSensorPort);
        this.handoffLimitSwitch = new CANifier(0);
        this.topLimitMagneticSwitch = new DigitalInput(ElevatorConstants.topLimitMagneticSensorPort);
    }

    // Motor Controls

    /**
     * Sets ElevatorLift motor output (-1.0 to 1.0)
     */

    public void set(double percent) {
        // Clamp input percentage to proper range
        percent = MathUtil.clamp(percent, -1.0, 1.0);

        // Check to make sure the elevator is safe to move up
        if (percent < 0.0 && topLimitActive()) {
            // if it is not safe, dont allow the motor to move
            motor.dc(0.0);
            return;
        }

        // check to make sure the elevator is safe to move down
        if (percent > 0.0 && lowerLimitActive()) {
            // if it is not safe, dont allow the motor to move
            motor.dc(0.0);
            return;
        }

        motor.dc(percent);
    }

    // Sensor Controls

    /**
     * Get lower limit switch made status.
     * 
     * @return active status (sensor is made)
     */

     public boolean lowerLimitActive() {
        return  !this.lowerLimitMagneticSwitch.get();
     }

     /**
     * Get handoff limit switch made status.
     * 
     * @return active status (sensor is made)
     */

     public boolean handoffLimitActive() {
        // return  this.handoffLimitMagneticSwitch.get();
        return !this.handoffLimitSwitch.getGeneralInput(CANifier.GeneralPin.LIMF);
     }

     /**
     * Get top limit switch made status.
     * 
     * @return active status (sensor is made)
     */

     public boolean topLimitActive() {
        return !this.topLimitMagneticSwitch.get();
     }

    @Override
    public void periodic() {
        // TODO: put logic to send position states to dashboard
        System.out.println("--------------------------------------------------");
        System.out.println("Swith 0: " + lowerLimitActive());
        System.out.println("Swith 1: " + handoffLimitActive());
        System.out.println("Swith 2: " + topLimitActive()); 
    }
}
