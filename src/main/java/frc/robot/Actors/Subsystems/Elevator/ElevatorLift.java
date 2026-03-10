package frc.robot.Actors.Subsystems.Elevator;

// Import WPILib Libraries
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Phoenix 5 Libraries
import com.ctre.phoenix.CANifier;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorLift extends SubsystemBase {
    
    // Define variables
    private Motor motor;                                // Motor to control the elevator lift position
    private DigitalInput lowerLimitMagneticSwitch;      // Lower limit magnetic switch for the elevator lift
    // private DigitalInput handoffLimitMagneticSwitch; // Handoff limit magnetic switch for the elevator lift
    private CANifier handoffLimitSwitch;                // Handoff limit magnetic switch for the elevator lift
    private DigitalInput topLimitMagneticSwitch;        // Top limit magnetic switch for the elevator lift

    // code to rotate 30 motor rotations
    private double initMotorRotations = -999999.0;
    private double motorRotationsSinceTopLimitSwitch = 0.0;

    public ElevatorLift() {
        // Configure the elevator lift motor
        this.motor = new Motor(ElevatorConstants.liftMotorID, MotorType.TFX, "rio");

        // Configure the elevator magnetic switches
        this.lowerLimitMagneticSwitch = new DigitalInput(ElevatorConstants.lowerLimitMagneticSensorPort);
        // this.handoffLimitMagneticSwitch = new DigitalInput(ElevatorConstants.handoffMagneticSensorPort);
        this.handoffLimitSwitch = new CANifier(0);
        this.topLimitMagneticSwitch = new DigitalInput(ElevatorConstants.topLimitMagneticSensorPort);
    }

    // Motor Controls

    /**
     * Sets ElevatorLift motor output (-1.0 to 1.0)
     * @param percent
     */

    public void set(double percent) {
        // Clamp input percentage to proper range
        percent = MathUtil.clamp(percent, -1.0, 1.0);

        // Set initMotorRotations
        if (topLimitActive() && this.initMotorRotations == -999999.0) {
            // Set motor rotations
            this.initMotorRotations = motor.pos();
        }

        if (!topLimitActive()) {
            // Set motor rotations
            this.initMotorRotations = -999999.0;
        } else {
            this.motorRotationsSinceTopLimitSwitch = motor.pos();
        }

        // Check to make sure the elevator is safe to move up
        if (percent < 0.0 && topLimitActive() && Math.abs(Math.abs(this.motorRotationsSinceTopLimitSwitch) - Math.abs(this.initMotorRotations)) > 30.0) {
            // if it is not safe, dont allow the motor to move
            motor.dc(0.0);
            return;
        }

        // check to make sure the elevator is safe to move down
        if (percent > 0.0 && lowerLimitActive() && handoffLimitActive()) {
            // if it is not safe, dont allow the motor to move
            motor.dc(0.0);
            return;
        }

        // Send the output to the motor
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
        System.out.println("Can I rotate (30+ no):" + Math.abs(Math.abs(this.motorRotationsSinceTopLimitSwitch) - Math.abs(this.initMotorRotations)));
    }
}
