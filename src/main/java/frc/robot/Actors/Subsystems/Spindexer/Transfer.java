package frc.robot.Actors.Subsystems.Spindexer;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import frc.robot.Constants.SpindexerConstants;

public class Transfer extends SubsystemBase {

    // Define variables
    public Motor motor; // Motor to control the transfer

    /**
     * Transfer Constructor
     */
    public Transfer() {
        // Configure the transfer motor
        this.motor = new Motor(SpindexerConstants.transferMotorID, MotorType.TFX, "rio");
        this.motor.motorConfig.direction = RotationDir.CounterClockwise;
        this.motor.motorConfig.dutyCycleClosedLoopRampPeriod = 0.3;
        this.motor.motorConfig.peakReverseDC = -0.3; //?
        this.motor.applyConfig();
        this.motor.pid(0.3, 0.0, 0.0); // Setup the transfer PID
    }

    public double getTemp() {
        return this.motor.getTemp();
    }

    // Motor Controls

    /**
     * Sets spindexer motor output (-1.0 to 1.0)
     * @param percent
     */
    public void set(double percent) {
        // Send the output to the motor
        motor.dc(percent);
    }

     /**
     * Set target RPS
     * @param rps
     */
    public void setRPS(double rps) {
        // Send the output to the motor
        motor.vel(rps);
    }

    /**
     * Stop motor
     */
    public void stop() {
        // Send the output to the motor
        motor.vel(0.0);
    }

    /**
     * Get current RPS from internal encoder
     */
    public double getRPS() {
        // Send the output to the motor
        return motor.vel();
    }
}