package frc.robot.Actors.Subsystems.Spindexer;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    
    // Define variables
    public Motor motor; // Motor to control the spindexer

    /**
     * Spindexer Constructor
     */
    public Spindexer() {
        // Configure the spindexer motor
        this.motor = new Motor(SpindexerConstants.spinMotorID, MotorType.TFX, "rio");
        this.motor.motorConfig.direction = RotationDir.CounterClockwise;
        this.motor.motorConfig.dutyCycleClosedLoopRampPeriod = 0.3;
        this.motor.motorConfig.peakReverseDC = -0.1; //?
        this.motor.applyConfig();
        this.motor.pid(0.3, 0.0, 0.0); // Setup the spindexer PID
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
        // Return the motor velocity in RPS
        return motor.vel();
    }
}