package frc.robot.Actors.Subsystems.Indexer;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    
    // Define variables
    public Motor motor; // Motor to control the indexer
    public Motor motor2; // Motor to control the roller bed

    /**
     * Indexer Constructor
     */
    public Indexer() {
        // Configure the indexer motor
        this.motor = new Motor(IndexerConstants.spinMotorID, MotorType.TFX, "rio");
        this.motor2 = new Motor(IndexerConstants.rollerMotorID, MotorType.TFX, "rio");
        this.motor.motorConfig.direction = RotationDir.CounterClockwise;
        this.motor2.motorConfig.direction = RotationDir.Clockwise; //TODO: Find out which way roller bed needs to go
        this.motor.motorConfig.dutyCycleClosedLoopRampPeriod = 0.3;
        this.motor2.motorConfig.dutyCycleClosedLoopRampPeriod = 0.3;
        this.motor.motorConfig.peakReverseDC = -0.1; //?
        this.motor2.motorConfig.peakReverseDC = -0.1;
        this.motor.applyConfig();
        this.motor2.applyConfig();
        this.motor.pid(0.3, 0.0, 0.0); // Setup the indexer PID
        this.motor2.pid(0.1, 0.0, 0.0); // Setup the roller bed PID
    }

    public double getTemp1() {
        return this.motor.getTemp();
    }

    public double getTemp2() {
        return this.motor2.getTemp();
    }

    // Motor Controls

    /**
     * Sets indexer motor output (-1.0 - 1.0)
     * Sets roller bed motor output (-1.0 - 1.0)
     * @param percent
     */
    public void set(double indexerPercent, double rollerPercent) {
        // Send the output to the motor
        motor.dc(indexerPercent);
        motor2.dc(rollerPercent);
    }

    /**
     * Set target RPS for indexer & roller bed
     * @param rps
     */
    public void setRPS(double indexerRPS, double rollerRPS) {
        // Send the output to the motor
        motor.vel(indexerRPS);
        motor2.vel(rollerRPS);
    }

    /**
     * Stops both indexer and roller bed
     */
    public void stop() {
        // Send the output to the motor
        motor.vel(0.0);
        motor2.vel(0.0);
    }

    /**
     * Get current indexer RPS from internal encoder
     */
    public double getRPS1() {
        // Return the motor velocity in RPS
        return motor.vel();
    }
    
     /**
     * Get current roller bed RPS from internal encoder
     */
    public double getRPS2() {
        // Return the motor2 velocity in RPS
        return motor2.vel();
    }
}