package frc.robot.Actors.Subsystems.Indexer;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
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
    public Timer stallTimer;
    public boolean isStalled = false;

    /**
     * Indexer Constructor
     */
    public Indexer() {
        // Configure the indexer motor
        this.motor = new Motor(IndexerConstants.spinMotorID, MotorType.TFX, "rio");
        this.motor2 = new Motor(IndexerConstants.rollerMotorID, MotorType.TFX, "rio");
        this.motor.motorConfig.direction = RotationDir.Clockwise;
        this.motor2.motorConfig.direction = RotationDir.Clockwise; //TODO: Find out which way roller bed needs to go
        this.motor.motorConfig.dutyCycleClosedLoopRampPeriod = 0.3;
        this.motor2.motorConfig.dutyCycleClosedLoopRampPeriod = 0.3;
        this.motor.motorConfig.peakReverseDC = -0.1; //?
        this.motor2.motorConfig.peakReverseDC = -1;
        this.motor.motorConfig.brake = false;
        this.motor2.motorConfig.brake = false;
        this.motor.applyConfig();
        this.motor2.applyConfig();
        this.motor.slot0TFX.kV = 0.011;
        this.motor2.slot0TFX.kV = 0.011;
        this.motor.pid(0.1, 0.0, 0.0); // Setup the indexer PID
        this.motor2.pid(0.01, 0.0, 0.0); // Setup the roller bed PID

        stallTimer = new Timer();
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
        if (!isStalled) {
            motor2.dc(rollerPercent);
        }
    }

    /**
     * Set target RPS for indexer & roller bed
     * @param rps
     */
    public void setRPS(double indexerRPS, double rollerRPS) {
        // Send the output to the motor
        motor.vel(indexerRPS);
        if (!isStalled) {
            motor2.vel(rollerRPS);
        }
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

    @Override public void periodic() { //stall detection
        if (!((this.motor2.vel() < 1.0 && this.motor2.motorTFX.getDutyCycle().getValueAsDouble() > 0.9 && RobotState.isEnabled())||isStalled)) {
            stallTimer.reset();
        }
        if (stallTimer.get() > 0.3) {
            isStalled = true;
        }
        if (isStalled) {
            this.motor2.vel(-20);
        }
        if (isStalled && stallTimer.get() > 0.5) {
            stallTimer.reset();
            isStalled = false;
        }
    }
}