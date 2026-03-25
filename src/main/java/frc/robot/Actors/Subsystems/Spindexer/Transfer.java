package frc.robot.Actors.Subsystems.Spindexer;

import edu.wpi.first.wpilibj.DigitalInput;
// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import frc.robot.Constants.SpindexerConstants;
import edu.wpi.first.wpilibj.Timer;

public class Transfer extends SubsystemBase {

    // Define variables
    public Motor motor; // Motor to control the transfer
    
    // Beam break + BPS tracking
    private DigitalInput bpsBeamBreak;
    private boolean lastBeamState;
    public static double bps;
    private int ballCount;
    
    // Measuring flag
    private boolean isMeasuring;

    // Timer for measurement window
    private Timer timer;

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

        // Beam break setup
        this.bpsBeamBreak = new DigitalInput(SpindexerConstants.bpsBeamBreakPort);
        this.lastBeamState = this.bpsBeamBreak.get();

        // BPS tracking
        Transfer.bps = 0.0;
        this.ballCount = 0;

        // Timer setup
        this.timer = new Timer();
        
        this.isMeasuring = false;
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

    /**
     * Start measuring a shooting cycle
     */
    public void startMeasuring() {
        isMeasuring = true;

        ballCount = 0;
        Transfer.bps = 0.0;

        timer.reset();
        timer.start();
    }

    /**
     * Stop measuring
     */
    public void stopMeasuring() {
        isMeasuring = false;

        timer.stop();
        timer.reset();

        ballCount = 0;
        Transfer.bps = 0.0;
    }

    @Override
    public void periodic() {
        boolean currentBeamState = this.bpsBeamBreak.get();

        if (isMeasuring) {

            // Detect ball passing (edge detection)
            if (lastBeamState && !currentBeamState) {
                ballCount++;
            }

            double time = timer.get();

            if (time > 0.0) {
                Transfer.bps = this.ballCount / time;
            } else {
                Transfer.bps = 0.0;
            }
        }

        lastBeamState = currentBeamState;
    }
}