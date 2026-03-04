package frc.robot.Actors.Subsystems.Spindexer;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors and Utils
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
// Import Spindexer Contants
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    // intatiate motors
    private Motor motor;

    /**
     * Spindexer Constructor
     */
    public Spindexer() {
        this.motor = new Motor(SpindexerConstants.spinMotorID, MotorType.TFX, "rio");
        this.motor.motorConfig.direction = RotationDir.CounterClockwise;
        this.motor.motorConfig.dutyCycleClosedLoopRampPeriod = 0.3;
        this.motor.applyConfig();
        this.motor.pid(13, 0.6, 0.025); // Setup the spindexer PID
    }

    // Motor Controls

    /**
     * Sets spindexer motor output (-1.0 to 1.0)
     */
    public void set(double percent) {
        motor.dc(percent);
    }

    /**
     * Set target RPS
     */
    public void setRPS(double rps) {
        motor.vel_dc(rps);
    }

    /**
     * Stop motor
     */
    public void stop() {
        motor.vel_dc(0.0);
    }

    /**
     * Get current RPS from internal encoder
     */
    public double getRPS() {
        return motor.vel();
    }
}