package frc.robot.Actors.Subsystems.Spindexer;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors and Utils
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;

// Import Spindexer Contants
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    // intatiate motors
    private Motor motor;

    /**
     * Spindexer Constructor
     */
    public Spindexer() {
        this.motor = new Motor(SpindexerConstants.spinMotorID, MotorType.TFX);
    }

    // Motor Controls

    /**
     * Sets spindexer motor output (-1.0 to 1.0)
     */
    public void set(double percent) {
        motor.dc(percent);
    }
}