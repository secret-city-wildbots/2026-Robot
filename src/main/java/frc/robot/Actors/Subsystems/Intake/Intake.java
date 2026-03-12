package frc.robot.Actors.Subsystems.Intake;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    // Define variables
    private Motor motor; // Motor to control the intake position

    public Intake() {
        // Configure the intake motor
        this.motor = new Motor(IntakeConstants.intakeMotorID, MotorType.TFX, "drivetrain");
    }

    // Motor Controls

    /**
     * Sets Intake motor output (-1.0 to 1.0)
     * @param percent
     */
    public void set(double percent) {
        // Send the output to the motor
        motor.dc(percent);
    }
}