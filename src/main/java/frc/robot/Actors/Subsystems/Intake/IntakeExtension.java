package frc.robot.Actors.Subsystems.Intake;

// Import WPILib Libraries
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import frc.robot.Constants.IntakeConstants;

public class IntakeExtension extends SubsystemBase {

    // Define variables
     private Motor motor; // Motor to control the intake extension position

    public IntakeExtension() {
        // Configure the intake extension motor
        this.motor = new Motor(IntakeConstants.extensionMotorID, MotorType.TFX, "rio");

        this.motor.motorTFX.setPosition(0.0);             // 0 degrees
        this.motor.motorConfig.direction = RotationDir.Clockwise;
        this.motor.applyConfig();
        this.motor.pid(0.02, 0.01, 0.0);
    }

    // Motor Controls

    /**
     * Sets Intake extension motor output (-1.0 to 1.0)
     * @param percent
     */
    public void set(double percent) {
        // Send the output to the motor
        motor.dc(percent);
    }

    /**
     * Sets the intake target in degrees
     * 
     * @param degrees
     */
    public void setIntakePos(double degrees) {
        degrees = MathUtil.clamp(degrees, IntakeConstants.minDegree, IntakeConstants.maxDegree);
        double motorRotations = degreesToMotorRotations(degrees);
        motor.pos(motorRotations);
    }

    /**
     * degreesToMotorRotations is a private function to calculate what the degrees translates to for motor rotations for the motor
     * 
     * @param degrees
     */
    private double degreesToMotorRotations(double degrees) {
        return (degrees - IntakeConstants.minDegree) * IntakeConstants.extensionGearRatio / 360.0;
    }
}