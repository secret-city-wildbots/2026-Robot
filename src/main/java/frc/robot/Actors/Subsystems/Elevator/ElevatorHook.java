package frc.robot.Actors.Subsystems.Elevator;

// Import WPILib Libraries
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Phoenix 6 Libraries
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorHook extends SubsystemBase {
    
    // Define variables
    private Motor motor;           // Motor to control the elevator hook position
    private CANcoder encoder;      // Encoder to control the position of the hook
    private double targetAngle;    // Target angle to set the hook position to

    public ElevatorHook() {
        // Configure the elevator hook motor
        this.motor = new Motor(ElevatorConstants.hookMotorID, MotorType.TFX, "rio");
        this.motor.motorConfig.direction = RotationDir.CounterClockwise;
        this.motor.applyConfig();

        // Configure the elevator hook encoder
        this.encoder = new CANcoder(ElevatorConstants.hookMotorCancoderID);
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.withMagnetOffset(-ElevatorConstants.hookEncoderOffset);
        this.encoder.getConfigurator().apply(config);

        // Initialize the target angle to be 0.0 degrees
        this.targetAngle = 0.0;
    }

    public double getTemp() {
        return this.motor.getTemp();
    }

    // Motor Controls

    /**
     * Sets ElevatorHook motor output (-1.0 to 1.0)
     * @param percent
     */
    public void set(double percent) {
        // Clamp input percentage to proper range
        percent = MathUtil.clamp(percent, -1.0, 1.0);

        // Check to make sure the hooks are safe to extend out
        if (percent > 0.0 && getCurrentAngle() >= ElevatorConstants.hookDeployedPosition) {
            // if it is not safe, dont allow the motor to move
            motor.dc(0.0);
            return;
        }

        // check to make sure the hooks are safe to retract in
        if (percent < 0.0 && getCurrentAngle() <= ElevatorConstants.hookSafePosition) {
            // if it is not safe, dont allow the motor to move
            motor.dc(0.0);
            return;
        }
        
        // Send the output to the motor
        motor.dc(percent);
    }

    /**
     * Sets the target angle for the hooks to travel to
     * @param angle
     */
    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }

    // Sensor Controls

    /**
     * Get the encoder position in degrees
     */
    public double getCurrentAngle() {
        // We get the absolute position of the encoder (-1 - 1 rotations) and multiply by 360 to get degrees
        return this.encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    /**
     * Get the target position in degrees
     */
    public double getTargetAngle() {
        // We return the target angle
        return this.targetAngle;
    }

    @Override
    public void periodic() {
        // TODO: put logic to send position states to dashboard
        // System.out.println("--------------------------------------------------");
        // System.out.println("Encoder: " + this.getCurrentAngle()); 
    }
}
