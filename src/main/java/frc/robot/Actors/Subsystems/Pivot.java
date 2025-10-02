package frc.robot.Actors.Subsystems;

// Import custom actors and utils
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Pivot extends SubsystemBase {
    // Real Motor Variables
    private Motor motor;
    private int motor_canID;

    // Real Encoder Variables
    private DutyCycleEncoder encoder;
    private int encoder_channelID;
    private double encoderOffset = 160.0;

    // Soft Limits
    private double minAngle_deg = 4.0; // max back
    private double maxAngle_deg = 194.0; // max front

    // PID Controller
    private PIDController pid;

    // State
    private double targetAngle_deg = 190.0; // Degrees

    /**
     * Creates the CoralIntake Class
     * 
     * @param motor_canID       The canID number of the intake, used to set the CAN
     *                          ID of the motor
     * @param encoder_channelID The Channel ID of the encoder plugged into the RIO
     */
    public Pivot(int motor_canID, int encoder_channelID) {
        // initialize the motor number and motor
        this.motor_canID = motor_canID;
        this.motor = new Motor(this.motor_canID, MotorType.TFX);

        // Setup the motor configurations
        this.motor.motorConfig.brake = true;
        this.motor.motorConfig.direction = RotationDir.CounterClockwise;
        this.motor.applyConfig();

        // initialize the encoder channel and encoder
        this.encoder_channelID = encoder_channelID;
        this.encoder = new DutyCycleEncoder(this.encoder_channelID, 1, 0); // Absolute encoder

        // initialize the PID
        // TODO: This needs to be tuned for the real system
        this.pid = new PIDController(0.008, 0.0, 0.0);
    }

    /**
     * Sets the pivot motor to spin in the direction to move the pivot forwards
     */
    public void forward() {
        // Enforce soft limits: prevent movement beyond range
        if (isAtUpperLimit()) {
            this.motor.dc(0.0);
        } else {
            this.motor.dc(0.2);
        }
    }

    /**
     * Sets the pivot motor to spin in the direction to move the pivot backwards
     */
    public void backward() {
        // Enforce soft limits: prevent movement beyond range
        if (isAtLowerLimit()) {
            this.motor.dc(0.0);
        } else {
            this.motor.dc(-0.2);
        }
    }

    /**
     * Sets the target angle for the Pivot system.
     * This function will ensure it is clamped between the min and max angles.
     * 
     * @param angle_deg The angle that the pivot is requested to go to
     */
    public void setTargetAngle(double angle_deg) {
        // Clamp target angle within soft limits
        this.targetAngle_deg = Math.max(this.minAngle_deg, Math.min(this.maxAngle_deg, angle_deg));
    }

    /**
     * Returns the current pivot angle
     * 
     * @return the pivot angle in degrees
     */
    public Rotation2d getAngle() {
        double encoderRotations = (encoderOffset / 360.0) - encoder.get();
        return Rotation2d.fromRotations(encoderRotations);
    }

    /**
     * Returns the current pivot angle
     * 
     * @return the pivot angle in degrees
     */
    public double getAngleDegrees() {
        // Get the current angle degrees of the encoder
        double degrees = getAngle().getDegrees();

        // Return the degrees adjusting for the wrapping
        return (degrees < 0) ? 360 + degrees : degrees;
    }

    /**
     * Returns if the pivot is at the lower limit
     * 
     * @return true if at the lower limit, else false
     */
    public boolean isAtLowerLimit() {
        return getAngleDegrees() <= this.minAngle_deg;
    }

    /**
     * Returns if the pivot is at the upper limit
     * 
     * @return true if at the upper limit, else false
     */
    public boolean isAtUpperLimit() {
        return getAngleDegrees() >= this.maxAngle_deg;
    }

    /**
     * Sets the pivot motor to 0 speed
     */
    public void stop() {
        this.motor.dc(0.0);
    }

    public void updateOutputs() {
        // Calculate motor output from PID controller
        double motorOutput = pid.calculate(getAngleDegrees(), this.targetAngle_deg);

        // Clamp output to safe range [-1.0, 1.0]
        motorOutput = Math.max(-0.5, Math.min(0.5, motorOutput));

        // Enforce soft limits: prevent movement beyond range
        if (isAtUpperLimit() && motorOutput > 0) {
            motorOutput = 0.0;
        }
        if (isAtLowerLimit() && motorOutput < 0) {
            motorOutput = 0.0;
        }

        // Send the output to the motor
        this.motor.dc(motorOutput);

        // Logging to console code
        // TODO: Remove when done testing
        // System.out.print(Math.round(getAngleDegrees()));
        // System.out.print("; ");
        // System.out.print((Math.round(this.targetAngle_deg)));
        // System.out.print("; ");
        // System.out.println(Math.round(motorOutput*100.0)/100.0);
    }

    @Override
    public void periodic() {
        // Logging to console code
        // TODO: Remove when done testing
        //System.out.println(Math.round(this.getAngleDegrees()));
    }
}
