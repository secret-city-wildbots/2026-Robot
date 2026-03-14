package frc.robot.Actors.Subsystems.Shooter;

// Import WPILib Libraries
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Phoneix6 Libraries
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
//import frc.robot.Utils.HubShooterTrajectoryCalc;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    
    // initiate motors
    private Motor leadMotor;   // Motor to control the shooter (all commands come to this motor)
    private Motor followMotor; // Motor to control the shooter (this motor will just copy the lead motor)
    private Motor hoodMotor;        // Motor to control the hood of the shooter (angle the ball exits)


    public Shooter() {
        // Initialize the motors
        this.leadMotor = new Motor(ShooterConstants.leadMotorID, MotorType.TFX);
        this.followMotor = new Motor(ShooterConstants.followMotorID, MotorType.TFX);
        this.hoodMotor = new Motor(ShooterConstants.hoodMotorID, MotorType.TFX);

        // Configure the lead motor
        this.leadMotor.motorConfig.direction = RotationDir.CounterClockwise;
        this.leadMotor.motorConfig.dutyCycleClosedLoopRampPeriod = 0.3;
        this.leadMotor.applyConfig();
        this.leadMotor.pid(13, 0.6, 0.025); // Setup the Shooter PID

        // Set the followMotor to follow the lead motor and make it opposed
        this.followMotor.motorTFX.setControl(new Follower(ShooterConstants.leadMotorID, MotorAlignmentValue.Opposed));
    
        this.hoodMotor.motorTFX.setPosition(0.02604);        // 5 degrees
        this.hoodMotor.motorConfig.direction = RotationDir.Clockwise;
        this.hoodMotor.applyConfig();
        this.hoodMotor.pid(6.0, 0.0, 0.0);
    }

    /**
     * Set target RPS for the shooter
     * 
     * @param rps
     */
    public void setRPS(double rps) {
        // Send the output to the motor
        leadMotor.vel(rps);
    }

    /**
     * Stop shooter motor - sets the target RPS for the shooter to 0.0
     */
    public void stop() {
        // Send the output to the motor
        leadMotor.vel(0.0);
    }

    /**
     * Get current RPS from internal encoder
     * 
     * @return double (rps of the motors)
     */
    public double getRPS() {
        return leadMotor.vel();
    }

    /**
     * Sets the hood angle target in degrees
     * 
     * @param degrees
     */
    public void setHoodAngle(double degrees) {
        degrees = MathUtil.clamp(degrees, ShooterConstants.minDegree, ShooterConstants.maxDegree);
        double motorRotations = degreesToMotorRotations(degrees);
        hoodMotor.pos(motorRotations);
    }

    /**
     * degreesToMotorRotations is a private function to calculate what the degrees translates to for motor rotations for the motor
     * 
     * @param degrees
     */
    private double degreesToMotorRotations(double degrees) {
        return (degrees - ShooterConstants.minDegree) * ShooterConstants.hoodGearRatio / 360.0;
    }
}