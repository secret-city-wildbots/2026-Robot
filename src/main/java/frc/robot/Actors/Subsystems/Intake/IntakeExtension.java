package frc.robot.Actors.Subsystems.Intake;

// Import WPILib Libraries
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import frc.robot.Constants.IntakeConstants;

public class IntakeExtension extends SubsystemBase {

    // Define variables
     private Motor motor; // Motor to control the intake extension position
     public double p = 0.02;
     public double i = 0.01;
     public double d = 0.00;

    public IntakeExtension() {
        // Configure the intake extension motor
        this.motor = new Motor(IntakeConstants.extensionMotorID, MotorType.TFX, "rio");

        this.motor.motorTFX.setPosition(0.0);             // 0 degrees
        this.motor.motorConfig.direction = RotationDir.Clockwise;
        this.motor.applyConfig();
        this.motor.pid(p, i, d);

        SmartDashboard.putNumber("P", p);
        SmartDashboard.putNumber("I", i);
        SmartDashboard.putNumber("D", d);
    }

    public double getTemp() {
        return this.motor.getTemp();
    }

    public void setBrake(boolean brake) {
        this.motor.motorConfig.brake = brake;
        this.motor.applyConfig();
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

    @Override
    public void periodic() {
        if (SmartDashboard.getNumber("P", p) != p || 
            SmartDashboard.getNumber("I", i) != i ||
            SmartDashboard.getNumber("D", d) != d) {
                p = SmartDashboard.getNumber("P", p);
                i = SmartDashboard.getNumber("I", i);
                d = SmartDashboard.getNumber("D", d);
                this.motor.pid(p,i,d);
        }
    }
}