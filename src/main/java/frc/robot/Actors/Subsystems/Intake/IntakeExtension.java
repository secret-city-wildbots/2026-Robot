package frc.robot.Actors.Subsystems.Intake;

// Import WPILib Libraries
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
        this.motor.motorConfig.brake = false;
        this.motor.applyConfig();
        this.motor.motionMagic(0.15, 0.02, 0.01, 0.0, 0.0, 40.0, 50.0);

        this.motor.curlim.SupplyCurrentLimit = 15;
        this.motor.curlim.SupplyCurrentLimitEnable = true;
        this.motor.motorTFX.getConfigurator().apply(this.motor.curlim);
        //this.motor.pid(0.0, 0.0, 0.0);
        this.setIntakePos(IntakeConstants.minDegree);
    }

    public double getTemp() {
        return this.motor.getTemp();
    }

    public double getPos() {
        return this.motor.pos() / IntakeConstants.extensionGearRatio * 360.0;
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
        motor.posMM(motorRotations);
    }

    /**
     * degreesToMotorRotations is a private function to calculate what the degrees translates to for motor rotations for the motor
     * 
     * @param degrees
     */
    private double degreesToMotorRotations(double degrees) {
        return (degrees) * IntakeConstants.extensionGearRatio / 360.0;
    }

    @Override
    public void periodic() {
        if (RobotState.isEnabled()) {
            motor.setBrake(true);
        } else {
            motor.setBrake(false);
        }
        //System.out.println("I: "+(motor.pos() * 360 / IntakeConstants.extensionGearRatio + IntakeConstants.minDegree));
    }
}
