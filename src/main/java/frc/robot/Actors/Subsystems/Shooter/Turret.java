package frc.robot.Actors.Subsystems.Shooter;

// Import WPILib Libraries
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Phoneix6 Libraries
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;

// Import Actors, Utils & Constants
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    // initiate motors
    private Motor motor;

    public Turret() {
        // Configure the turret motor
        this.motor = new Motor(TurretConstants.turretMotorID, MotorType.TFX);
        this.motor.motorConfig.direction = RotationDir.Clockwise;
        this.motor.motorConfig.peakForwardDC = 1;
        this.motor.motorConfig.peakReverseDC = -1;

        this.motor.configTFX.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANifier;
        this.motor.configTFX.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 51;
        this.motor.configTFX.HardwareLimitSwitch.ForwardLimitEnable = false;
        this.motor.configTFX.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;

        // TODO: DELETE THIS?
        // this.motor.configTFX.ClosedLoopGeneral.ContinuousWrap = true;
        // this.motor.configTFX.Feedback.SensorToMechanismRatio =
        // TurretConstants.turretGearRatio;

        this.motor.applyConfig();
        this.motor.motionMagic(1.0, 0.0, 0.0, 0.05, 0.0, 8.0, 10.0);
    }

    /**
     * Set the duty cycle output for the turret motor
     * 
     * @param dc
     */
    public void dc(double dc) {
        // Send the output to the motor
        this.motor.dc(dc);
    }

    public void setBrake(boolean brake) {
        this.motor.motorConfig.brake = brake;
        this.motor.applyConfig();
    }

    /**
     * Set the zero of the turret motor
     */
    public void zero() {
        // Set the current position to 0.0
        this.motor.motorTFX.setPosition(0.0);
    }

    /**
     * @return true if beam break is active, false otherwise
     */
    public boolean beambreakActive() {
        // Return beam breat status (true or false)
        return this.motor.motorTFX.getForwardLimit().getValue().equals(ForwardLimitValue.Open);
    }

    /**
     * Sets the forward limit to a true or false state for the turret motor
     * @param state
     */
    public void setForwardLimit(boolean state) {
        this.motor.configTFX.HardwareLimitSwitch.ForwardLimitEnable = state;
        this.motor.applyConfig();
    } 

    /**
     * @return the turret position in degrees
     */
    public double getTurretDegrees() {
        return this.motor.pos() / TurretConstants.turretGearRatio * 360.0; // Convert rotations to degrees
    }

    /**
     * @return the turret position in Rotation2d 
     */
    public Rotation2d getTurretAngle() {
        return Rotation2d.fromDegrees(getTurretDegrees());
    }

    public double getTemp() {
        return this.motor.getTemp();
    }

    /**
     * Sets the target angle for the turret
     * 
     * @param angle 
     */
    public void setTargetAngle(Rotation2d angle) {
        // System.out.println((Math.round(angle.getRotations() * 100) / 100.0) + " : "
        //         + (Math.round(motor.motorTFX.getPosition().getValueAsDouble() / TurretConstants.turretGearRatio * 100)
        //                 / 100.0)
        //         + " : " +
        //         (Math.round(motor.motorTFX.getClosedLoopOutput().getValueAsDouble() * 100) / 100.0));
        double desired = angle.getRotations() * TurretConstants.turretGearRatio;
        while (desired < -1.83 || desired > 9.14) {
            if (desired < -1.83) {
                desired+=TurretConstants.turretGearRatio;
            }
            if (desired > 9.14) {
                desired-=TurretConstants.turretGearRatio;
            }
        }

        this.motor.posMM(desired);


        SmartDashboard.putNumber("diff", Math.abs((angle.minus(new Rotation2d(
            motor.motorTFX.getPosition().getValueAsDouble() * 2 * Math.PI / TurretConstants.turretGearRatio)))
            .getDegrees()));
    }

    @Override
    public void periodic() {
        //System.out.println("turret: "+this.getTurretDegrees());
    }
}