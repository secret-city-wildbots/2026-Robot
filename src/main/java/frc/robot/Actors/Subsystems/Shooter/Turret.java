package frc.robot.Actors.Subsystems.Shooter;

// Import WPILib Libraries
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
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
    private boolean lastBrake = true;

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
        this.motor.motionMagic(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);//?
        //this.motor.motionMagic(0.0, 0.0, 0.0, 0.00/5.0, 0.0, 30.0*5.0, 30.0*5.0);//?
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
        double current = this.motor.pos()/TurretConstants.turretGearRatio*360;

        double desired_deg = angle.getDegrees()%360;
        if (desired_deg < 0) desired_deg+=360;

        double[] goals = new double[] {
            desired_deg-360,
            desired_deg,
            desired_deg+360
        };

        if (Math.abs(goals[0]-current) <= 180 && goals[0] > -185) {
            desired_deg = goals[0];
        } else if (Math.abs(goals[2]-current) <= 180 && goals[2] < 450) {
            desired_deg = goals[2];
        } else {
            desired_deg = goals[1];
        }
        
        double desired_mRot = desired_deg/360*TurretConstants.turretGearRatio;

        /*if (Math.abs(current - (desiredOld)) > 180) {
            desired+= Math.signum(current - (desiredOld)) * TurretConstants.turretGearRatio;
        }*/

        //System.out.println(Math.round(current) + " : "+Math.round(desired_deg));

        /*while (desired_mRot > TurretConstants.posExtension) {
            desired_mRot-=TurretConstants.turretGearRatio;
        }
        while (desired_mRot < TurretConstants.negExtension) {
            desired_mRot+=TurretConstants.turretGearRatio;
        }*/

        this.motor.posMM(desired_mRot);


        SmartDashboard.putNumber("diff", Math.abs((angle.minus(new Rotation2d(
            motor.motorTFX.getPosition().getValueAsDouble() * 2 * Math.PI / TurretConstants.turretGearRatio)))
            .getDegrees()));
    }

    @Override
    public void periodic() {
        /*if (RobotState.isEnabled()) {
            this.motor.motorConfig.brake = true;
        } else {
            this.motor.motorConfig.brake = false;
        }
        if (this.motor.motorConfig.brake != this.lastBrake) {
            this.motor.applyConfig();
            this.lastBrake = this.motor.motorConfig.brake;
        }*/
        //System.out.println("turret: "+this.getTurretDegrees());
    }
}