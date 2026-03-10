package frc.robot.Actors.Subsystems.Shooter;

// Import Phoneix6 Libraries
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors and Utils
import frc.robot.Actors.Motor;
//import frc.robot.Utils.HubShooterTrajectoryCalc;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

// Import Spindexer Contants
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    
    // intatiate motors
    private Motor leadMotor;
    private Motor followMotor;

    private Motor hood;


    public Shooter() {
        // Initialize the motors
        this.leadMotor = new Motor(ShooterConstants.leadMotorID, MotorType.TFX);
        this.followMotor = new Motor(ShooterConstants.followMotorID, MotorType.TFX);
        this.hood = new Motor(ShooterConstants.hoodMotorID, MotorType.TFX);

        // Configure the lead motor
        this.leadMotor.motorConfig.direction = RotationDir.CounterClockwise;
        this.leadMotor.motorConfig.dutyCycleClosedLoopRampPeriod = 0.3;
        this.leadMotor.applyConfig();
        this.leadMotor.pid(13, 0.6, 0.025); // Setup the Shooter PID

        // Set the followMotor to follow the lead motor, and make it opposed
        this.followMotor.motorTFX.setControl(new Follower(ShooterConstants.leadMotorID, MotorAlignmentValue.Opposed));
    
        this.hood.motorTFX.setPosition(0.02604);
        this.hood.motorConfig.direction = RotationDir.Clockwise;
        this.hood.applyConfig();
        this.hood.pid(0.0, 0.0, 0.0);
    }

    /**
     * Set target RPS
     */
    public void setRPS(double rps) {
        leadMotor.vel(rps);
    }

    public void setHoodPos(double pos) {
        this.hood.pos(pos);
    }

    /**
     * Stop motor
     */
    public void stop() {
        leadMotor.vel(0.0);
    }

    /**
     * Get current RPS from internal encoder
     */
    public double getRPS() {
        return leadMotor.vel();
    }
}