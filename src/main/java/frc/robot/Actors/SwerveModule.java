package frc.robot.Actors;

// Import CTRE Hardware Libraries
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// Import WPI Libraries to help Swerve Drive Management
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Utils and Constants
import frc.robot.Utils.MotorType;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule extends SubsystemBase {
    /*
     * Module number for the FRC Robot - Help determine the CAN ID of the swerve module
     * 
     * 4265 Naming/ID Convention:
     * Module 0 - Front Right (Drive Motor CAN ID - 10, Azimuth Motor CAN ID - 20)
     * Module 1 - Front Right (Drive Motor CAN ID - 11, Azimuth Motor CAN ID - 21)
     * Module 2 - Front Right (Drive Motor CAN ID - 12, Azimuth Motor CAN ID - 22)
     * Module 3 - Front Right (Drive Motor CAN ID - 13, Azimuth Motor CAN ID - 23)
     */
    private final int moduleNumber;

    // Define our drive and azimuth motors
    private Motor drive;
    private Motor azimuth;

    // Setup variables for tracking the speed and angle of the module
    private double currentDriveSpeed_mPs = 0;
    private double currentAzimuthAngle_rad = 0;

    public SwerveModule(int moduleNumber, TalonFXConfiguration driveMotorConfig, TalonFXConfiguration azimuthMotorConfig) {
        // Set the module number of the swerve
        this.moduleNumber = moduleNumber;

        // Setup the drive motor configurations
        this.drive = new Motor(10 + this.moduleNumber, MotorType.TFX);
        this.drive.applyTalonFxConfig(driveMotorConfig);

        // Setup the azimuth motor configurations
        this.azimuth = new Motor(20 + moduleNumber, MotorType.TFX);
        this.azimuth.applyTalonFxConfig(azimuthMotorConfig);

        // Setup the Azimuth PID
        this.azimuth.pid(0.15, 0.0, 0.0);
    }

    /**
     * updates the internal current state of the swerve module.
     * 
     * @return the current swerveModuleState of the module
     */
    public SwerveModuleState getCurrentState() {
        // Calculate the current module angle in radians
        currentAzimuthAngle_rad = Units.rotationsToRadians(azimuth.pos() / DrivetrainConstants.azimuthGearRatio);
        // Calculate the current module wheel speein in meters / second
        currentDriveSpeed_mPs = drive.vel()
            / DrivetrainConstants.driveGearRatio * 2
            * Math.PI
            * DrivetrainConstants.wheelRadius_m;

        // Return the swerve module state
        return new SwerveModuleState(currentDriveSpeed_mPs, new Rotation2d(currentAzimuthAngle_rad));
    }

    /**
     * Returns the position of the drive and azimuth motors
     * 
     * @return A SwerveModulePosition object
     */
    public SwerveModulePosition getPosition() {
        // Calculate the swerve module position
        return new SwerveModulePosition(
            (drive.pos() / DrivetrainConstants.driveGearRatio) * (2 * Math.PI * DrivetrainConstants.wheelRadius_m),
            new Rotation2d((azimuth.pos() / DrivetrainConstants.azimuthGearRatio) * 2 * Math.PI)
        );
    }

    /**
     * Gets any faults from the drive and azimuth motors
     * 
     * @return A boolean array with the structure:
     *         <ul>
     *         <li>drive fault,
     *         <li>azimuth fault
     */
    public boolean[] getSwerveFaults() {
        return new boolean[] { drive.isFault(), azimuth.isFault() };
    }

    public void pushModuleState(SwerveModuleState moduleState, double maxGroundSpeed_mPs) {
        /*
         * Determine the module states to create
         */
        // Get the encoderRotation and azimuth angle in radians
        var encoderRotation = new Rotation2d(Units.rotationsToRadians(azimuth.pos() / DrivetrainConstants.azimuthGearRatio));
        double azimuthAngle_rad = Units.rotationsToRadians(azimuth.pos() / DrivetrainConstants.azimuthGearRatio);

        // Optimize the reference state to avoid spinning further than 90 degrees
        moduleState.optimize(encoderRotation);

        // TODO: I think we can uncomment this when we are actually driving the robot. The signal output I thinks goes to 0 when testing
        // because the angle of the wheel is not being updated
        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        //moduleState.cosineScale(encoderRotation);

        // Wrapping the angle to allow for "continuous input"
        double minDistance = MathUtil.angleModulus(moduleState.angle.getRadians() - azimuthAngle_rad);

        /*
         * Calculate the azimuth output
         */
        double normalAzimuthOutput_rot = Units.radiansToRotations(azimuthAngle_rad + minDistance)
                * DrivetrainConstants.azimuthGearRatio;

        /*
         * Calculate the drive output
         */
        // Output drive
        double driveOutput = moduleState.speedMetersPerSecond / maxGroundSpeed_mPs;

        // TODO: Is this the same as: moduleState.cosineScale(encoderRotation) (LINE 113)
        driveOutput *= moduleState.angle.minus(new Rotation2d(currentAzimuthAngle_rad)).getCos();

        // Send the outputs to the drive and azimuth motors
        azimuth.pos(normalAzimuthOutput_rot);
        drive.dc(driveOutput);
    }

    /**
     * TODO: describe the purpose of this function
     */
    public void pushLockState(boolean calibrateWheels, boolean unlockWheels) {
        if (calibrateWheels) {
            azimuth.resetPos(0.0);
        }

        azimuth.setBrake(!unlockWheels); // invert bc unlock != lock
    }

    /**
     * Temperature of drive motor
     * 
     * <ul>
     * <li><b>Minimum Value:</b> 0.0
     * <li><b>Maximum Value:</b> 255.0
     * <li><b>Default Value:</b> 0
     * <li><b>Units:</b> ℃
     * </ul>
     * 
     * @return Double temperature in degrees Celcius
     */
    public double getTemp() {
        return drive.getTemp();
    }
}