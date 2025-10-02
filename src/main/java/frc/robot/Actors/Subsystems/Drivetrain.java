package frc.robot.Actors.Subsystems;

// Import CTRE Hardware Libraries
import com.ctre.phoenix6.hardware.Pigeon2;

// Import WPI Libraries to help Swerve Drive Management
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Constants and Utils
import frc.robot.Utils.*;
import frc.robot.Constants.*;

// Import Subsystems
import frc.robot.Actors.SwerveModule;
import frc.robot.Actors.SwerveModules;

// Import States
import frc.robot.States.DrivetrainState;


public class Drivetrain extends SubsystemBase {

    // Inertial Measurement Unit
    private Pigeon2 pigeon;

    // Swerve Modules
    private SwerveModules swerveModules; // Helper class to help easily manage the individual swerve modules
    private Translation2d[] swerveModuleLocations_m; // Used to hold the position of the swerve modules with respect to the center of the robot
    private SwerveDriveOdometry odometry;
    private final SwerveDriveKinematics swerveKinematics;

    // state
    // TODO: Need to investigate how to use this as it is not currently used
    public DrivetrainState state = new DrivetrainState();
    public SwerveModuleState[] moduleStates;

    public Drivetrain() {
        /*
         * Setup auxillary sensors and components to help with the drivetrain
         */

        // pigeon
        this.pigeon = new Pigeon2(6);

        // Define the swerve modules
        // TODO: Test changing the numbers back to how we did last year. Found out how to display the swerves correctly on AdvantageScope
        this.swerveModules = new SwerveModules(
            new SwerveModule[] {
                new SwerveModule(1, Swerve.swerveModuleDriveConfigs()[1], Swerve.swerveModuleAzimuthConfigs()[1]),
                new SwerveModule(0, Swerve.swerveModuleDriveConfigs()[0], Swerve.swerveModuleAzimuthConfigs()[0]),
                new SwerveModule(2, Swerve.swerveModuleDriveConfigs()[2], Swerve.swerveModuleAzimuthConfigs()[2]),
                new SwerveModule(3, Swerve.swerveModuleDriveConfigs()[3], Swerve.swerveModuleAzimuthConfigs()[3])
            }
        );

        /*
         * Setup the module locations with respect to the center of the robot. To calculate the center of the modules we
         * take have the module to module length and width. We also need to take into consideration the coordinate system
         * of WPILib. That can be found
         * here: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#coordinate-system
         */
        // TODO: Test changing the numbers back to how we did last year. Found out how to display the swerves correctly on AdvantageScope
        this.swerveModuleLocations_m = new Translation2d[4];
        // Module 0 should be +X and -Y (Front Right - FR)
        this.swerveModuleLocations_m[1] = new Translation2d(
            Units.inchesToMeters(DrivetrainConstants.moduleToModuleLength_X_in / 2.0),
            Units.inchesToMeters(-DrivetrainConstants.moduleToModuleWidth_Y_in / 2.0)
        );
        // Module 1 should be +X and +Y (Front Left - FL)
        this.swerveModuleLocations_m[0] = new Translation2d(
            Units.inchesToMeters(DrivetrainConstants.moduleToModuleLength_X_in / 2.0),
            Units.inchesToMeters(DrivetrainConstants.moduleToModuleWidth_Y_in / 2.0)
        );
        // Module 2 should be -X and +Y (Back Left - BL)
        this.swerveModuleLocations_m[2] = new Translation2d(
            Units.inchesToMeters(-DrivetrainConstants.moduleToModuleLength_X_in / 2.0),
            Units.inchesToMeters(DrivetrainConstants.moduleToModuleWidth_Y_in / 2.0)
        );
        // Module 3 should be -X and -Y (Back Right - BR)
        this.swerveModuleLocations_m[3] = new Translation2d(
            Units.inchesToMeters(-DrivetrainConstants.moduleToModuleLength_X_in / 2.0),
            Units.inchesToMeters(-DrivetrainConstants.moduleToModuleWidth_Y_in / 2.0)
        );

        // Setup the swerve drive kinematics
        this.swerveKinematics = new SwerveDriveKinematics(this.swerveModuleLocations_m);

        // Setup the odometry tracking
        this.odometry = new SwerveDriveOdometry(
          this.swerveKinematics,
          this.pigeon.getRotation2d().unaryMinus(),
          swerveModules.getPosition()
        );
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        this.odometry.update(
            this.pigeon.getRotation2d().unaryMinus(),
            this.swerveModules.getPosition()
        );

        System.out.println(this.pigeon.getRotation2d().unaryMinus().getDegrees());
    }

    /**
     * gets the current pose from the swerve odometry.
     * 
     * @return Pose2d
     */
    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        this.odometry.resetPosition(
            this.pigeon.getRotation2d().unaryMinus(),
            this.swerveModules.getPosition(),    
            pose
        );
    }

    /**
     * drive the drivetrain at an x, y and h power field relative
     * 
     * @param xpow        power to drive at in field relative x
     * @param ypow        power to drive at in field relative y
     * @param hpow        power to change the heading (spin) at
     */
    public void drive(double xpow, double ypow, double hpow) {
        /**
         * This handles the issue where the xpow and ypow are values that end up in the shaded (unreal areas) of a circumscribed square
         * We caclulate the hypotenuse and if it ends up in the area between the square and circle (> 1), then we divide the values
         * by the hypotenuse to get them in a valid range. If we don't do this, then the swerveKinematics.toSwerveModuleStates will seem
         * to not be "reacting" to the controller changes
         */
        double hyp = Math.hypot(xpow, ypow);
        if (hyp > 1.0){
          ypow /= hyp;
          xpow /= hyp;
        }

        // Calculate the swerve module states (drive and azimuth motor commands) based on the controller inputs and the max ground and rotate speeds
        SwerveModuleState[] moduleStateOutputs = this.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                xpow * DrivetrainConstants.maxGroundSpeed_mPs,
                ypow * DrivetrainConstants.maxGroundSpeed_mPs,
                hpow * DrivetrainConstants.maxRotateSpeed_radPs,
                // Converting pigeon from left hand rule (x+) to a right hand rule (x+)
                this.pigeon.getRotation2d().unaryMinus()
                ),
                0.001 * RobotConstants.loopTime_ms
            )
        );

        // Renormalizes the wheel speeds if any individual speed is above the specified maximum. 
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStateOutputs, DrivetrainConstants.maxGroundSpeed_mPs);

        // Send the commands to the swerve modules
        swerveModules.pushModuleStates(moduleStateOutputs, DrivetrainConstants.maxGroundSpeed_mPs);
    }

    /** Zeroes the heading of the robot. */
    public void resetIMU() {
        this.pigeon.reset();
    }
}