package frc.robot.Actors;

// Import WPI Libraries to help Swerve Drive Management
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

// Import SmartDashbaord Library for sending information to the Network Tables
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Import SubsystemBase for extension
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModules extends SubsystemBase {
    // Array to hold all of our swerve modules
    public SwerveModule[] swerveModules;

    /**
     * a helper class so that you don't have to be constantly annoyed by arrays
     * 
     * @param modules the swerve modules to incorporate
     */
    public SwerveModules(SwerveModule[] modules) {
        this.swerveModules = modules;
    }

    /**
     * get the positions of all the swerve modules
     * 
     * @return and array containing all the SwerveModulePositions
     */
    public SwerveModulePosition[] getPosition() {
        // Create array of swerve module positions
        SwerveModulePosition[] positions = new SwerveModulePosition[this.swerveModules.length];

        // Loop through each of the swerve modules and get their indivual positions and add them to the array
        for (int i = 0; i < this.swerveModules.length; i++) {
            positions[i] = this.swerveModules[i].getPosition();
        }

        // Return all of the positions
        return positions;
    }

    /**
     * sets the output states of all the swerve modules
     */
    public void pushModuleStates(SwerveModuleState[] moduleState, double maxGroundSpeed_mPs) {
        // TODO: Test changing the numbers back to how we did last year. Found out how to display the swerves correctly on AdvantageScope
        // for (int i = 0; i < this.swerveModules.length; i++) {
        //     this.swerveModules[i].pushModuleState(moduleState[i], maxGroundSpeed_mPs);
        // }

        // This worked when testing Saturday
        this.swerveModules[0].pushModuleState(moduleState[0], maxGroundSpeed_mPs);
        this.swerveModules[1].pushModuleState(moduleState[1], maxGroundSpeed_mPs);
        this.swerveModules[2].pushModuleState(moduleState[2], maxGroundSpeed_mPs);
        this.swerveModules[3].pushModuleState(moduleState[3], maxGroundSpeed_mPs);

        // Logging values for use in Advantage Scope
        double[] loggingState = new double[] {
                moduleState[0].angle.getRadians(),
                moduleState[0].speedMetersPerSecond,
                moduleState[1].angle.getRadians(),
                moduleState[1].speedMetersPerSecond,
                moduleState[2].angle.getRadians(),
                moduleState[2].speedMetersPerSecond,
                moduleState[3].angle.getRadians(),
                moduleState[3].speedMetersPerSecond
        };

        SmartDashboard.putNumberArray("moduleStates", loggingState);
    }

    public void zeroAzimuths() {
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].azimuth.resetPos(0);
        }
    }

    public void unlockAzimuths() {
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].azimuth.setBrake(false);
        }
    }

    /**
     * get the module state from all the swerve modules
     * 
     * @return and array containing all the SwerveModuleStates
     */
    public SwerveModuleState[] getCurrentState() {
        // Create array of swerve module states
        SwerveModuleState[] states = new SwerveModuleState[this.swerveModules.length];

        // Loop through each of the swerve modules and get their indivual states and add them to the array
        for (int i = 0; i < this.swerveModules.length; i++) {
            states[i] = this.swerveModules[i].getCurrentState();
        }

        // Return the states
        return states;
    }

    @Override
    public void periodic() {
        // Get the current states of the modules
        var moduleStates = getCurrentState();

        // Logging values for use in Advantage Scope
        double[] loggingState = new double[] {
            moduleStates[0].angle.getRadians(),
            moduleStates[0].speedMetersPerSecond,
            moduleStates[1].angle.getRadians(),
            moduleStates[1].speedMetersPerSecond,
            moduleStates[2].angle.getRadians(),
            moduleStates[2].speedMetersPerSecond,
            moduleStates[3].angle.getRadians(),
            moduleStates[3].speedMetersPerSecond
        };

        SmartDashboard.putNumberArray("realModuleStates", loggingState);
    }
}