package frc.robot.States;

import edu.wpi.first.math.geometry.Rotation2d;

public class DrivetrainState {
    // state
    public double currentDriveSpeed_mPs;
    public Rotation2d currentAngle_rad;

    public void updateState(double currentDriveSpeed_mPs, Rotation2d currentAngle_rad) {
        this.currentDriveSpeed_mPs = currentDriveSpeed_mPs;
        this.currentAngle_rad = currentAngle_rad;
    }
}
