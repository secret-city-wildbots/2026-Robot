package frc.robot.Utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Utils.LimelightHelpers.PoseEstimate;

public class Limelight {
    public String id;

    public Limelight(String id) {
        this.id = id;

        LimelightHelpers.setLimelightNTDouble(id, "Throttle", 100.0);
        LimelightHelpers.setPipelineIndex(id, 1);
    }

    public void update(boolean isEnabled, boolean disableFusion) {
        if (isEnabled || !disableFusion) {
            LimelightHelpers.setLimelightNTDouble(id, "Throttle", 0.0);
            LimelightHelpers.setPipelineIndex(id, 0);
        } else {
            LimelightHelpers.setLimelightNTDouble(id, "Throttle", 100.0);
            LimelightHelpers.setPipelineIndex(id, 1);
        }
    }

    /**
     * Gets an estimate for the current position of the robot using the limelight
     * 
     * @param IMURotation an optional containing the Rotation2d if the limelight
     *                    sees an april tag, otherwise it is empty
     */
    public Optional<PoseEstimate> getPosEstimate(Rotation2d IMURotation) {
        if (!DriverStation.getAlliance().isEmpty()) {
            if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
                LimelightHelpers.SetRobotOrientation(id, IMURotation.getDegrees(), 0, 0, 0, 0, 0);
            } else {
                LimelightHelpers.SetRobotOrientation(id, IMURotation.minus(Rotation2d.fromDegrees(180)).getDegrees(), 0,
                        0, 0, 0, 0);
            }
        }

        if (!DriverStation.getAlliance().isEmpty()) {
            PoseEstimate pos = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
                    ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(id)
                    : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(id);

            boolean estimateValid = LimelightHelpers.validPoseEstimate(pos);

            if (estimateValid)
                return Optional.of(pos);
        }

        return Optional.empty();
    }
}