package frc.robot.Actors.Subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.Limelight;
import frc.robot.Utils.LimelightHelpers;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final List<Limelight> limelights = new ArrayList<>();

    public Vision() {
        for (String limelightID : VisionConstants.limelightNames) {
            limelights.add(new Limelight(limelightID));
        }
    }

    public void getLimelightFrontPose() {
        // "limelight-front", "limelight-back", "limelight-left", "limelight-right"
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-front");
        System.out.println("Front-pose: " + mt1.pose);
        System.out.println("Front-timestamp(S): " + mt1.timestampSeconds);
    }

    public void getLimelightBackPose() {
        // "limelight-front", "limelight-back", "limelight-left", "limelight-right"
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-back");
        System.out.println("Back-pose: " + mt1.pose);
        System.out.println("Back-timestamp(S): " + mt1.timestampSeconds);
    }

    public void getLimelightLeftPose() {
        // "limelight-front", "limelight-back", "limelight-left", "limelight-right"
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-left");
        System.out.println("Left-pose: " + mt1.pose);
        System.out.println("Left-timestamp(S): " + mt1.timestampSeconds);
    }

    public void getLimelightRightPose() {
        // "limelight-front", "limelight-back", "limelight-left", "limelight-right"
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-right");
        System.out.println("Right-pose: " + mt1.pose);
        System.out.println("Right-timestamp(S): " + mt1.timestampSeconds);
    }

    @Override
    public void periodic() {
        this.getLimelightBackPose();
        this.getLimelightFrontPose();  
        this.getLimelightRightPose();
        this.getLimelightLeftPose();  
    }
}
