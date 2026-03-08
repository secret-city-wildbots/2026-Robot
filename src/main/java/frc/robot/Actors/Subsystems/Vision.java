package frc.robot.Actors.Subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.LimelightHelpers.RawFiducial;

public class Vision extends SubsystemBase {

    // This is a scored pose record to help compare the camera readings to determine
    // the best position
    private record ScoredPose(
            LimelightHelpers.PoseEstimate pose,
            double avgAmbiguity,
            double avgDistance) {

    }

    // These are suppliers needing to be fed when instantiated. These will help to
    // align the cameras indiviually with each
    // heading and rotation speeds in more real time
    private final DoubleSupplier headingSupplier;
    private final DoubleSupplier omegaRpsSupplier;

    /*
     * Constructor for vision
     */
    public Vision(DoubleSupplier headingSupplier, DoubleSupplier omegaRpsSupplier) {
        this.headingSupplier = headingSupplier;
        this.omegaRpsSupplier = omegaRpsSupplier;
    }

    public LimelightHelpers.PoseEstimate[] getPoses() {
        // Create an array of poses for our cameras
        LimelightHelpers.PoseEstimate[] poses = new LimelightHelpers.PoseEstimate[VisionConstants.limelightNames.length];

        // Loop through each camera name to setup the camera to pull data
        for (int index = 0; index < VisionConstants.limelightNames.length; index++) {
            // Get the camera name
            String limelightID = VisionConstants.limelightNames[index];
            // Set the camera to the robot orientation
            LimelightHelpers.SetRobotOrientation(limelightID, this.headingSupplier.getAsDouble(), 0, 0, 0, 0, 0);
            // Get the pose of the camera
            poses[index] = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightID);
        }

        return poses;
    }

    public LimelightHelpers.PoseEstimate getMeanPose() {
        LimelightHelpers.PoseEstimate[] poses = this.getPoses();

        Translation2d meant2d = new Translation2d();
        double totalt2d = 0.0;
        Rotation2d meanr2d = new Rotation2d();
        double totalr2d = 0.0;

        double timestamp = 0.0;
        double highestWeight = 0.0;

        for (LimelightHelpers.PoseEstimate pose : poses) {
            if (!pose.isMegaTag2) {
                continue;
            }
            if (pose.tagCount == 0) {
                continue;
            }

            // change this to however we want to pick whats best
            double weight = 0.0;

            for (RawFiducial rawF : pose.rawFiducials) {
                weight += 1 / rawF.distToCamera;
            }

            meant2d.plus(pose.pose.getTranslation().times(weight));
            meanr2d.plus(pose.pose.getRotation().times(weight));

            totalt2d += weight;
            totalr2d += weight;

            if (weight > highestWeight) {
                timestamp = pose.timestampSeconds;
            }
        }

        meant2d.div(totalt2d);
        meanr2d.div(totalr2d);

        Pose2d mean = new Pose2d(meant2d, meanr2d);

        LimelightHelpers.PoseEstimate poseEst = new LimelightHelpers.PoseEstimate();
        poseEst.pose = mean;
        poseEst.timestampSeconds = timestamp;
        return poseEst;
    }

    /*
     * Get the most accurate pose from all of the limelights
     */
    public LimelightHelpers.PoseEstimate getBestPose() {
        LimelightHelpers.PoseEstimate[] poses = this.getPoses();

        // initialize a variable to hold the best pose
        ScoredPose best = null;

        // Loop through all of the poses to determine the best one
        for (var pose : poses) {
            // If the pose is null skip this instance
            if (pose == null) {
                continue;
            }
            // If the pose is not a MegaTag2 skip this instance
            if (!pose.isMegaTag2) {
                continue;
            }
            // If the pose has 0 tag counts skip this instance
            if (pose.tagCount == 0) {
                continue;
            }
            // If we are spinning faster than 720 deg / sec skip this instance
            if (Math.abs(omegaRpsSupplier.getAsDouble()) > 720) {
                continue;
            }

            // Score the pose estimate
            var scored = score(pose);

            // Check if our best estimate is null, if null set it to the scored pose move on
            // to the next instance
            if (best == null) {
                best = scored;
                continue;
            }

            // Prefer:
            // 1) more tags
            // 2) lower ambiguity
            // 3) closer tags
            if (pose.tagCount > best.pose.tagCount
                    || (pose.tagCount == best.pose.tagCount && scored.avgAmbiguity < best.avgAmbiguity)
                    || (pose.tagCount == best.pose.tagCount
                    && scored.avgAmbiguity == best.avgAmbiguity
                    && scored.avgDistance < best.avgDistance)) {
                // Set the best to the currently better scored camera
                best = scored;
            }
        }

        // If best is not null return the pose else return null
        return best != null ? best.pose : null;
    }

    /*
     * Get the Pose of the Front LimeLight
     */
    public LimelightHelpers.PoseEstimate getLimelightFrontPose() {
        // access limelight-front
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        // return mt2
        return mt2;
    }

    /*
     * Get the Pose of the Back LimeLight
     */
    public LimelightHelpers.PoseEstimate getLimelightBackPose() {
        // access limelight-back
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
        // return mt2
        return mt2;
    }

    /*
     * Get the Pose of the Left LimeLight
     */
    public LimelightHelpers.PoseEstimate getLimelightLeftPose() {
        // access limelight-left
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
        // return mt2
        return mt2;
    }

    /*
     * Get the Pose of the Right LimeLight
     */
    public LimelightHelpers.PoseEstimate getLimelightRightPose() {
        // access limelight-right
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
        // return mt2
        return mt2;
    }

    // Getting values in mt1 for configuring pigeon to mt1 yaw

    /*
     * Get the Pose of the Front LimeLight mt1
     */
    public LimelightHelpers.PoseEstimate getLimelightFrontPosemt1() {
        // access limelight-front
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
        // return mt1
        return mt1;
    }

    /*
     * Get the Pose of the Back LimeLight mt1
     */
    public LimelightHelpers.PoseEstimate getLimelightBackPosemt1() {
        // access limelight-back
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");
        // return mt2
        return mt1;
    }

    /*
     * Get the Pose of the Left LimeLight mt1
     */
    public LimelightHelpers.PoseEstimate getLimelightLeftPosemt1() {
        // access limelight-left
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        // return mt1
        return mt1;
    }

    /*
     * Get the Pose of the Right LimeLight mt1
     */
    public LimelightHelpers.PoseEstimate getLimelightRightPosemt1() {
        // access limelight-right
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
        // return mt1
        return mt1;
    }

    /*
     * Takes in a LimelightHelpers.PoseEstimate and returns a scored value based on
     * the information. We average the ambiguity and distance
     * values and return the scored pose with mt2, average ambiguity and average
     * distance.
     */
    private ScoredPose score(LimelightHelpers.PoseEstimate mt2) {
        // Initialize the ambiguity and distance
        double amb = 0.0;
        double dist = 0.0;

        // Loop through all of the raw fiducial data and add them all up
        for (var rawF : mt2.rawFiducials) {
            amb += rawF.ambiguity;
            dist += rawF.distToRobot;
        }

        // Average the results
        amb /= mt2.rawFiducials.length;
        dist /= mt2.rawFiducials.length;

        // Return a new scored pose
        return new ScoredPose(mt2, amb, dist);
    }

    @Override
    public void periodic() {
    }
}
