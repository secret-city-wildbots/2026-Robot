package frc.robot.Actors.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Actors, Utils & Constants
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.LimelightHelpers.RawFiducial;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    public record FusedVisionResult(Pose2d pose, double tiemstamp) {}

    // This is a scored pose record to help compare the camera readings to determine the best position
    private record ScoredPose(
        LimelightHelpers.PoseEstimate pose,
        double lowestDist
    ) {}

    // These are suppliers needing to be fed when instantiated. These will help to align the cameras indiviually with each
    // heading and rotation speeds in more real time
    private final DoubleSupplier headingSupplier;
    private final DoubleSupplier omegaRpsSupplier;
    private final Supplier<Rotation2d> rotation2dSupplier;

    /*
     * Constructor for vision
     */
    public Vision(DoubleSupplier headingSupplier, DoubleSupplier omegaRpsSupplier, Supplier<Rotation2d> rotation2dSupplier) {
        this.headingSupplier = headingSupplier;
        this.omegaRpsSupplier = omegaRpsSupplier;
        this.rotation2dSupplier = rotation2dSupplier;
    }

    /*
     * Get the most accurate pose from all of the limelights
     */
    public LimelightHelpers.PoseEstimate getBestPose() {
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

        // initialize a variable to hold the best pose
        ScoredPose best = null;

        // Loop through all of the poses to determine the best one
        for (var pose : poses) {
            // If the pose is null skip this instance
            if (pose == null) continue;
            // If the pose is not a MegaTag2 skip this instance
            if (!pose.isMegaTag2) continue;
            // If the pose has 0 tag counts skip this instance
            if (pose.tagCount == 0) continue;
            // If we are spinning faster than 720 deg / sec skip this instance
            if (Math.abs(omegaRpsSupplier.getAsDouble()) > 720) continue;

            // Score the pose estimate
            var scored = score(pose);

            // Check if our best estimate is null, if null set it to the scored pose move on to the next instance
            if (best == null) {
                best = scored;
                continue;
            }

            // Prefer:
            // 1) closer tags
            if (
                scored.lowestDist < best.lowestDist
                && scored.lowestDist < 2) {
                // Set the best to the currently better scored camera
                best = scored;
            }
        }

        // If best is not null return the pose else return null
        return best != null ? best.pose : null;
    }

    public FusedVisionResult fuseFourLimelights() {
        double sumX = 0, sumY = 0, sumWeight = 0;

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

        // Loop through all of the poses to determine the best one
        for (var pose : poses) {
            if (pose.tagCount == 0) continue;

            // Weight = 1 / variance = 1 / (k * distance)²
            double dist = pose.avgTagDist;
            double weight = 1.0 / (dist * dist);

            // Scale down if only 1 tag
            if (pose.tagCount == 1) weight *= 0.5;

            sumX += pose.pose.getX() * weight;
            sumY += pose.pose.getY() * weight;
            sumWeight += weight;
        }

        if (sumWeight == 0) return null; // No valid estimates

        Pose2d fusedPose = new Pose2d(
            sumX / sumWeight,
            sumY / sumWeight,
            this.rotation2dSupplier.get() // Trust gyro for heading
        );

        return new FusedVisionResult(fusedPose, poses[VisionConstants.limelightNames.length - 1].timestampSeconds);
    }

    /*
     * returns all the poses from all of the cameras
     */
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

    /*public LimelightHelpers.PoseEstimate[] getTurretPoses() {
        LimelightHelpers.PoseEstimate[] poses = getPoses();

        int[] hubIDs = new int[] {
            2,3,4,5,8,9,10,11,18,19,20,21,24,25,26
        };

        for (LimelightHelpers.PoseEstimate pose: poses) {
            LimelightHelpers.RawFiducial[] rawFs = pose.rawFiducials;

            for (LimelightHelpers.RawFiducial rawF: rawFs) {
                boolean good = false;
                for (int id: hubIDs) {
                    if (rawF.id == id) {
                        good = true;
                    }
                }
                //TODO finish that
            }
        }
    }*/

    /*
     * Get the meanPose from all of the cameras
     */
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
     * Takes in a LimelightHelpers.PoseEstimate and returns a scored value based on the information. We average the ambiguity and distance
     * values and return the scored pose with mt2, average ambiguity and average distance.
     */
    private ScoredPose score(LimelightHelpers.PoseEstimate mt2) {
        // Initialize the ambiguity and distance
        double lowestDist = 999999999.9;

        // Loop through all of the raw fiducial data and add them all up
        for (var rawF : mt2.rawFiducials) {
            if (rawF.distToRobot < lowestDist) lowestDist = rawF.distToRobot;
        }

        // Return a new scored pose
        return new ScoredPose(mt2, lowestDist);
    }

    @Override
    public void periodic() {}
}