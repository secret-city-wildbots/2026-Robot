package frc.robot.Actors.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// Import WPILib Libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

// Import Actors, Utils & Constants
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.LimelightHelpers.RawFiducial;

import java.util.function.Supplier;
import java.util.function.DoubleSupplier;
import frc.robot.Constants.VisionConstants;




public class Vision extends SubsystemBase {

    // This is a scored pose record to help compare the camera readings to determine
    // the best position
    private record ScoredPose(
            LimelightHelpers.PoseEstimate pose,
            double avgAmbiguity,
            double avgDistance) {

    }

    // Used to define a rectangular vision zone on field 
    // Since coordinates are relative to blue-origin whether were on red or blue side
    // We only need to take in the value of the blue and red AprilTags    
    private record VisionZone(
        double xMin,
        double xMax,
        double yMin,
        double yMax,
        int[] blueTags,
        int[] redTags
    ) {}
    // These are suppliers needing to be fed when instantiated. These will help to
    // align the cameras indiviually with each
    // heading and rotation speeds in more real time
    private final DoubleSupplier headingSupplier;
    private final DoubleSupplier omegaRpsSupplier;
    private final Supplier<Pose2d> poseSupplier;

    /*
    TODO: Add all Zones
    Limelight Zoning disabling all other tags other 
    then the ones listed in that zone 
    */
    private static final VisionZone[] StructureZones = new VisionZone[] {
        // Right Trench Zone:
        new VisionZone(3.0, 6.0, 0.0, 1.25, new int[] {17, 28}, new int[] {12, 1}),

        // Left Trench Zone:
        new VisionZone(3.0, 6.0, 6.75, 8.0, new int[] {22, 23}, new int[] {7, 6})
    };
    /*
     * Constructor for vision
     */
    public Vision(DoubleSupplier headingSupplier, DoubleSupplier omegaRpsSupplier, Supplier<Pose2d> poseSupplier) {
        this.headingSupplier = headingSupplier;
        this.omegaRpsSupplier = omegaRpsSupplier;
        this.poseSupplier = poseSupplier;
    }

    /*
     * Returns true if the robot is currently on the red alliance.
     */
    private boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    /*
     * Checks whether a pose is inside a given zone.
     * Since your zones are fixed field locations, there is NO mirroring here.
     */
    private boolean isPoseInZone(Pose2d pose, VisionZone zone) {
        double x = pose.getX();
        double y = pose.getY();

        return x >= zone.xMin()
                && x <= zone.xMax()
                && y >= zone.yMin()
                && y <= zone.yMax();
    }

    /*
     * Figures out which AprilTag IDs should be allowed right now based on:
     * 1. current odometry pose
     * 2. which zone the robot is in
     * 3. alliance color
     *
     * Returns:
     * - the zone's blue tag list if on blue
     * - the zone's red tag list if on red
     * - null if not inside any zone, which clears the filter override
     */
    private int[] getAllowedTagsForCurrentZone() {
        Pose2d currentPose = poseSupplier.get();
        boolean redAlliance = isRedAlliance();
        
        for (VisionZone zone : StructureZones) {
            if (isPoseInZone(currentPose, zone)) {
                return redAlliance ? zone.redTags() : zone.blueTags();
            }
        }
        
        return null;
    }   

    /*
     * Pushes the currently allowed tag list to every Limelight.
     * This should happen BEFORE reading pose estimates, so each Limelight
     * only solves pose using the tags that make sense for the current zone.
     */
    private void updateTagFilters() {
        int[] allowedTags = getAllowedTagsForCurrentZone();
        
        for (String limelightName : VisionConstants.limelightNames) {
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, allowedTags);
        }
    }

    public LimelightHelpers.PoseEstimate[] getPoses() {
        // Update Limelight tag filters
        updateTagFilters();

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
            if (pose == null) {
                continue;
            }
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

            meant2d = meant2d.plus(pose.pose.getTranslation().times(weight));
            meanr2d = meanr2d.plus(pose.pose.getRotation().times(weight));

            totalt2d += weight;
            totalr2d += weight;

            if (weight > highestWeight) {
                highestWeight = weight;
                timestamp = pose.timestampSeconds;
            }
        }

            if (totalt2d == 0.0 || totalr2d == 0.0) {
                return null;
            }

        meant2d = meant2d.div(totalt2d);
        meanr2d = meanr2d.div(totalr2d);

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
            // 1) closer tags
            // 2) lower ambiguity
            // 3) more tags
            if (
                scored.avgDistance < best.avgDistance ||
                (scored.avgDistance < best.avgDistance && scored.avgAmbiguity < best.avgAmbiguity) ||
                (scored.avgDistance < best.avgDistance && scored.avgAmbiguity < best.avgAmbiguity && pose.tagCount > best.pose.tagCount)
            ) {
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