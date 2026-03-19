package frc.robot.Utils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.TurretConstants;

public class ShotPredictor {

    private static final Translation2d hubPosition = new Translation2d(11.9, 4.035);
    private static final Translation2d bumpLeft = new Translation2d(4.35, 5.8);
    private static final Translation2d bumpRight = new Translation2d(4.35, 8-5.8);

    private static final double g = 9.81;

    /**
     * Output of ShotPredictor: contains desired angle, velocity and speed.
     */
    public static class Shot {

        public Rotation2d yaw;
        public Rotation2d tilt;
        public double velocity_mPs;
        public double airtime_s;
    }

    /**
     * Predict a shot given
     *
     * @param robotPoseSupplier The pose of the robot (continuous supplier)
     * @param robotVelSupplier The velocity of the robot (continuous
     * supplier)(robot relative)
     */
    public static Shot predict(Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelSupplier) {

        Shot shot = new Shot();

        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d robotPos = robotPose.getTranslation();
        Rotation2d robotRot = robotPose.getRotation();
        ChassisSpeeds robotVel = robotVelSupplier.get();

        Translation2d targetPos;

        if (robotPos.getX() < 4.25) {
            targetPos = hubPosition;
        } else if (robotPos.getY() > 4.0) {
            targetPos = bumpLeft;
        } else {
            targetPos = bumpRight;
        }
        targetPos = hubPosition;

        Translation2d turretPos = robotPos.plus(TurretConstants.turretPos.rotateBy(robotRot));

        double distance = targetPos.getDistance(turretPos);
        double airtime = getAirtime(distance);

        shot.airtime_s = airtime;

        // predict robot future pos during flight (accounts for movement, allowing empirical calculations where there would otherwise be too many variables)
        Translation2d futureRobotPos = robotPos.plus(
                new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond).times(airtime));
        Rotation2d futureRobotRot = robotRot.plus(
                new Rotation2d(robotVel.omegaRadiansPerSecond).times(airtime));
        Translation2d futureTurretPos = futureRobotPos.plus(TurretConstants.turretPos.rotateBy(futureRobotRot));

        // yaw calculation
        Translation2d delta = targetPos.minus(futureTurretPos);
        Rotation2d fieldAngleToTarget = delta.getAngle();
        shot.yaw = fieldAngleToTarget.minus(futureRobotRot);

        // horizontal distance (adjusted by airtime)
        double futureDist = delta.getNorm();

        shot.velocity_mPs = getVelocity(futureDist);
        shot.tilt = getTilt(futureDist);

        return shot;
    }

    public static double getVelocity(double dist) {
        return HubShooterTrajectoryCalc.lookupCache(dist).speed_rps;
        //return 1.0;
    }

    public static Rotation2d getTilt(double dist) {
        return new Rotation2d((90-HubShooterTrajectoryCalc.lookupCache(dist).elevation_deg)/180*Math.PI);
        //return new Rotation2d();
    }

    public static double getAirtime(double dist) {
        return HubShooterTrajectoryCalc.lookupCache(dist).airtime_s;
        //return TurretConstants.turretBaseAirtime_s
        //        + TurretConstants.turretDistAirtime_sPm * dist;
    }
}