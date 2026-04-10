package frc.robot.Utils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShotPredictor {

    /**
     * Output of ShotPredictor: contains desired angle, velocity and speed.
     */
    public static class Shot {

        public Rotation2d yaw;
        public Rotation2d tilt;
        public double velocity_rPs;
        public double airtime_s;
        public Translation2d robotPos;
        public Rotation2d robotRot;
        public ChassisSpeeds robotVel;
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

        //Target poses
        double targetX = (DriverStation.getAlliance().get() == Alliance.Blue) ? (4.63-2.0):(11.9+2.0); //?
        double hubX = (DriverStation.getAlliance().get() == Alliance.Blue) ? (4.63):(11.9);
        Translation2d hubPosition = new Translation2d(hubX, 4.035);
        Translation2d bumpLeft = new Translation2d(targetX, 6);
        Translation2d bumpRight = new Translation2d(targetX, 8-6);

        Shot shot = new Shot();

        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d robotPos = robotPose.getTranslation();
        Rotation2d robotRot = robotPose.getRotation(); //?
        ChassisSpeeds robotVel = robotVelSupplier.get();

        Translation2d targetPos;

        boolean lobbing = false;

        if (((DriverStation.getAlliance().get() == Alliance.Blue) ? (robotPos.getX() < hubX):(robotPos.getX() > hubX))) { //?
            targetPos = hubPosition;
        } else if (robotPos.getY() > 4.0) {
            targetPos = bumpLeft;
            lobbing = true;
        } else {
            lobbing = true;
            targetPos = bumpRight;
        }

        Translation2d turretPos = robotPos.plus(TurretConstants.turretPos.rotateBy(robotRot));

        double distance = targetPos.getDistance(turretPos);
        //System.out.println(distance);
        double airtime = getAirtime(distance);

        shot.airtime_s = airtime;

        // predict robot future pos during flight (accounts for movement, allowing empirical calculations where there would otherwise be too many variables)
        Translation2d futureRobotPos = robotPos.plus(
                new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond).times(airtime));
        Rotation2d futureRobotRot = robotRot.plus(
                new Rotation2d(robotVel.omegaRadiansPerSecond).times(0.0));//airtime));
        Translation2d futureTurretPos = futureRobotPos.plus(TurretConstants.turretPos.rotateBy(futureRobotRot));

        // yaw calculation
        Translation2d delta = targetPos.minus(futureTurretPos);
        Rotation2d fieldAngleToTarget = delta.getAngle();
        shot.yaw = fieldAngleToTarget.minus(futureRobotRot);

        // horizontal distance (adjusted by airtime)
        double futureDist = delta.getNorm();

        shot.velocity_rPs = getVelocity(futureDist);
        shot.tilt = (lobbing) ? new Rotation2d((40)/180*Math.PI):getTilt(futureDist);

        return shot;
    }

    public static double getVelocity(double dist) {
        return (1.456*(dist-2.0) + 50.2 + ((dist > 3.0) ? 3.7*(dist-3.0):0.0));
        //return 1.0;
    }

    public static Rotation2d getTilt(double dist) {
        return new Rotation2d((90-(Math.pow(0.475086, dist-4.67884)+63+(-1.37205*dist)))/180*Math.PI);
        //return new Rotation2d();
    }

    public static double getAirtime(double dist) {
        return 0.177067*dist + 0.546781;
        //return 0.0;//HubShooterTrajectoryCalc.lookupCache(dist).airtime_s;
        //return TurretConstants.turretBaseAirtime_s
        //        + TurretConstants.turretDistAirtime_sPm * dist;
    }
}