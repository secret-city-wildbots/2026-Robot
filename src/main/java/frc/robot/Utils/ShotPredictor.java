package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotPredictor {
    public static double blah;

    /**
     * Output of ShotPredictor: contains desired angle, velocity and speed.
     */
    public static class Shot {
        // turret angle
        public Rotation2d yaw;
        // vertical angle (0-45)
        public Rotation2d tilt;

        public double velocity_mPs;
    }

    /**
     * Predict a shot given ROBOT RELATIVE hub pos and chassis velocity
     */
    public static Shot predict(Pose2d robotPos, ChassisSpeeds velocity) {
        Shot shot = new Shot();

        shot.yaw = new Rotation2d(0);
        shot.tilt = new Rotation2d(45);
        shot.velocity_mPs = 5;

        return shot;
    }
}