// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Operator Controller Port IDs
    public static final int DriverControllerPort = 0;
    public static final int ManipulatorControllerPort = 1;
  }

  public static class RobotConstants {
    // Robot loop time
    public static final int loopTime_ms = 20;
  }

  public static class DrivetrainConstants {
    // Robot Dimensions
    public static final double robotLength_X_in = 29.0;
    public static final double robotWidth_Y_in = 29.0;
    public static final double robotLength_X_m = Units.inchesToMeters(robotLength_X_in);
    public static final double robotWidth_Y_m = Units.inchesToMeters(robotWidth_Y_in);

    // Module Dimensions
    public static final double moduleToModuleLength_X_in = 26.25;
    public static final double moduleToModuleWidth_Y_in = 26.25;
    public static final double moduleToModuleLength_X_m = Units.inchesToMeters(moduleToModuleLength_X_in);
    public static final double moduleToModuleWidth_Y_m = Units.inchesToMeters(moduleToModuleWidth_Y_in);

    // Wheel Specs
    public static final double wheelDiameter_in = 4.0;
    public static final double wheelDiameter_m = Units.inchesToMeters(wheelDiameter_in);
    public static final double wheelRadius_in = wheelDiameter_in / 2.0;
    public static final double wheelRadius_m = Units.inchesToMeters(wheelRadius_in);

    // Drive Gear Ratio Specs
    public static final double driveGearRatio = 6.12;

    // Azimuth Gear Ratio Specs
    public static final double azimuthGearRatio = 150.0 / 7.0;

    // Robot Speed and Rotation Specs
    public static final double maxGroundSpeed_mPs = 5.0;
    public static final double maxRotateSpeed_radPs = maxGroundSpeed_mPs / Math.hypot(moduleToModuleLength_X_m, moduleToModuleWidth_Y_m);
  }
}