// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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

  public static class VisionConstants {
    // Limelight Names
    //public static final String[] limelightNames = {"limelight-front", "limelight-back", "limelight-left", "limelight-right"};
    public static final String[] limelightNames = {"limelight-front", "limelight-left"}; //?
  }

  public static class IntakeConstants {
    // Intake motor CANBus IDs
    public static final int intakeMotorID = 40;
    public static final int extensionMotorID = 41;

    // Gear Ratios
    public static final double extensionGearRatio = 81.0;

    // Intake Ranges
    public static final double minDegree = 0.0;
    public static final double maxDegree = 86.0;
  }
  public static class ElevatorConstants {
    // Elevator motor CANBus IDs
    public static final int liftMotorID = 48;
    public static final int hookMotorID = 49;
    public static final double maxSpeedPercentage = 0.1;

    // Sensor ID / Ports
    public static final int lowerLimitMagneticSensorPort = 0;
    public static final int handoffMagneticSensorPort = 1;
    public static final int topLimitMagneticSensorPort = 2;

    // Hook Cancoder CANBus IDs
    public static final int hookMotorCancoderID = 34;
    public static final double hookEncoderOffset = 0.0794921875;

    // Hook motor positions
    public static final double hookSafePosition = 0.0;
    public static final double hookGuideDeployedPosition = 0.1;
    public static final double hookDeployedPosition = 0.18;
    public static final double hookPosForTopRungClearance = 43.56;
    public static final double angleTolerance = 2.0; 
  }

  public static class SpindexerConstants {
    // CANBus IDs
    public static final int spinMotorID = 42;
    public static final int transferMotorID = 43;

    // Properties
    public static final int transferRPS = 70; //?
    public static final int spindexerRPS = 10;
    public static final double spinupTime = 0.1;
  }

  public static class ShooterConstants {
    // CANBus IDs
    public static final int leadMotorID = 46;
    public static final int followMotorID = 47;
    public static final int hoodMotorID = 44;

    // Gear Ratios
    public static final double hoodGearRatio = 23.5714;

    // Hood Ranges
    public static final double minDegree = 5.0;
    public static final double maxDegree = 55.0;

    public static final double wheelRadius_m = 0.05; //abt 2 in
  }

  public static class TurretConstants {
    // CANBus IDs
    public static final int turretMotorID = 45;
    //public static final int encoderID = 44;
    public static final double turretBaseAirtime_s = 0.0;
    public static final double turretDistAirtime_sPm = 0.0; // Additional airtime per meter of distance to target
    public static final double turretGearRatio = (114.0/11.0)*5.0; //??
    public static final double posExtension = 480/360*turretGearRatio; //??
    public static final double negExtension = -185/360*turretGearRatio; //??

    public static final Translation2d turretPos = new Translation2d(-0.133,-0.02413);
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