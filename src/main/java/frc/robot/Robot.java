// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import WPI Libraries
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;

// Import Limelight Utils
import frc.robot.Utils.LimelightHelpers;

// Import Subsystems
// Vision was put here for now so we can utilize the periodic loops
// We can look to refactor in the future if people want :)
import frc.robot.Actors.Subsystems.Vision;  


/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation. If you change the name of
 * this class or the
 * package after creating this project, you must also update the Main.java file
 * in the project.
 */
public class Robot extends TimedRobot {

  // Set Variables
  private Command autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final Vision vision;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // Reset the Pigeon2 on the drivetrain.
    // TODO: is this needed?
    m_robotContainer.drivetrain.getPigeon2().reset();

    // Setup vision with the suppliers from the drivetrain (heading and rotation (rps))
    // This allows each limelight to be as accurate as possible when being setup
    vision = new Vision(
      () -> m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees(),
      () -> Units.radiansToRotations(m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond)
    );
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Get the best pose estimate from all of the cameras
    LimelightHelpers.PoseEstimate bestPose = vision.getBestPose();

    // If bestPose is not null, add vision measurement to the drivetrain
    // TODO: need to tune 0.7,0.7 values
    if (bestPose != null) {
      // TODO: Do we want to just only add or reset the whole pose?
      m_robotContainer.drivetrain.addVisionMeasurement(bestPose.pose, bestPose.timestampSeconds, VecBuilder.fill(0.7,0.7,9999999));
      //m_robotContainer.drivetrain.resetPose(bestPose.pose);
    }
    // TODO: Printing pose
    // System.out.println(m_robotContainer.drivetrain.getState().Pose);
  }

  @Override
  public void autonomousInit() {
    // TODO: Setup the configuration and selection of the autonomous command
    // TODO: Uncomment the line below for the setup and put in the proper code to
    // make it function correctly.
    autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // The below code is used to set the real pos of the robot from the front limelight. In the competition we need to make sure
    // the limelight can see a tag when disabled

    // Get front limelight pose
    LimelightHelpers.PoseEstimate LLFrontPose = vision.getLimelightFrontPosemt1();

    // If the pose is not null and it sees an april tag
    if (LLFrontPose != null && LLFrontPose.tagCount > 0) {
      // Reset the robots rotation and pose directly
      m_robotContainer.drivetrain.resetRotation(LLFrontPose.pose.getRotation());
      m_robotContainer.drivetrain.resetPose(LLFrontPose.pose);
    }
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}