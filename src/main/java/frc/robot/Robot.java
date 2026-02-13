// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Import External Libraries
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Actors.Subsystems.Vision;
import frc.robot.Utils.LimelightHelpers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation. If you change the name of
 * this class or the
 * package after creating this project, you must also update the Main.java file
 * in the project.
 */
public class Robot extends TimedRobot {

  private Command autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Vision vision;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.drivetrain.getPigeon2().reset();

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
    LimelightHelpers.PoseEstimate LLFrontPose = vision.getLimelightFrontPosemt1();
    if (LLFrontPose != null && LLFrontPose.tagCount > 0) {
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