// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_swerve.getDaqThread().setThreadPriority(99);
  }
  private void useLimelight () {
    var lastResult = RobotLimelightHelpers.getLatestResults("limelight").targetingResults;
    Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

    if (!lastResult.valid) return;
    m_robotContainer.m_swerve.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();


    // useLimelight();
    // SmartDashboard.putNumber("Drive Right", m_robotContainer.driveController.getRightX());
    // SmartDashboard.putNumber("Drive Left", m_robotContainer.driveController.getLeftY());

    // SmartDashboard.putNumber("Manip Right", -m_robotContainer.manipulateController.getRightY());
    // SmartDashboard.putNumber("Manip Left", m_robotContainer.manipulateController.getLeftY());
  }
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
