// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class autonRobotCentricDrive extends Command {
  Swerve m_swerve;
  double seconds;
  double forwardSpeed;
  double sidewaysSpeed;

  Timer timer;

  SwerveRequest.RobotCentric driveForward;

  /** Creates a new limelightRobotCentric. */
  public autonRobotCentricDrive(Swerve sentSwerve, double sentForwardSpd, double sentSidewaysSpd, double sentSeconds) {
    m_swerve = sentSwerve;
    forwardSpeed = sentForwardSpd;
    sidewaysSpeed = sentSidewaysSpd;
    seconds = sentSeconds;

    timer = new Timer();

    forwardSpeed = 0;
    sidewaysSpeed = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_swerve.applyRequest(()-> driveForward.withVelocityX(forwardSpeed).withVelocityY(sidewaysSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > seconds) {
      return true;
    } else {
      return false;
    }
  }
}
