// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.LauncherConstants;
import frc.robot.subsystems.Launcher.LauncherLift;

public class liftLauncherEnc extends Command {
  LauncherLift m_launcherLift;
  double enc;
  /** Creates a new liftLauncherEnc. */
  public liftLauncherEnc(LauncherLift sentLauncherLift, double sentEnc) {
    m_launcherLift = sentLauncherLift;
    enc = sentEnc;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcherLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcherLift.runLift(LauncherConstants.launcherLiftSpeedMultiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcherLift.runLift(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_launcherLift.getThroughBoreEncoder() > enc && m_launcherLift.getThroughBoreEncoder() < LauncherConstants.liftUpLimitForLiftLauncherEnc) {
      return true;
    } else {
      return false;
    }
  }
}
