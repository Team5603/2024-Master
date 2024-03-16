// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.constants.GeneralConstants.LauncherConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Launcher.Launcher;

public class releaseNoteShootNoteAuton extends Command {
  Intake m_intake;
  Launcher m_launcher;
  double delay;
  double endTime;
  boolean shootHold;
  Timer m_timer;

  /** Creates a new releaseNoteShootNoteAuton. */
  public releaseNoteShootNoteAuton(Intake sentIntake, Launcher sentLauncher, double sentDelay, boolean sentShootHold, double timeToRun) {
    m_intake = sentIntake;
    m_launcher = sentLauncher;
    delay = sentDelay;
    shootHold = sentShootHold;
    endTime = timeToRun;
    m_timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.runMotors(LauncherConstants.launcherSpeedLauncher);

    if (m_timer.get() > delay) {
      m_intake.runIntake(1, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!shootHold) {
      m_launcher.runMotors(0);
    }
    m_intake.runIntake(0, false);
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() > endTime) {
      return true;
    } else {
      return false;
    }
  }
}
