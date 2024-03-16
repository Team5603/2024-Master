// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.LauncherConstants;
import frc.robot.subsystems.Launcher.Launcher;

public class stopLauncher extends Command {
  Launcher m_launcher;
  /** Creates a new stopLauncher. */
  public stopLauncher(Launcher sentLauncher) {
    m_launcher = sentLauncher;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.runMotors(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_launcher.getMotorSpeeds() > 0) {
    //   return false;
    // } else {
    //   return true;
    // }
    return true;

  }
}
