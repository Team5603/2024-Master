// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Launcher.Launcher;

public class launcherIntake extends Command {
  Launcher m_launcher;
  Intake m_intake;
  double speed;
  boolean reverse;
  boolean end;
  boolean sensorStatus;

  /** Creates a new launcherIntake. */
  public launcherIntake(Launcher sentLauncher, Intake sentIntake, double sentSpeed, boolean sentReverse) {
    m_launcher = sentLauncher;
    m_intake = sentIntake;
    speed = sentSpeed;
    reverse = sentReverse;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensorStatus = m_launcher.getSensor();
    if (reverse) {
      m_launcher.runMotors(-speed);
    } else {
      m_launcher.runMotors(speed);
    }

    m_intake.runIntake(IntakeConstants.intakeSpeed, true);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.runMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!sensorStatus) {
      return false;
    } else {
      return true;
    }
  }
}
