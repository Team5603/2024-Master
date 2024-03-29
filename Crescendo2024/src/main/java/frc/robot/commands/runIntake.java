// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.subsystems.Intake.Intake;

public class runIntake extends Command {
  Intake m_intake;
  double speed;
  boolean reverse;
  /** Creates a new runIntake. */
  public runIntake(Intake sentIntake, double sentSpeed, boolean sentReverse) {
    m_intake = sentIntake;
    speed = sentSpeed;
    reverse = sentReverse;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // changed to reverse from !reverse 3/16/24
    if (reverse) {
      speed = IntakeConstants.outakeSpeed;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runIntake(speed, reverse);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runIntake(0, reverse);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
