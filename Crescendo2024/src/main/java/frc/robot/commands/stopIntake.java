// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class stopIntake extends Command {
  Intake m_intake;
  /** Creates a new stopIntake. */
  public stopIntake(Intake sentIntake) {
    m_intake = sentIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runIntake(0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_intake.getIntakeSpeed() > 0) {
    //   return false;
    // } else {
    //   return true;
    // }
    return true;
  }
}
