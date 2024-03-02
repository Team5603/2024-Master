// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSparks;

public class spinin extends Command {
  ClawSparks m_ClawSparks;
  DoubleSupplier m_LeftSpin;
  DoubleSupplier m_RightSpin;

  /** Creates a new spinin. */
  public spinin(ClawSparks sentClawSparks, DoubleSupplier sentLeftSpin, DoubleSupplier sentRightSpin) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_ClawSparks = sentClawSparks;
    addRequirements(m_ClawSparks);
    m_LeftSpin = sentLeftSpin;
    m_RightSpin = sentRightSpin;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClawSparks.work(m_LeftSpin.getAsDouble(), m_RightSpin.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setDefaultCommand(spinin spinin) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDefaultCommand'");
  }
}
