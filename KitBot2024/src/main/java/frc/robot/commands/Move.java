// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class Move extends Command {
  DriveTrain m_DriveTrain;
  DoubleSupplier m_LSpeed;
  DoubleSupplier m_RSpeed;

  /** Creates a new Move. */
  public Move(DriveTrain sentDriveTrain, DoubleSupplier sentLSpeed, DoubleSupplier sentRSpeed) {
    m_DriveTrain = sentDriveTrain;

    addRequirements(m_DriveTrain);

    m_LSpeed = sentLSpeed;
    m_RSpeed = sentRSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_DriveTrain.Drive(m_LSpeed.getAsDouble(), m_RSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.Drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
