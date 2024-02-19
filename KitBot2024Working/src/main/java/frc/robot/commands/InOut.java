// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class InOut extends Command {

  Shooter m_Shooter;
  double m_SpinSpeed; //Motor speed for both

  /** Creates a new InOut. */
 public InOut(Shooter sentShooter, double sentLSpinSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = sentShooter;
    addRequirements(m_Shooter);
    m_SpinSpeed = sentLSpinSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.shoot(m_SpinSpeed, m_SpinSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.shoot(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
