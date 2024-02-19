// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class Shootyshoot extends Command {
  Shooter m_Shooter;
  double m_SpinSpeed, m_delay; // Motor speed for both
  Timer delayTimer;

  /** Creates a new Shootyshoot. */
  public Shootyshoot(Shooter sentShooter, double sentSpinSpeed, double delay) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = sentShooter;
    addRequirements(m_Shooter);
    m_SpinSpeed = sentSpinSpeed;
    m_delay = delay;
    delayTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delayTimer.reset();
    delayTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (delayTimer.get() >= m_delay) {
      m_Shooter.shoot(m_SpinSpeed, m_SpinSpeed);

    } else {
      m_Shooter.shoot(0, m_SpinSpeed);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.shoot(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
