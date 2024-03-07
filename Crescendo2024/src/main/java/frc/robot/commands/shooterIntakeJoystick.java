// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Launcher.Launcher;

public class shooterIntakeJoystick extends Command {
  Intake m_intake;
  Launcher m_launcher;
  DoubleSupplier speed;
  /** Creates a new runIntakeJoystick. */
  public shooterIntakeJoystick(Intake sentIntake, Launcher sentLauncher, DoubleSupplier sentSpeed) {
    m_intake = sentIntake;
    m_launcher = sentLauncher;
    speed = sentSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.runMotors((-1 * speed.getAsDouble()) * .2);

    m_intake.runIntake((speed.getAsDouble()) * .2, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
