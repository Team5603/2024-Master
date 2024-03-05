// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher.LauncherLift;

public class liftLauncher extends Command {
  LauncherLift m_launcherLift;
  DoubleSupplier speed;
  /** Creates a new angleLauncher. */
  public liftLauncher(LauncherLift sentLaucher, DoubleSupplier sentSpeed) {
    m_launcherLift = sentLaucher;
    speed = sentSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcherLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double finalSpeed = speed.getAsDouble();
    m_launcherLift.runLift(finalSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcherLift.runLift(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
