// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher.Launcher;

public class shootNoteTimed extends Command {
  Launcher m_launcher;
  double speed;
  double seconds;
  Timer timer;
  /** Creates a new shootNote. */
  public shootNoteTimed(Launcher sentLauncher, double sentSpeed, double sentSeconds) {
    m_launcher = sentLauncher;
    speed = sentSpeed;
    seconds = sentSeconds;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.runMotors(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.runMotors(0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() >= seconds) {
      return true;
    } else {
      return false;
    }
  }
}
