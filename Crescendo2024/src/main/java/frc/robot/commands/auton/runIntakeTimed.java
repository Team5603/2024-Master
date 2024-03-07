// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class runIntakeTimed extends Command {
  Intake m_intake;
  double seconds;
  boolean reverse;
  Timer timer;
  /** Creates a new runIntakeTimed. */
  public runIntakeTimed(Intake sentIntake, double sentSeconds, boolean sentReverse) {
    m_intake = sentIntake;
    seconds = sentSeconds;
    reverse = sentReverse;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runIntake(.4, reverse);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    m_intake.runIntake(0, false);
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
