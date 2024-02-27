// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class liftIntake extends Command {
  Intake m_intake;
  DoubleSupplier speed;
  /** Creates a new liftIntake. */
  public liftIntake(Intake intake, DoubleSupplier inputSpeed) {
    m_intake = intake;
    speed = inputSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (speed.getAsDouble() == 0) {
      m_intake.liftPIDIntake();
      SmartDashboard.putString("IntakeMode", "PID");
    } else {
      m_intake.liftIntakeSpd(speed.getAsDouble());
      SmartDashboard.putString("IntakeMode", "Controller");
    }
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
