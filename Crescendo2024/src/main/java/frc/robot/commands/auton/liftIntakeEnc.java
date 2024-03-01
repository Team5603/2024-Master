// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeLift;

public class liftIntakeEnc extends Command {
  IntakeLift m_intakeLift;
  double encoderValue;

  /** Creates a new liftIntakeEnc. */
  public liftIntakeEnc(IntakeLift sentIntakeLift, double sentEncoder) {
    m_intakeLift = sentIntakeLift;
    encoderValue = sentEncoder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Intake Enc Running", true);
    m_intakeLift.liftIntakeEnc(encoderValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Intake Enc Running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_intakeLift.getLiftEncoder() >= encoderValue) {
      return true;
    } else {
      return false;
    }
  }
}
