// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeLift;

public class liftIntakeDownOrUp extends Command {
  IntakeLift m_intakeLift;
  double startEncoder;
  boolean goingUp;
  /** Creates a new liftIntakeDownOrUp. */
  public liftIntakeDownOrUp(IntakeLift sentIntakeLift) {
    m_intakeLift = sentIntakeLift;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startEncoder = m_intakeLift.getLiftThroughBoreEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (startEncoder > .75 || startEncoder < .25) {
      m_intakeLift.liftIntakeSpd(IntakeConstants.intakeLiftSpeedMultiplier);
      goingUp = false;
    } else if (startEncoder >= .25 || startEncoder <= 0.75) {
      m_intakeLift.liftIntakeSpd(-1 * IntakeConstants.intakeLiftSpeedMultiplier);
      goingUp = true;
    }
    SmartDashboard.putBoolean("goingUp", goingUp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeLift.liftIntakeSpd(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (goingUp) {
      if (m_intakeLift.getLiftThroughBoreEncoder() >= IntakeConstants.liftUpLimitLow || m_intakeLift.getLiftThroughBoreEncoder() <= 0.01) {
        return true;
      } else {
        return false;
      }
    } else {
      if (m_intakeLift.getLiftThroughBoreEncoder() >= IntakeConstants.liftDownLimitLow) {
        return true;
      } else {
        return false;
      }
    }
  }
}
