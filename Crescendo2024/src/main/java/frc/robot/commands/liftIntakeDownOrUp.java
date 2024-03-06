// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeLift;

public class liftIntakeDownOrUp extends Command {
  IntakeLift m_intakeLift;
  /** Creates a new liftIntakeDownOrUp. */
  public liftIntakeDownOrUp(IntakeLift sentIntakeLift) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_intakeLift.getLiftThroughBoreEncoder() > .75 || m_intakeLift.getLiftThroughBoreEncoder() < .25) {
    //   m_intakeLift.liftIntakeSpd(IntakeConstants.intakeSpeedMultiplier);
    // } else if (m_intakeLift.getLiftThroughBoreEncoder() <
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
