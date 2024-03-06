// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeLift;

public class liftIntake extends Command {
  IntakeLift m_intakeLift;
  double speed;
  Boolean stopPoint;

  /** Creates a new liftIntake. */
  public liftIntake(IntakeLift intakeLift, double inputSpeed) {
    m_intakeLift = intakeLift;
    speed = inputSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopPoint = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("StopPoint", stopPoint);
    if (speed == 0) {
      if (!stopPoint) {
        SmartDashboard.putNumber("Lift Encoder", m_intakeLift.getLiftEncoder());
      }
      stopPoint = true;      
      SmartDashboard.putString("IntakeMode", "PID");
    } else {
      stopPoint = false;
      m_intakeLift.liftIntakeSpd(speed * IntakeConstants.intakeSpeedMultiplier);
      SmartDashboard.putString("IntakeMode", "Controller");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeLift.liftIntakeSpd(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
