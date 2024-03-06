// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.ArmConstants;
import frc.robot.subsystems.Arm;

public class runLeftArm extends Command {
  Arm m_arm;
  double speed;

  /** Creates a new liftArmsPOV. */
  public runLeftArm(Arm sentArm, double sentSpeed) {
    m_arm = sentArm;
    speed = sentSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Arm Status", "Left Arm Running:" + speed);
    m_arm.runLeftArm(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.runLeftArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
