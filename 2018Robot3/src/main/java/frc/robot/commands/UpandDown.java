// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lift;

public class UpandDown extends Command {
  Lift m_Lift;
  DoubleSupplier m_UpDownSpeed;
 // DoubleSupplier m_wristtilt;

  /** Creates a new Shootyshoot. */
  public UpandDown(Lift sentLift, DoubleSupplier sentUpDownSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Lift = sentLift;
    addRequirements(m_Lift);
    m_UpDownSpeed = sentUpDownSpeed;
  //  m_wristtilt = sentwristtilt;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Lift.Lifter(m_UpDownSpeed.getAsDouble());


  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Lift.Lifter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

 
}
