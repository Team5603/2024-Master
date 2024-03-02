// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  Climber m_Climber;
  DoubleSupplier m_UpSpeed;
  Boolean StopEnlightenment;
  /** Creates a new Climb. */
  public Climb(Climber sentClimber, DoubleSupplier sentUpSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Climber = sentClimber;
    addRequirements(m_Climber);
    m_UpSpeed = sentUpSpeed;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    StopEnlightenment = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      SmartDashboard.putBoolean("STOPRPOINT", StopEnlightenment);
    // m_Climber.Up(m_UpSpeed.getAsDouble());
    //   SmartDashboard.putNumber("Climb", m_UpSpeed.getAsDouble());
      // m_Climber.LiftPLS(m_UpSpeed.getAsDouble(), .4);
      if (m_UpSpeed.getAsDouble() == 0){
        m_Climber.MAINTAINumYESSSSSSS(StopEnlightenment);
        if(!StopEnlightenment){
          SmartDashboard.putNumber("LENCODER", m_Climber.GetEncoderL());
          SmartDashboard.putNumber("RENCODER", m_Climber.GetEncoderR());
        
        }
        StopEnlightenment = true;
        SmartDashboard.putString("CLIMB MODE", "MAINTAINING");
      }else{

        StopEnlightenment = false;
        m_Climber.Up(m_UpSpeed.getAsDouble());
        SmartDashboard.putString("CLIMB MODE", "HUMAN CONTROL");
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climber.Up(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
