// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  CANSparkMax WLlift,WRlift;
  /** Creates a new Climber. */

  public Climber() {
    WLlift = new CANSparkMax(33,MotorType.kBrushless);
    WRlift = new CANSparkMax(36,MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void Up(double UpSpeed){
    WLlift.set(UpSpeed* -1);
    WRlift.set(UpSpeed);
  }
}
