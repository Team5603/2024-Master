// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private TalonSRX LF;
  private VictorSPX LB;
  private VictorSPX RF;
  private TalonSRX RB;

  public DriveTrain() {
    LF = new TalonSRX(3);
    LB = new VictorSPX(6);
    RF = new VictorSPX(5);
    RB = new TalonSRX(2);

    LB.follow(LF);
    RB.follow(RF);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Drive(double LSpeed, double RSpeed) {
    // TODO Auto-generated method stub
    LF.set(TalonSRXControlMode.PercentOutput, LSpeed*-1);
    RF.set(VictorSPXControlMode.PercentOutput, RSpeed);

  }
}
