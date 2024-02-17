// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonSRX LSpinner, RSpinner;

  /** Creates a new shooter. */
  public Shooter() {
    LSpinner = new TalonSRX(31);
    RSpinner = new TalonSRX(11);
    // LSpinner.
   // RSpinner.follow(LSpinner);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double m_LSpinSpeed) {
    LSpinner.set(TalonSRXControlMode.PercentOutput, m_LSpinSpeed * -1);
    RSpinner.set(TalonSRXControlMode.PercentOutput, m_LSpinSpeed * -1);
  }

}