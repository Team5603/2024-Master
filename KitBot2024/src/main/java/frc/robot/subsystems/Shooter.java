// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonSRX BSpinner, TSpinner;

  /** Creates a new shooter. */
  public Shooter() {
    BSpinner = new TalonSRX(31); // bottom
    TSpinner = new TalonSRX(11); // top

    BSpinner.setNeutralMode(NeutralMode.Coast);
    TSpinner.setNeutralMode(NeutralMode.Coast);

    // LSpinner.
    // RSpinner.follow(LSpinner);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void shoot(double lowerSpin, double upperSpin) {
    BSpinner.set(TalonSRXControlMode.PercentOutput, lowerSpin);
    TSpinner.set(TalonSRXControlMode.PercentOutput, upperSpin);

  }

}