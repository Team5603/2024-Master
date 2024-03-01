// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants.LauncherConstants;

public class LauncherLift extends SubsystemBase {

  private static final SparkMaxAlternateEncoder.Type alternateEncoderType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static int cPR = 8192;
  CANSparkMax liftRight, liftLeft;

  /** Creates a new LauncherLift. */
  public LauncherLift() {

    liftLeft = new CANSparkMax(12, MotorType.kBrushless);
    liftRight = new CANSparkMax(9, MotorType.kBrushless);

    liftLeft.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ThroughBore",getThroughBoreEncoder());
  }

  public void runLift(double speed) {
    if (getThroughBoreEncoder() > LauncherConstants.liftDownLimit || getThroughBoreEncoder() < LauncherConstants.liftUpLimit) {
      liftLeft.set(speed);
    }
  }

  public double getThroughBoreEncoder() {
    return liftLeft.getAlternateEncoder(alternateEncoderType, cPR).getPosition();
  }
}