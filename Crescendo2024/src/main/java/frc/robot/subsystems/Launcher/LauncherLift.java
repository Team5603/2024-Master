// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Launcher;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants.LauncherConstants;
import frc.robot.constants.GeneralConstants.MotorConstants;

public class LauncherLift extends SubsystemBase {

  private static final SparkAbsoluteEncoder.Type absoluteEncoderType = SparkAbsoluteEncoder.Type.kDutyCycle;
  private static int cPR = 8192;
  CANSparkMax liftRight, liftLeft;
  DigitalInput limitSwitchDown;

  /** Creates a new LauncherLift. */
  public LauncherLift() {

    liftLeft = new CANSparkMax(LauncherConstants.liftLeftMotor, MotorType.kBrushless);
    liftRight = new CANSparkMax(LauncherConstants.liftRightMotor, MotorType.kBrushless);

    limitSwitchDown = new DigitalInput(LauncherConstants.liftLimitSwitch);

    liftLeft.restoreFactoryDefaults();
    liftRight.restoreFactoryDefaults();
    
    liftLeft.setInverted(false);
    liftRight.setInverted(true);

    liftLeft.setIdleMode(IdleMode.kBrake);
    liftRight.setIdleMode(IdleMode.kBrake);

    liftLeft.setSmartCurrentLimit(MotorConstants.nonDriveCurrentLimitREV);
    liftRight.setSmartCurrentLimit(MotorConstants.nonDriveCurrentLimitREV);

    getThroughBoreEncoderRaw().setZeroOffset(LauncherConstants.absoluteEncoderZeroPoint);
    getThroughBoreEncoderRaw().setInverted(false);

  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("ThroughBore", getThroughBoreEncoder());

    // SmartDashboard.putBoolean("Lift Limit", getLimitSwitch());

    // SmartDashboard.putNumber("Left Motor Speed", liftLeft.get());
    // SmartDashboard.putNumber("Right Motor Speed", liftRight.get());
  }

  public void runLift(double speed) {
    double finalSpeed = speed;
    
    if (speed < 0) {
      if (getThroughBoreEncoder() >= LauncherConstants.liftDownLimitLow && getThroughBoreEncoder() < LauncherConstants.liftDownLimitHigh || getLimitSwitch()) {
        finalSpeed = 0;
      }
    } else if (speed > 0) {
      if (getThroughBoreEncoder() >= LauncherConstants.liftUpLimitLow && getThroughBoreEncoder() < LauncherConstants.liftUpLimitHigh) {
        finalSpeed = 0;
      }
    }
    liftLeft.set(finalSpeed);
    liftRight.set(finalSpeed);
  }

  public double getThroughBoreEncoder() {
    return liftRight.getAbsoluteEncoder(absoluteEncoderType).getPosition();
  }

  private SparkAbsoluteEncoder getThroughBoreEncoderRaw() {
    return liftRight.getAbsoluteEncoder(absoluteEncoderType);
  }

  public boolean getLimitSwitch() {
    return !limitSwitchDown.get();
  }
  
}
