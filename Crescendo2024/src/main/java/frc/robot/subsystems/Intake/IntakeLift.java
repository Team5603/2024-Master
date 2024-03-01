// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.GeneralConstants.IntakeConstants;

public class IntakeLift extends SubsystemBase {
  CANSparkMax intakeLift;
  SparkPIDController maintain;

  /** Creates a new IntakeLift. */
  public IntakeLift() {
    intakeLift = new CANSparkMax(40, MotorType.kBrushless);

    intakeLift.setInverted(true);

    intakeLift.getEncoder().setPosition(0);

    maintain = intakeLift.getPIDController();
    maintain.setP(.1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Encoder Lift", getLiftEncoder());
  }

  public void liftPIDIntake(boolean stopPoint) {
    if (!stopPoint) {
      SmartDashboard.putNumber("Lift Encoder", getLiftEncoder());
      maintain.setReference(getLiftEncoder(), CANSparkMax.ControlType.kPosition);
    }
  }

  public void liftIntakeSpd(double speed) {
      intakeLift.set(speed);
  }

  public void liftIntakeEnc(double encoder) {
    maintain.setReference(encoder, CANSparkMax.ControlType.kPosition);
  }

  public void lowerIntakeEnc(double speed) {
    if (getLiftEncoder() < GeneralConstants.IntakeConstants.liftDownSetpoint) {
      intakeLift.set(speed);
    }
  }

  public double getLiftEncoder() {
    return intakeLift.getEncoder().getPosition();
  }
}
