// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.liftIntake;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.constants.GeneralConstants.LauncherConstants;
import frc.robot.constants.GeneralConstants.MotorConstants;

public class IntakeLift extends SubsystemBase {
  CANSparkMax intakeLift;
  SparkPIDController maintain;
  DigitalInput intakeLimit;
  SparkAbsoluteEncoder intakeLiftEncoder;
  SparkAbsoluteEncoder.Type encoderType = SparkAbsoluteEncoder.Type.kDutyCycle;
  double lastPosition;

  /** Creates a new IntakeLift. */
  public IntakeLift() {
    intakeLift = new CANSparkMax(IntakeConstants.liftMotor, MotorType.kBrushless);

    intakeLimit = new DigitalInput(IntakeConstants.liftLimitSwitch);

    intakeLift.restoreFactoryDefaults();

    intakeLift.setInverted(true);

    intakeLift.setSmartCurrentLimit(MotorConstants.nonDriveCurrentLimitREV);

    intakeLiftEncoder = intakeLift.getAbsoluteEncoder(encoderType);

    intakeLiftEncoder.setZeroOffset(IntakeConstants.absoluteEncoderZeroPoint);


    maintain = intakeLift.getPIDController();
    maintain.setP(.1);
    lastPosition= getLiftEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Encoder Lift", getLiftThroughBoreEncoder());
    SmartDashboard.putBoolean("Intake Limit Switch", intakeLimit.get());
    SmartDashboard.putNumber("Intake Lift Motor Speed", intakeLift.get());
  }

/*   public void liftPIDIntake(boolean stopPoint) {
    if (!stopPoint) {
      SmartDashboard.putNumber("Lift Encoder", getLiftThroughBoreEncoder());
      maintain.setReference(getLiftEncoder(), CANSparkMax.ControlType.kPosition);
    }
  }
*/

  public void liftIntakeSpd(double speed) {
    double finalSpeed = speed;
    SmartDashboard.putNumber("sentSpeed", speed);
    if (speed > 0) {
      if (getLiftThroughBoreEncoder() >= IntakeConstants.liftDownLimitLow && getLiftThroughBoreEncoder() < IntakeConstants.liftDownLimitHigh) {
        finalSpeed = 0;
      }
    } else if (speed < 0) {
      if (getLiftThroughBoreEncoder() >= IntakeConstants.liftUpLimitLow && getLiftThroughBoreEncoder() < IntakeConstants.liftUpLimitHigh) {
        finalSpeed = 0;
      }
    }
    SmartDashboard.putNumber("final speed", finalSpeed);
    if (finalSpeed==0)
      maintain.setReference(lastPosition, CANSparkMax.ControlType.kPosition);
    else {
      intakeLift.set(finalSpeed);
      lastPosition = getLiftEncoder();
    }
  }

  // public void liftIntakeEnc(double encoder) {
  //   maintain.setReference(encoder, CANSparkMax.ControlType.kPosition);
  // }

  // public void lowerIntakeEnc(double speed) {
  //   if (getLiftEncoder() < GeneralConstants.IntakeConstants.liftDownSetpoint) {
  //     intakeLift.set(speed);
  //   }
  // }

  public double getLiftEncoder() {
    return intakeLift.getEncoder().getPosition();
  }

  public double getLiftThroughBoreEncoder() {
    return intakeLiftEncoder.getPosition();
  }
}
