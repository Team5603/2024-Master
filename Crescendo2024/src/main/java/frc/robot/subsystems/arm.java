// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants.ArmConstants;
import frc.robot.constants.GeneralConstants.MotorConstants;

public class Arm extends SubsystemBase {
  CANSparkMax leftArm, rightArm;
  DigitalInput leftArmLimitSwitch, rightArmLimitSwitch;
  /** Creates a new arm. */
  public Arm() {

    leftArm = new CANSparkMax(ArmConstants.leftArmMotor, CANSparkMax.MotorType.kBrushless);
    rightArm = new CANSparkMax(ArmConstants.rightArmMotor, CANSparkMax.MotorType.kBrushless);

    leftArmLimitSwitch = new DigitalInput(ArmConstants.leftArmLimitSwitch);
    rightArmLimitSwitch = new DigitalInput(ArmConstants.rightArmLimitSwitch);

    leftArm.restoreFactoryDefaults();
    rightArm.restoreFactoryDefaults();

    leftArm.getEncoder().setPosition(0);
    rightArm.getEncoder().setPosition(0);

    leftArm.setInverted(true);
    rightArm.setInverted(false);

    leftArm.setIdleMode(IdleMode.kBrake);
    rightArm.setIdleMode(IdleMode.kBrake);

    leftArm.setSmartCurrentLimit(MotorConstants.nonDriveCurrentLimitREV);
    rightArm.setSmartCurrentLimit(MotorConstants.nonDriveCurrentLimitREV);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Left Motor Encoder", leftArm.getEncoder().getPosition());
    // SmartDashboard.putNumber("Right Motor Encoder", rightArm.getEncoder().getPosition());
  }

  public void runArmsTogether(double speed) {
    double finalSpeed = speed;

    if (speed > 0) {
      if (getLimitSwitch("both")) {
        finalSpeed = 0;
      }
    } else if(speed < 0) {
      // if (getLeftEncoder() > ArmConstants.extendEncoderLimit) {
      //   finalSpeed = 0;
      // }
    }
    leftArm.set(finalSpeed);
    rightArm.set(finalSpeed);
  }

  public void runLeftArm(double speed) {
    double finalSpeed = speed;
    leftArm.set(finalSpeed);
  }

  public void runRightArm(double speed) {
    double finalSpeed = speed;
    rightArm.set(finalSpeed);
  }

  public boolean getLimitSwitch(String side) {
    switch (side) {
      case "left":
        return leftArmLimitSwitch.get();
      case "right":
        return rightArmLimitSwitch.get();
      case "both":
        if (rightArmLimitSwitch.get() && leftArmLimitSwitch.get()) {
          return true;
        } else {
          return false;
        }
      default:
        throw new Error("Incorrect limit switch side called; arm subsystem, getLimitSwitch function", null);
    }
  }

  public double getLeftEncoder() {
    return leftArm.getEncoder().getPosition();
  }

  public double getRightEncoder() {
    return rightArm.getEncoder().getPosition();
  }
}
