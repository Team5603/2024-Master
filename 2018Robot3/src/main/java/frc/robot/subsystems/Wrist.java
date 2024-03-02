// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private TalonSRX wrist;
  double ENCPosWrist;
  private DigitalInput WristIsUp;
  private DigitalInput WristIsDown;
  private static final double Raise_Multiplier = 0.4;// .95
  private static final double Lower_Multiplier = 0.3;// .6

  /** Creates a new Wrist. */
  public Wrist() {
    wrist = new TalonSRX(1);
    wrist.setNeutralMode(NeutralMode.Brake);
    WristIsUp = new DigitalInput(1);
    WristIsDown = new DigitalInput(2);

    wrist.setSelectedSensorPosition(0, 0, 0);
    wrist.setSensorPhase(false);

    wrist.configNominalOutputForward(0, 30);
    wrist.configNominalOutputReverse(0, 30);
    wrist.configPeakOutputForward(1, 30);
    wrist.configPeakOutputReverse(-1, 30);
    wrist.configAllowableClosedloopError(0, 0, 30);

    wrist.config_kP(0, 5);// 5
    wrist.config_kI(0, 0);// 0
    wrist.config_kD(0, 1);// 1
    wrist.config_kF(0, 0);// 0

  }

  public void Wrister(double WristPower) {
    double WristPowerFinal;

    if (WristPower <= 0.1 && WristPower >= -0.1) {
      // encPOSLIFT = GetEncoder();
      wrist.set(ControlMode.Position, ENCPosWrist);
      // Lift.set(ControlMode.PercentOutput, 0);
    } else {
      if (WristPower <= -0.1)
        WristPowerFinal = WristPower * Raise_Multiplier;
      else
        WristPowerFinal = WristPower * Lower_Multiplier;

      ENCPosWrist = GetEncoder();
      if (WristIsUp.get() == false && WristPower < -0.1) {
        WristPowerFinal = 0;

      }
      if (WristIsDown.get() == false && WristPower > 0.1) {
        WristPowerFinal = 0;

      }

      if (WristPowerFinal == 0) {

        // Lift.set(ControlMode.Position, encPOSLIFT);
        // encPOSLIFT = GetEncoder();
        SmartDashboard.putString("WRISTMODE", "HOLD:" + "True");
      } else {
        wrist.set(ControlMode.PercentOutput, WristPowerFinal);
        SmartDashboard.putString("WRISTMODE", "PERCENT:" + WristPowerFinal);
      }
      // }
      SmartDashboard.putNumber("WristPower", WristPower);
      SmartDashboard.putNumber("wristPowerFinal", WristPowerFinal);

    }

    SmartDashboard.putNumber("Wrist Motor Power", wrist.getMotorOutputPercent());
    SmartDashboard.putNumber("encPOSWrist", ENCPosWrist);
    SmartDashboard.putNumber("SendWristPower", WristPower);
    SmartDashboard.putNumber("Wrist Encoder Position", GetEncoder());
    SmartDashboard.putBoolean("WristIsUp", WristIsUp.get());
    SmartDashboard.putBoolean("WristIsDown", WristIsDown.get());

  }

  public double GetEncoder() {
    return wrist.getSelectedSensorPosition(0);
  }

  public void resetPID() {
    ENCPosWrist = GetEncoder();

  }

  public void Stop() {
    wrist.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("wrist.getSelectedSensorPosition(0)", wrist.getSelectedSensorPosition(0));
  }

}