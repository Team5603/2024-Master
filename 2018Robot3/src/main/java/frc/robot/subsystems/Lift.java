// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  private TalonSRX Lift;
  private static final double Raise_Multiplier = 0.7;// .95
  private static final double Lower_Multiplier = 0.45;// .6
  private DigitalInput ArmIsUp;
  private DigitalInput ArmIsDown;
  // private static final double Maintain_Power = 0.15;
  double encPOSLIFT = 0;

  /** Creates a new shooter. */
  public Lift() {
    ArmIsUp = new DigitalInput(4);
    ArmIsDown = new DigitalInput(0);
    Lift = new TalonSRX(4);
    Lift.configFactoryDefault();
    Lift.setInverted(true);
    Lift.setNeutralMode(NeutralMode.Brake);
    // Lift.configSelectedFeedbackSensor(, 0, 30);
    Lift.setSelectedSensorPosition(0, 0, 0);
    Lift.setSensorPhase(false);

    Lift.configNominalOutputForward(0, 30);
    Lift.configNominalOutputReverse(0, 30);
    Lift.configPeakOutputForward(1, 30);
    Lift.configPeakOutputReverse(-1, 30);
    Lift.configAllowableClosedloopError(0, 0, 30);

    Lift.config_kP(0, 5);// 5
    Lift.config_kI(0, 0);// 0
    Lift.config_kD(0, 1);// 1
    Lift.config_kF(0, 0);// 0

  }

  public void Lifter(double LiftPower) {
    double liftPowerFinal;
    // Boolean controlmode;//true=percent false=position

    if (LiftPower <= 0.1 && LiftPower >= -0.1) {
    //  encPOSLIFT = GetEncoder();
      Lift.set(ControlMode.Position, encPOSLIFT);
      // Lift.set(ControlMode.PercentOutput, 0);
    } else {
      if (LiftPower > 0.1)
        liftPowerFinal = LiftPower * Raise_Multiplier;
      else
        liftPowerFinal = LiftPower * Lower_Multiplier;

      encPOSLIFT = GetEncoder();
      if (ArmIsUp.get() == false && LiftPower > 0.05) {
        liftPowerFinal = 0;

        // controlmode = false;
      }
      if (encPOSLIFT <= 50 && LiftPower < -0.05) {
        liftPowerFinal = 0;

        // controlmode = false;
      }
      // if (LiftPower != 0) {
      // controlmode = true;
      // }
      // if (controlmode = true) {

      if (liftPowerFinal == 0) {
        
        // Lift.set(ControlMode.Position, encPOSLIFT);
        // encPOSLIFT = GetEncoder();
        SmartDashboard.putString("LIFTMODE", "HOLD:" + "True");
      } else {
        Lift.set(ControlMode.PercentOutput, liftPowerFinal);
        SmartDashboard.putString("LIFTMODE", "PERCENT:" + liftPowerFinal);
      }
      // }
      SmartDashboard.putNumber("liftPowerFinal", liftPowerFinal);

    }

    SmartDashboard.putNumber("Lift Motor Power", Lift.getMotorOutputPercent());
    SmartDashboard.putNumber("encPOSLIFT", encPOSLIFT);
    SmartDashboard.putNumber("SendLiftPower", LiftPower);
    SmartDashboard.putNumber("Encoder Position", GetEncoder());
    SmartDashboard.putBoolean("ArmIsUp", ArmIsUp.get());
    SmartDashboard.putBoolean("ArmIsDown", ArmIsDown.get());

  }

  public double GetEncoder() {
    return Lift.getSelectedSensorPosition(0);
  }

  public void resetPID() {
    encPOSLIFT = GetEncoder();

  }

  public void Stop() {
    Lift.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /*
   * public void shoot(double m_LSpinSpeed, double m_wristtilt) {
   * Lift.set(TalonSRXControlMode.PercentOutput, m_LSpinSpeed);
   * Wrist.set(TalonSRXControlMode.PercentOutput, m_wristtilt * .3);
   * }
   */
}