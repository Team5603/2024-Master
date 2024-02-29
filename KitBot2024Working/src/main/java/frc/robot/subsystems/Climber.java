// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  CANSparkMax WLlift,WRlift;
  SparkPIDController LEFTUPMaintain;
  SparkPIDController RIGHTUPMaintain;

  // private static final double RAISE_MULTIPLIER = .65; //.5 BEFORE CHANGING AT BOT BASH
	// private static final double LOWER_MULTIPLIER = .2;
  // private static final double MAINTAIN_POWER = .1;
  //private static final double LIFT_CLIMB_POWER = .5;
  // double encPOSLIFT = 0;
  
  
  // CHECK TO MAKE SURE
  // int kCPR = 8192; 
  
  // PID doubles  
  // kI, kD,kF; //kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new Climber. */

  public Climber() {
    WLlift = new CANSparkMax(33,MotorType.kBrushless);
    WRlift = new CANSparkMax(36,MotorType.kBrushless);

    //restore motors
    WLlift.restoreFactoryDefaults();
    WRlift.restoreFactoryDefaults();

    //calling CANSPARKMAX for PID and encooders
    LEFTUPMaintain = WLlift.getPIDController();
    RIGHTUPMaintain = WRlift.getPIDController();
    
    LEFTUPMaintain.setP(.15);
    RIGHTUPMaintain.setP(.15);

    WLlift.setIdleMode(IdleMode.kBrake);
    WRlift.setIdleMode(IdleMode.kBrake);

    //setting to the same values
    // WLlift.setInverted(false);
    WLlift.setInverted(true);

    // WRlift.follow(WLlift);
    //setfeedbackdevice
    // LEFTUPpidController.setFeedbackDevice(WLlift.getEncoder());
    // RIGHTUPpidController.setFeedbackDevice(WRlift.getEncoder());


    
    
    //PID Coefficients  
    // kP = 5;
    // kI = 0;
    // kD = 1;
    // kF = 0;
  
    // kIz =
    // kFF =
    // kMaxOutput = 9999999999999999999999999
    // kMinOutput =999999999999999999999999  

    //setting motors


    // LEFTUPpidController.setP(kP);
    // LEFTUPpidController.setI(kI);
    // LEFTUPpidController.setD(kD);
    // LEFTUPpidController.setFF(kF);

    // RIGHTUPpidController.setP(kP);
    // RIGHTUPpidController.setI(kI);
    // RIGHTUPpidController.setD(kD);
    // RIGHTUPpidController.setFF(kF);

    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  public void Up(double UpSpeed){
    WLlift.set(UpSpeed);
    WRlift.set(UpSpeed);
        SmartDashboard.putNumber("LENCODER", WLlift.getEncoder().getPosition());
        SmartDashboard.putNumber("RENCODER", WRlift.getEncoder().getPosition());
  
      }

  public void MAINTAINumYESSSSSSS (boolean StopEnlightenment)
{
if (!StopEnlightenment){
  SmartDashboard.putNumber("LENDCODER", WLlift.getEncoder().getPosition());
  SmartDashboard.putNumber("RENDCODER", WRlift.getEncoder().getPosition());
  LEFTUPMaintain.setReference(WLlift.getEncoder().getPosition(),CANSparkMax.ControlType.kPosition);
  RIGHTUPMaintain.setReference(WRlift.getEncoder().getPosition(),CANSparkMax.ControlType.kPosition);
}
}
  // public void LiftPLS (double UpSpeed ,double LiftPower){
  //   double liftPowerFinal;

  //   SmartDashboard.putNumber("LiftSentPower", LiftPower);
  //   //m_lifter.set(LiftPower *liftmultiplier );
  //   if (LiftPower == 0) {
  //     //liftPowerFinal = MAINTAIN_POWER;
  //     if (WLlift.getEncoder().getPosition() < /*arbitrary*/1000) {
  //       WLlift.set(LiftPower);
  //       System.out.println("WLliftsetstoLiftPower");
  //     } else {
  //       StopALL();
  //       System.out.println("STOPALL");
  //     }
  //     //System.out.println("lift Maintain-out:" + (int)(m_lifter.getMotorOutputPercent()*100)+"%\tpos:"+GetEncoder()+"\terr:"+m_lifter.getClosedLoopError(0)+"\ttrg:"+encPOSLIFT);
  //   } else {
  //     if (LiftPower<0)
  //       liftPowerFinal = LiftPower*RAISE_MULTIPLIER;
        
  //     else  
  //       liftPowerFinal = LiftPower*LOWER_MULTIPLIER;
      
  //     encPOSLIFT=GetEncoderL();
  //     WLlift.set(liftPowerFinal);

      
  //     //System.out.println("Lift Joystick:" + (int)(m_lifter.getMotorOutputPercent()*100)+"%\tpos:"+GetEncoder()+"\tLift Final:"+liftPowerFinal);

  //   }    // SmartDashboard.putNumber("Lift Motor Power",3); 
    // SmartDashboard.putNumber("liftower", LiftPower);
   


  //}
  // //LEFT SIDE FUNCTIONS
  public  double GetEncoderL(){
    return WLlift.getEncoder().getPosition();
  }

  // public void resetPIDL() {
  //   encPOSLIFT=GetEncoderL();
  // }


  // //RIGHT SIDE FUNCTIONS
    public  double GetEncoderR(){
    return WRlift.getEncoder().getPosition();
  }

  // public void resetPIDR() {
  //   encPOSLIFT=GetEncoderR();
  // }

  //STOPPPPPPP

  public void StopALL() {
      WLlift.set(0);
      WRlift.set(0);
    }
  
}
