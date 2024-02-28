package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants;

public class Intake extends SubsystemBase {
    CANSparkMax intake, intakeLift;
    SparkPIDController maintain;
    DigitalInput sensor;

    public Intake () {
        // placeholder values !!!!!!]
        intake = new CANSparkMax(38, MotorType.kBrushless);
        intakeLift = new CANSparkMax(40, MotorType.kBrushless);

        intakeLift.setInverted(true);
        
        intakeLift.getEncoder().setPosition(0);

        maintain = intakeLift.getPIDController();
        maintain.setP(.1);

        sensor = new DigitalInput(1);
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Sensor", sensor.get());
        SmartDashboard.putNumber("Current Encoder Lift", getLiftEncoder());
    }

    public void runIntake(double speed, boolean reverse) {
        if (reverse) {
            intake.set(-speed);
        } else {
            intake.set(speed);
        }
    }
    
    public void liftPIDIntake(boolean stopPoint) {
        if (!stopPoint) {
            SmartDashboard.putNumber("Lift Encoder", getLiftEncoder());
            maintain.setReference(getLiftEncoder(), CANSparkMax.ControlType.kPosition);
        }
    }

    public void liftIntakeSpd(double speed) {
        //if (getLiftEncoder() >= 0 && getLiftEncoder() < /*arbitary */100000) {
            intakeLift.set(speed);
        //}
    }

    public void liftIntakeEnc(double speed) {
        if (getLiftEncoder() > GeneralConstants.IntakeConstants.liftUpLimit) {
            intakeLift.set(speed);
        }
    }

    public void lowerIntakeEnc(double speed) {
        if (getLiftEncoder() < GeneralConstants.IntakeConstants.liftDownLimit) {
            intakeLift.set(speed);
        }
    }

    public double getLiftEncoder() {
        return intakeLift.getEncoder().getPosition();
    }

//     public boolean getBeamBroken() {
//     if (beamBreakSensor.get()) {
//       return true;
//     } else {
//       return false;
//     }
//   }
}