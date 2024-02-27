package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants;

public class Intake extends SubsystemBase {
    CANSparkMax intake, intakeLift;
    PIDController maintain;
    public Intake () {
        // placeholder values !!!!!!]
        intake = new CANSparkMax(38, MotorType.kBrushless);
        intakeLift = new CANSparkMax(40, MotorType.kBrushless);

        intakeLift.setInverted(true);

        maintain = new PIDController(50, 0, 0);

    }
    @Override
    public void periodic() {
    }

    public void runIntake(double speed) {
        intake.set(-1 * speed);
    }
    
    public void liftPIDIntake() {
        double currentEnc = getLiftEncoder();
        SmartDashboard.putNumber("IntakePIDSpeed", maintain.calculate(getLiftEncoder(), currentEnc));
        intakeLift.set(maintain.calculate(getLiftEncoder(), currentEnc));
    }

    public void liftIntakeSpd(double speed) {
        intakeLift.set(speed);
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