package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants;

public class Intake extends SubsystemBase {
    CANSparkMax intake, intakeLift;
    DigitalInput beamBreakSensor;
    public Intake () {
        // placeholder values !!!!!!
        intake = new CANSparkMax(/*placeholder*/4, MotorType.kBrushless);
        intakeLift = new CANSparkMax(/*placeholder*/5, MotorType.kBrushless);

        beamBreakSensor = new DigitalInput(/*placeholder*/ 6);
    }
    @Override
    public void periodic() {
    }

    public void runIntake(double speed, boolean reverse) {
        if (!reverse) {
            intake.set(speed);
        } else {
            intake.set(-1 * speed);
        }
    }

    public void liftIntake(double speed) {
        if (getLiftEncoder() > GeneralConstants.Intake.liftUpLimit) {
            intakeLift.set(speed);
        }
    }

    public void lowerIntake(double speed) {
        if (getLiftEncoder() < GeneralConstants.Intake.liftDownLimit) {
            intakeLift.set(speed);
        }
    }

    public double getLiftEncoder() {
        return intakeLift.getEncoder().getPosition();
    }

      public boolean getBeamBroken() {
    if (beamBreakSensor.get()) {
      return true;
    } else {
      return false;
    }
  }
}