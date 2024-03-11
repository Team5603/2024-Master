package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.constants.GeneralConstants.MotorConstants;

public class Intake extends SubsystemBase {
    CANSparkMax intake;

    public Intake () {
        intake = new CANSparkMax(IntakeConstants.intakeMotor, MotorType.kBrushless);

        intake.restoreFactoryDefaults();

        intake.setInverted(false);
    }
    @Override
    public void periodic() {
    }

    public void runIntake(double speed, boolean reverse) {
        if (reverse) {
            intake.set(-speed);
        } else {
            intake.set(speed);
        }
    }


//     public boolean getBeamBroken() {
//     if (beamBreakSensor.get()) {
//       return true;
//     } else {
//       return false;
//     }
//   }
}