package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax intake;

    public Intake () {
        // placeholder values !!!!!!]
        intake = new CANSparkMax(38, MotorType.kBrushless);
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