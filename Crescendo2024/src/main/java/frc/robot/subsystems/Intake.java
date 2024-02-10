package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax intake, intakeLift;
    public Intake () {
        // placeholder values !!!!!!
        intake = new CANSparkMax(4, MotorType.kBrushless);
        intakeLift = new CANSparkMax(5, MotorType.kBrushless);
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
}