package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax inkakeRight, intakeLeft;
    public Intake () {
        // placeholder values !!!!!!
        // inkakeRight = new CANSparkMax(4, MotorType.kBrushless);
        // intakeLeft = new CANSparkMax(5, MotorType.kBrushless);
    }
    @Override
    public void periodic() {
    }
}