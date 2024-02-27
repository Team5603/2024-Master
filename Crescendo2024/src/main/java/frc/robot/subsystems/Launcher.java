package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    TalonFX shootRight, shootLeft;
    CANSparkMax liftRight, liftLeft;
    DigitalInput sensor = new DigitalInput(1);

    public Launcher() {
        // placeholder values!
        shootLeft = new TalonFX(0);
        shootRight = new TalonFX(1);

        // liftRight = new CANSparkMax(2, MotorType.kBrushless);
        // liftLeft = new CANSparkMax(3, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Sensor", sensor.get());
    }
}
