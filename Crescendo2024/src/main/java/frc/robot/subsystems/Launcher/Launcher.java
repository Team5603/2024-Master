package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    TalonFX shootRight, shootLeft;
    DigitalInput sensor;
    

    public Launcher() {
        // placeholder values!
        shootLeft = new TalonFX(10);
        shootRight = new TalonFX(26);

        sensor = new DigitalInput(1);

        shootLeft.setPosition(0);
        shootRight.setPosition(0);

        shootRight.setInverted(true);
        shootLeft.setInverted(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Sensor", getSensor());
    }

    public void runMotors(double speed) {
        shootLeft.set(speed);
        shootRight.set(speed);
    }

    public boolean getSensor() {
        return sensor.get();
    }
}
