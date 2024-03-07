package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants.LauncherConstants;

public class Launcher extends SubsystemBase {
    TalonFX shootRight, shootLeft;
    DigitalInput sensor;
    

    public Launcher() {
        // placeholder values!
        shootLeft = new TalonFX(LauncherConstants.shootLeftMotor);
        shootRight = new TalonFX(LauncherConstants.shootRightMotor);

        sensor = new DigitalInput(LauncherConstants.sensor);

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
        return !sensor.get();
    }
}
