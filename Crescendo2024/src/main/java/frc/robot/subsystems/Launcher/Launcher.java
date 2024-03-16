package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants.LauncherConstants;
import frc.robot.constants.GeneralConstants.MotorConstants;

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

        shootRight.setNeutralMode(NeutralModeValue.Coast);
        shootLeft.setNeutralMode(NeutralModeValue.Coast);

        shootLeft.getConfigurator().apply(MotorConstants.nonDriveCurrentLimitCTRE);
        shootRight.getConfigurator().apply(MotorConstants.nonDriveCurrentLimitCTRE);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Sensor", getSensor());
    }

    public void runMotors(double speed) {
        shootLeft.set(speed);
        shootRight.set(speed);
    }

    public double getMotorSpeeds() {
        return (shootLeft.get() + shootRight.get());
    }

    public boolean getSensor() {
        return !sensor.get();
    }
}
