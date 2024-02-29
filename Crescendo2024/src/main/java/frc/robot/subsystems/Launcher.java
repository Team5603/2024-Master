package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    TalonFX shootRight, shootLeft;
    CANSparkMax liftRight, liftLeft;
    private static final SparkMaxAlternateEncoder.Type alternateEncoderType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static int cPR = 8192;
    

    public Launcher() {
        // placeholder values!
        shootLeft = new TalonFX(10);
        shootRight = new TalonFX(26);

        liftLeft = new CANSparkMax(12, MotorType.kBrushless);
        liftRight = new CANSparkMax(9, MotorType.kBrushless);

        shootLeft.setPosition(0);
        shootRight.setPosition(0);

        shootRight.setInverted(true);
        shootLeft.setInverted(false);

        liftLeft.setInverted(true);

    }

    @Override
    public void periodic() {
    }

    public void runLift(double speed) {
        liftRight.set(speed);
        liftLeft.set(speed);
    }

    public void shootNote(double speed) {
        shootLeft.set(speed);
        shootRight.set(speed);
    }

    public double getThroughBoreEncoder() {
        return liftLeft.getAlternateEncoder(alternateEncoderType, cPR).getPosition();
    }
}
