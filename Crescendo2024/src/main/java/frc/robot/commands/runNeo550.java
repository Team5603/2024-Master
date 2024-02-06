package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class runNeo550 extends Command {
    private CANSparkMax sparkMax = new CANSparkMax(12, MotorType.kBrushless);
    private Timer timer = new Timer();
    public runNeo550 () {}

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    @Override
    public void execute() {
        SmartDashboard.putNumber("Timer", timer.get());
        sparkMax.set(0.1);
    }
    @Override
    public void end(boolean interrupted) {
        sparkMax.set(0);
    }
    @Override
    public boolean isFinished() {
        // when timer reaches 1.5, it's done!
        return timer.get() >= 1.5;
    }
}
