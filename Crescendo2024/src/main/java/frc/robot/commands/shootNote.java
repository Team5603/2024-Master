package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Launcher;

public class shootNote extends Command {
    private static Launcher m_Launcher = new Launcher();

    public shootNote (Launcher launcher) {
        m_Launcher = launcher;
        addRequirements(m_Launcher);
    }

    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
        m_Launcher.shootNote(1);
    }
    @Override
    public void end(boolean interrupted) {
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
