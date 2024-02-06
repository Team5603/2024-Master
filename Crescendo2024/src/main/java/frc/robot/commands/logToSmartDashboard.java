package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class logToSmartDashboard extends Command {
    public logToSmartDashboard () {}
    @Override
    public void execute() {
        SmartDashboard.putString("Auton Command", "YAY");
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
