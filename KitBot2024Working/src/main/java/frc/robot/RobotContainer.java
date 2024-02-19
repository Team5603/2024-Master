// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Climb;
import frc.robot.commands.InOut;
import frc.robot.commands.Move;
import frc.robot.commands.Shootyshoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_DriveTrain;
  private final Shooter m_shooter;
  private final Climber m_Climber;
  private CommandXboxController playerController;
  private Joystick LJoy, RJoy;
  // private JoystickButton ShootButton;
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_DriveTrain = new DriveTrain();
    m_shooter = new Shooter();
    m_Climber = new Climber();

    LJoy = new Joystick(0);
    RJoy = new Joystick(1);
    playerController = new CommandXboxController(2);
    m_DriveTrain.setDefaultCommand(new Move(m_DriveTrain,
     () -> (Math.abs(LJoy.getRawAxis(1))<0.18)?0:LJoy.getRawAxis(1),
     () -> (Math.abs(RJoy.getRawAxis(1))<0.1)?0:RJoy.getRawAxis(1)));
    playerController.b().whileTrue(new Shootyshoot(m_shooter, 1, .3));
    playerController.x().whileTrue( new Shootyshoot(m_shooter, 0.3, .3));
    playerController.a().whileTrue(new InOut(m_shooter, -0.25));
    m_Climber.setDefaultCommand(new Climb(m_Climber, ()->Math.abs(playerController.getRawAxis(1))<.1?0:playerController.getRawAxis(1)));
    // m_shooter.setDefaultCommand(new Shootyshoot(m_shooter, () ->
    // playerController.getRawAxis(1)));

    configureBindings();
  }

  private void configureBindings() {
  }
}
