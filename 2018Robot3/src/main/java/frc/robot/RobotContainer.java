// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Move;
import frc.robot.commands.UpandDown;
import frc.robot.commands.moveWrist;
import frc.robot.commands.spinin;
import frc.robot.subsystems.ClawSparks;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

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
  private final DriveTrain m_DriveTrain = new DriveTrain();
  private final Lift m_Lift = new Lift();
  private final Wrist m_Wrist = new Wrist();
  private final ClawSparks m_ClawSparks = new ClawSparks();
  public static Joystick playerController;
  public static Joystick LJoy, RJoy;  
  private CameraServer Eyes;
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    LJoy = new Joystick(0);
    RJoy = new Joystick(1);
    playerController = new Joystick(2);
    m_DriveTrain.setDefaultCommand(new Move(m_DriveTrain, () -> LJoy.getRawAxis(1), () -> RJoy.getRawAxis(1)));
    m_Lift.setDefaultCommand(new UpandDown(m_Lift, () -> (Math.abs(playerController.getRawAxis(1))<0.1) ? 0 : playerController.getRawAxis(1)*-1));
    m_ClawSparks.setDefaultCommand(new spinin(m_ClawSparks, ()-> playerController.getRawAxis(2), ()-> playerController.getRawAxis(3)));
    m_Wrist.setDefaultCommand(new moveWrist(m_Wrist, () -> playerController.getRawAxis(5))) ;
    Eyes.startAutomaticCapture();
    configureBindings();
  } 

  private void configureBindings() {
  }
}
