// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.logToSmartDashboard;
import frc.robot.commands.runNeo550;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants.TagData;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;


public class RobotContainer {
  // Sweve drive platform
  public static final Swerve m_swerve = SwerveConstants.DriveTrain;
  public static final Intake m_intake = new Intake();
  public static final Launcher m_launcher = new Launcher();
  public static final Vision m_vision = new Vision();

  private static SwerveRequest.FieldCentric swerve_drive;
  private static SwerveRequest.SwerveDriveBrake swerve_brake;
  private static SwerveRequest.RobotCentric swerve_forwardStraight;
  private static SwerveRequest.PointWheelsAt swerve_point;

  private double maxSpeed = 6; // 6 meters/sec desired top speed
  private double maxAngularRate = 1.5 * Math.PI; // 3/4 a rotation/sec max angular velocity. 1.5*pi as default

  // set up bindings for control of swerve drive platform
  public final CommandXboxController joystick = new CommandXboxController(0);

  private static Command command_joyDrive;
  private static Command command_joyPointDrive;
  private static Command command_runAuto;

  public void configureSwerve() {
    //m_swerve.getAutoPath("Tests");
    swerve_forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    swerve_brake = new SwerveRequest.SwerveDriveBrake();
    swerve_point = new SwerveRequest.PointWheelsAt();

    //command_runAuto = m_swerve.getAutoPath("Tests");
    command_joyDrive = m_swerve.applyRequest(() -> swerve_drive
        .withVelocityX(-joystick.getLeftY() * maxSpeed).withVelocityY(-joystick.getLeftX() * maxAngularRate).withRotationalRate(-joystick.getRightX() * maxAngularRate))
        .ignoringDisable(true);

    command_joyPointDrive = m_swerve.applyRequest(
        () -> swerve_point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX())));

    swerve_drive = new SwerveRequest.FieldCentric().withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.1)// add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  }

  // AprilTags stuff
  public final TagData tagData = new TagData();

  // Path follower
  // private Command command_runAuto = m_sw

  private final RobotTelemetry logger = new RobotTelemetry(maxSpeed);

  private void configureBindings() {
    configureSwerve();
    m_swerve.setDefaultCommand(command_joyDrive);
    joystick.a().whileTrue(m_swerve.applyRequest(() -> swerve_brake));
    //joystick.b().whileTrue(command_joyPointDrive);
    //joystick.x().onTrue(new PathPlannerAuto("Follow Path"));
    joystick.leftBumper().onTrue(m_swerve.runOnce(() -> m_swerve.seedFieldRelative()));
    // if (Utils.isSimulation()) {
    // m_swerve.seedFieldRelative(new Pose2d(new Translation2d(),
    // Rotation2d.fromDegrees(90)));
    // }
    m_swerve.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(m_swerve.applyRequest(() -> swerve_forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180)
        .whileTrue(m_swerve.applyRequest(() -> swerve_forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    if (joystick.getLeftX() > 0.1 || joystick.getLeftY() < -0.1) {
    }
  }

  public RobotContainer() {
    NamedCommands.registerCommand("logToSmartDashboard", new logToSmartDashboard());
    NamedCommands.registerCommand("runNeo550", new runNeo550());

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
      tagData.setAlliance(alliance.get());

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Simple Auto");
  }

  // public Command scheduleAprilTagPath() {

  // }
}
