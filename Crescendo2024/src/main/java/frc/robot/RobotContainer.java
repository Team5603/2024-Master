// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.angleLauncher;
import frc.robot.commands.launcherIntake;
import frc.robot.commands.liftIntake;
import frc.robot.commands.logToSmartDashboard;
import frc.robot.commands.runIntake;
import frc.robot.commands.shootNote;
import frc.robot.commands.auton.liftIntakeEnc;
import frc.robot.commands.auton.releaseNoteShootNote;
import frc.robot.commands.auton.finals.lowerIntakeRunIntake;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.constants.GeneralConstants.LauncherConstants;
import frc.robot.constants.VisionConstants.TagData;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeLift;
import frc.robot.subsystems.Launcher.Launcher;
import frc.robot.subsystems.Launcher.LauncherLift;


public class RobotContainer {
  // Sweve drive platform
  public static final Swerve m_swerve = SwerveConstants.DriveTrain;
  public static final Intake m_intake = new Intake();
  public static final IntakeLift m_intakeLift = new IntakeLift();
  public static final Launcher m_launcher = new Launcher();
  public static final LauncherLift m_launcherLift = new LauncherLift();
  public static final Vision m_vision = new Vision();

  private static SwerveRequest.FieldCentric swerve_drive;
  private static SwerveRequest.SwerveDriveBrake swerve_brake;
  private static SwerveRequest.RobotCentric swerve_forwardStraight;
  private static SwerveRequest.PointWheelsAt swerve_point;

  private double maxSpeed = 3; // 6 meters/sec desired top speed
  private double maxAngularRate = 1.5 * Math.PI; // 3/4 a rotation/sec max angular velocity. 1.5*pi as default

  // set up bindings for control of swerve drive platform
  public final CommandXboxController driveController = new CommandXboxController(0);
  public final CommandXboxController manipulateController = new CommandXboxController(1);

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
        .withVelocityX(-driveController.getLeftY() * maxSpeed).withVelocityY(-driveController.getLeftX() * maxAngularRate).withRotationalRate(-driveController.getRightX() * maxAngularRate))
        .ignoringDisable(true);

    command_joyPointDrive = m_swerve.applyRequest(
        () -> swerve_point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX())));

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
    m_intakeLift.setDefaultCommand(new liftIntake(m_intakeLift, () -> Math.abs(manipulateController.getLeftY()) < .1 ? 0: manipulateController.getLeftY()));
    m_launcherLift.setDefaultCommand(new angleLauncher(m_launcherLift, () -> Math.abs(manipulateController.getRightY()) < .1 ? 0: manipulateController.getRightY()));
    driveController.a().whileTrue(m_swerve.applyRequest(() -> swerve_brake));
    //joystick.b().whileTrue(command_joyPointDrive);
    //joystick.x().onTrue(new PathPlannerAuto("Follow Path"));
    driveController.leftBumper().onTrue(m_swerve.runOnce(() -> m_swerve.seedFieldRelative()));

    manipulateController.b().whileTrue(new runIntake(m_intake, IntakeConstants.intakeSpeed, true));
    manipulateController.a().whileTrue(new runIntake(m_intake, IntakeConstants.intakeSpeed, false));
    manipulateController.button(6).whileTrue(new shootNote(m_launcher, LauncherConstants.launcherSpeed));
    //manipulateController.y().onTrue(new liftIntakeEnc(m_intakeLift, IntakeConstants.liftDownLimit));
    manipulateController.button(5).toggleOnTrue(new launcherIntake(m_launcher, LauncherConstants.launcherIntakeSpeed, true));
    manipulateController.y().toggleOnTrue(new launcherIntake(m_launcher, LauncherConstants.launcherIntakeSpeed, false));
    
    // if (Utils.isSimulation()) {
    // m_swerve.seedFieldRelative(new Pose2d(new Translation2d(),
    // Rotation2d.fromDegrees(90)));
    // }
    m_swerve.registerTelemetry(logger::telemeterize);

    driveController.pov(0).whileTrue(m_swerve.applyRequest(() -> swerve_forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driveController.pov(180)
        .whileTrue(m_swerve.applyRequest(() -> swerve_forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    if (driveController.getLeftX() > 0.1 || driveController.getLeftY() < -0.1) {
    }
  }

  public RobotContainer() {
    NamedCommands.registerCommand("logToSmartDashboard", new logToSmartDashboard());
    NamedCommands.registerCommand("lowerIntakeRunIntake", new lowerIntakeRunIntake(m_intakeLift, m_intake));
    NamedCommands.registerCommand("reverseIntake", new runIntake(m_intake, IntakeConstants.intakeSpeed, true));
    NamedCommands.registerCommand("runLauncher", new shootNote(m_launcher, LauncherConstants.launcherSpeed));
    NamedCommands.registerCommand("raiseIntake", new liftIntakeEnc(m_intakeLift, IntakeConstants.liftUpLimit));
    NamedCommands.registerCommand("shootNote", new releaseNoteShootNote(m_intake, m_launcher));

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
      tagData.setAlliance(alliance.get());

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Shoot Test");
  }

  // public Command scheduleAprilTagPath() {

  // }
}
