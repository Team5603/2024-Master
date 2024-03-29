// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.auton.alignWithSpeaker;
import frc.robot.commands.auton.finals.lowerIntakeRunIntake;
import frc.robot.commands.auton.liftIntakeEnc;
import frc.robot.commands.auton.runIntakeTimed;
import frc.robot.commands.auton.shootNoteTimed;
import frc.robot.commands.launcherIntake;
import frc.robot.commands.liftIntake;
import frc.robot.commands.liftIntakeDownOrUp;
import frc.robot.commands.liftLauncher;
import frc.robot.commands.liftLauncherEnc;
import frc.robot.commands.logToSmartDashboard;
import frc.robot.commands.maintainIntakeLift;
import frc.robot.commands.releaseNoteShootNote;
import frc.robot.commands.releaseNoteShootNoteAuton;
import frc.robot.commands.runBothArms;
import frc.robot.commands.runIntake;
import frc.robot.commands.runLeftArm;
import frc.robot.commands.runRightArm;
import frc.robot.commands.shootNote;
import frc.robot.commands.shooterIntakeJoystick;
import frc.robot.commands.stopIntake;
import frc.robot.commands.stopLauncher;
import frc.robot.commands.waitSeconds;
import frc.robot.constants.GeneralConstants.ArmConstants;
import frc.robot.constants.GeneralConstants.IntakeConstants;
import frc.robot.constants.GeneralConstants.LauncherConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants.TagData;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeLift;
import frc.robot.subsystems.Launcher.Launcher;
import frc.robot.subsystems.Launcher.LauncherLift;
import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Vision;
import java.util.Optional;

public class RobotContainer {

  // Sweve drive platform
  public static final Swerve m_swerve = SwerveConstants.DriveTrain;
  public static final Intake m_intake = new Intake();
  public static final IntakeLift m_intakeLift = new IntakeLift();
  public static final Launcher m_launcher = new Launcher();
  public static final LauncherLift m_launcherLift = new LauncherLift();
  // public static final Vision m_vision = new Vision();
  public static final Arm m_arm = new Arm();

  private static SwerveRequest.FieldCentric swerve_drive;
  private static SwerveRequest.SwerveDriveBrake swerve_brake;
  private static SwerveRequest.RobotCentric swerve_forwardStraight;
  private static SwerveRequest.PointWheelsAt swerve_point;

  private double maxSpeed = 6; // 6 meters/sec desired top speed
  private double maxAngularRate = 1.5 * Math.PI; // 3/4 a rotation/sec max angular velocity. 1.5*pi as default

  // set up bindings for control of swerve drive platform
  public final CommandXboxController driveController = new CommandXboxController(0);
  public final CommandXboxController manipulateController = new CommandXboxController(1);

  private static Command command_joyDrive;
  private static Command command_joyPointDrive;
  private static Command command_runAuto;

  private static String kFailSafe = "Fail Safe";
  private static String kauto1 = "2 Note Amp Side";
  private static String kauto7 = "2 Note Amp Side - Angled";
  private static String kauto2 = "2 Note Middle";
  private static String kauto3 = "2 Note Blank Side";
  private static String kauto8 = "2 Note Blank Side - Angled";
  private static String kauto12 = "2 Note Blank Side - Angled FAR";
  private static String kauto4 = "3 Note Middle";
  private static String kauto15 = "4 Note Middle";
  private static String kauto9 = "4 Note Middle FAR";
  private static String kauto11 = "4 Note Middle HOLD";
  private static String kauto5 = "1 Note Blank Side";
  private static String kauto6 = "Get Out Of The Way";
  private static String kauto0 = "Delay Right";
  private static String kauto10 = "Delay Left";
  private static String kauto14 = "Just Shoot Amp Side";

  // private static SendableChooser<String> m_Chooser;

  private static SendableChooser<Command> m_AutonChooser;
  
  private double sentDelay;

  public RobotContainer() {
    // NamedCommands.registerCommand("logToSmartDashboard", new
    // logToSmartDashboard());
    // NamedCommands.registerCommand("lowerIntakeRunIntake", new
    // lowerIntakeRunIntake(m_intakeLift, m_intake));
    // NamedCommands.registerCommand("reverseIntake", new runIntake(m_intake,
    // IntakeConstants.intakeSpeed, true));
    // NamedCommands.registerCommand("runLauncher", new shootNote(m_launcher,
    // LauncherConstants.launcherSpeedLauncher));
    // NamedCommands.registerCommand("raiseIntake", new liftIntakeEnc(m_intakeLift,
    // 0));
    NamedCommands.registerCommand("shootNoteHold", new releaseNoteShootNoteAuton(m_intake, m_launcher, 0.4, true, 1.2));
    NamedCommands.registerCommand("shootNoteStop", new releaseNoteShootNoteAuton(m_intake, m_launcher, 0, false, .1));
    NamedCommands.registerCommand("moveIntake", new liftIntakeDownOrUp(m_intakeLift, 0.6));
    NamedCommands.registerCommand("runIntake", new runIntake(m_intake, IntakeConstants.intakeSpeed, false));
    NamedCommands.registerCommand("revLauncher", new shootNoteTimed( m_launcher, LauncherConstants.launcherSpeedLauncher, 15));
    NamedCommands.registerCommand("releaseNote", new runIntakeTimed(m_intake, .5, true));
    NamedCommands.registerCommand("stopLauncher", new stopLauncher(m_launcher));
    NamedCommands.registerCommand("stopIntake", new stopIntake(m_intake));
    NamedCommands.registerCommand("waitSeconds", new waitSeconds(sentDelay));

    // m_Chooser = new SendableChooser<>();

    // m_Chooser.setDefaultOption(kFailSafe, kFailSafe);
    // m_Chooser.addOption(kauto1, kauto1);
    // m_Chooser.addOption(kauto2, kauto2);
    // m_Chooser.addOption(kauto7, kauto7);
    // m_Chooser.addOption(kauto3, kauto3);
    // m_Chooser.addOption(kauto8, kauto8);
    // m_Chooser.addOption(kauto12, kauto12);
    // m_Chooser.addOption(kauto4, kauto4);
    // m_Chooser.addOption(kauto15, kauto15);
    // m_Chooser.addOption(kauto9, kauto9);
    // m_Chooser.addOption(kauto11, kauto11);
    // m_Chooser.addOption(kauto5, kauto5);
    // m_Chooser.addOption(kauto6, kauto6);
    // m_Chooser.addOption(kauto0, kauto0);
    // m_Chooser.addOption(kauto10, kauto10);
    // m_Chooser.addOption(kauto14, kauto14);

    // SmartDashboard.putData(m_Chooser);

    m_AutonChooser = AutoBuilder.buildAutoChooser();
    

    // m_AutonChooser.addOption(kFailSafe, new PathPlannerAuto("Leave Left"));
    // m_AutonChooser.addOption(kauto1, new PathPlannerAuto("Left 2 Note"));
    // m_AutonChooser.addOption(kauto7, new PathPlannerAuto("Left 2 Note Angled"));
    // m_AutonChooser.addOption(kauto2, new PathPlannerAuto("Middle 2 Note"));
    // m_AutonChooser.addOption(kauto3, new PathPlannerAuto("Right 2 Note"));
    // m_AutonChooser.addOption(kauto8, new PathPlannerAuto("Right 2 Note Angled"));
    // m_AutonChooser.addOption(kauto12, new PathPlannerAuto("Right 2 Note Angled FAR"));
    // m_AutonChooser.addOption(kauto4, new PathPlannerAuto("Middle 3 Note"));
    // m_AutonChooser.addOption(kauto15, new PathPlannerAuto("Middle 4 Note"));
    // m_AutonChooser.addOption(kauto9, new PathPlannerAuto("Middle 4 Note Far Right"));
    // m_AutonChooser.addOption(kauto11, new PathPlannerAuto("Middle 4 Note Far Right Hold"));
    // m_AutonChooser.addOption(kauto5, new PathPlannerAuto("Right 1 Note"));
    // m_AutonChooser.addOption(kauto6, new PathPlannerAuto("Leave Right"));
    // m_AutonChooser.addOption(kauto0, new PathPlannerAuto("Delay Right"));
    // m_AutonChooser.addOption(kauto10, new PathPlannerAuto("Delay Left"));
    // m_AutonChooser.addOption(kauto14, new PathPlannerAuto("Just Shoot Left"));
  

    SmartDashboard.putData(m_AutonChooser);

    tagData.setAlliance(getAlliance());
    if (getAlliance() == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
      m_swerve.setOperatorPerspectiveForward(new Rotation2d(0));
    } else {
      m_swerve.setOperatorPerspectiveForward(new Rotation2d(180));
    }

    configureBindings();
  }

  // Gets the current match's alliance (OR, defaults to Alliance.RED)
  public static Alliance getAlliance() {
    Optional<Alliance> _alliance = DriverStation.getAlliance();
    if (_alliance.isPresent())
      return _alliance.get();
    else
      return Alliance.Red;
  }

  public void configureSwerve() {
    // m_swerve.getAutoPath("Tests");
    swerve_forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    swerve_brake = new SwerveRequest.SwerveDriveBrake();
    swerve_point = new SwerveRequest.PointWheelsAt();

    // command_runAuto = m_swerve.getAutoPath("Tests");
    command_joyDrive = m_swerve
        .applyRequest(() -> swerve_drive
            .withVelocityX(-driveController.getLeftY() * maxSpeed)
            .withVelocityY(-driveController.getLeftX() * maxAngularRate)
            .withRotationalRate(-driveController.getRightX() * maxAngularRate))
        .ignoringDisable(true);

    command_joyPointDrive = m_swerve.applyRequest(() -> swerve_point.withModuleDirection(
        new Rotation2d(
            -driveController.getLeftY(),
            -driveController.getLeftX())));

    swerve_drive = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.1) // add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  }

  public void configureDelay() {
    sentDelay = Preferences.getDouble("delay", 8);//SmartDashboard.getNumber("delay", 0);
    //sentDelay = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("delay").getDouble(0);
    SmartDashboard.putNumber("sentDelay", sentDelay);  

  }

  // AprilTags stuff
  public static final TagData tagData = new TagData();

  // Path follower
  // private Command command_runAuto = m_sw

  private final RobotTelemetry logger = new RobotTelemetry(maxSpeed);

  private void configureBindings() {
    configureSwerve();
    m_swerve.setDefaultCommand(command_joyDrive);
    // m_intakeLift.setDefaultCommand(new liftIntake(m_intakeLift, () ->
    // Math.abs(manipulateController.getRightY()) < .1 ? 0:
    // manipulateController.getRightY()));
    m_intakeLift.setDefaultCommand(new maintainIntakeLift(m_intakeLift));
    // m_launcher.setDefaultCommand(new shooterIntakeJoystick(m_intake, m_launcher,
    // () -> Math.abs(manipulateController.getRightY()) < .1 ? 0:
    // manipulateController.getRightY()));
    // m_arm.setDefaultCommand(new runBothArms(m_arm, () ->
    // Math.abs(manipulateController.getRightY()) < .1 ? 0:
    // manipulateController.getRightY()));
    m_launcherLift.setDefaultCommand(new liftLauncher(m_launcherLift, () -> Math.abs(manipulateController.getLeftY()) < .1 ? 0 : -manipulateController.getLeftY()));

    driveController.a().whileTrue(m_swerve.applyRequest(() -> swerve_brake));
    // joystick.b().whileTrue(command_joyPointDrive);
    // joystick.x().onTrue(new PathPlannerAuto("Follow Path"));
    driveController.leftBumper().onTrue(m_swerve.runOnce(() -> m_swerve.seedFieldRelative()));

    manipulateController.b().whileTrue(new runIntake(m_intake, IntakeConstants.intakeSpeed, true));
    manipulateController.a().whileTrue(new runIntake(m_intake, IntakeConstants.intakeSpeed, false));
    // manipulateController.x().whileTrue(new shootNote(m_launcher,
    // LauncherConstants.launcherSpeedAmp));
    manipulateController.x().whileTrue(new launcherIntake(m_launcher, m_intake, LauncherConstants.launcherIntakeSpeed, false));
    //manipulateController.y().onTrue(new liftLauncherEnc(m_launcherLift, LauncherConstants.sourceEncoderPostion));
    manipulateController.axisGreaterThan(2, .1).whileTrue(new launcherIntake( m_launcher, m_intake, LauncherConstants.launcherIntakeSpeedSlower, true));

    manipulateController.axisGreaterThan(5, .1).whileTrue(new shooterIntakeJoystick(m_intake, m_launcher, () -> manipulateController.getRightY()));
    manipulateController.axisLessThan(5, .1).whileTrue(new shooterIntakeJoystick(m_intake, m_launcher, () -> manipulateController.getRightY()));

    manipulateController.axisGreaterThan(3, .1).whileTrue(new shootNote(m_launcher, LauncherConstants.launcherSpeedLauncher));
    manipulateController.button(5).whileTrue(new liftIntake(m_intakeLift, 1));
    manipulateController.button(6).whileTrue(new liftIntake(m_intakeLift, -1));

    //manipulateController.povUp().onTrue(new liftLauncherEnc(m_launcherLift,LauncherConstants.ampEncoderPosition));
    // manipulateController.povDown().onTrue(new alignWithSpeaker(m_swerve, m_vision, 0.1, 20));

    // manipulateController.y().onTrue((m_intakeLift.getLiftThroughBoreEncoder() >
    // .1)?new liftIntakeEnc(m_intakeLift, 0.01): new liftIntakeEnc(m_intakeLift,
    // IntakeConstants.liftDownSetpoint));

    driveController.axisGreaterThan(3, .2).whileTrue(new runBothArms(m_arm, ArmConstants.extendSpeedMultiplier));
    driveController.axisGreaterThan(2, .2).whileTrue(new runBothArms(m_arm, -1 * ArmConstants.extendSpeedMultiplier));

    driveController.povUp().whileTrue(new runLeftArm(m_arm, ArmConstants.extendSpeedMultiplier));
    driveController.povDown().whileTrue(new runLeftArm(m_arm, -1 * ArmConstants.extendSpeedMultiplier));

    driveController.povRight().whileTrue(new runRightArm(m_arm, ArmConstants.extendSpeedMultiplier)); 
    driveController.povLeft().whileTrue(new runRightArm(m_arm, -1 * ArmConstants.extendSpeedMultiplier));

    // if (Utils.isSimulation()) {
    // m_swerve.seedFieldRelative(new Pose2d(new Translation2d(),
    // Rotation2d.fromDegrees(90)));
    // }
    m_swerve.registerTelemetry(logger::telemeterize);
    // driveController.pov(0).whileTrue(m_swerve.applyRequest(() ->
    // swerve_forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    // driveController.pov(180)
    // .whileTrue(m_swerve.applyRequest(() ->
    // swerve_forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  }

  public Command getAutonomousCommand() {
    NamedCommands.registerCommand("waitSeconds", new waitSeconds(sentDelay));
    return m_AutonChooser.getSelected();
  //   String m_autoSelected = m_Chooser.getSelected();
  //   System.out.println("Auton Selected " + m_autoSelected);

  //   configureDelay();

  //   NamedCommands.registerCommand("waitSeconds", new waitSeconds(sentDelay));

  //   switch (m_autoSelected) {
  //     case "Fail Safe":
  //       return new PathPlannerAuto("Leave Left");
  //     case "2 Note Amp Side":
  //       return new PathPlannerAuto("Left 2 Note");
  //     case "2 Note Amp Side - Angled":
  //       return new PathPlannerAuto("Left 2 Note Angled");
  //     case "2 Note Middle":
  //       return new PathPlannerAuto("Middle 2 Note");
  //     case "2 Note Blank Side":
  //       return new PathPlannerAuto("Right 2 Note");
  //     case "2 Note Blank Side - Angled":
  //       return new PathPlannerAuto("Right 2 Note Angled");
  //     case "2 Note Blank Side - Angled FAR":
  //       return new PathPlannerAuto("Right 2 Note Angled Far");
  //     case "3 Note Middle":
  //       return new PathPlannerAuto("Middle 3 Note");
  //     case "4 Note Middle":
  //       return new PathPlannerAuto("Middle 4 Note");
  //     case "4 Note Middle FAR":
  //       return new PathPlannerAuto("Middle 4 Note Far Right");
  //     case "4 Note Middle HOLD":
  //       return new PathPlannerAuto("Middle 4 Note Far Right Hold");
  //     case "1 Note Blank Side":
  //       return new PathPlannerAuto("Right 1 Note");
  //     case "Get Out Of The Way":
  //       return new PathPlannerAuto("Leave Right");
  //     case "Delay Right":
  //       return new PathPlannerAuto("Delay Right");
  //     case "Delay Left":
  //       return new PathPlannerAuto("Delay Left");
  //     case "Just Shoot Amp Side":
  //       return new PathPlannerAuto("Just Shoot Left");
  //     default:
  //       return new PathPlannerAuto("Leave Left");
  //   }
  // }
  // public Command scheduleAprilTagPath() {

  }
}
