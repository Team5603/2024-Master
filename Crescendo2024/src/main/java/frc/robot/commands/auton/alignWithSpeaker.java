// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.GeneralConstants.LauncherConstants;
import frc.robot.subsystems.Launcher.Launcher;
import frc.robot.utils.VisionUtils.VisionStage;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class alignWithSpeaker extends Command {

  Swerve m_swerve;
  Launcher m_launcher;
  Vision m_vision;
  double speed;
  double seconds;
  Timer timer;
  boolean shouldStop;

  private double distance;
  private double verticalDistance_camToTag = VisionConstants.speakerAprilTagToGround - VisionConstants.camToGround;
  private double shooterPoseSpeed = 70;
  private double distanceToTag;
  private double angleToTag;
  private Rotation2d camRotation;
  private double verticalAngleToTag;
  private double robotHeading_current;
  private double angleHeading_new;
  private double debounce_count = 0;
  private double debounce_limit = 10;

  /**
   * Swerve requests
   */
  private final SwerveRequest.FieldCentricFacingAngle m_aim = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(SwerveConstants.kSpeedAt12VoltsMps * 0.1)
      .withRotationalDeadband((1.5 * Math.PI) * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withVelocityX(0)
      .withVelocityY(0);

  private final SwerveRequest.FieldCentric pid_aim = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.kSpeedAt12VoltsMps * 0.1)
      .withRotationalDeadband((1.5 * Math.PI) * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withVelocityX(0)
      .withVelocityY(0);

  /** Creates a new shootNoteVisually. */
  public alignWithSpeaker(
      Launcher sentLauncher,
      Swerve sentSwerve,
      Vision sentVision,
      double sentSpeed,
      double sentSeconds,
      boolean stopShooting) {
    m_launcher = sentLauncher;
    m_swerve = sentSwerve;
    m_vision = sentVision;
    speed = sentSpeed;
    seconds = sentSeconds;
    timer = new Timer();
    m_vision.setTargetID(RobotContainer.tagData.SpeakerCenter.tag_id);
    m_aim.HeadingController.setPID(20, 0, 0.05);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher, m_swerve, m_vision);
  }

  public Rotation2d rot2d(double deg) {
    return new Rotation2d(deg);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  private double kP = 0.07;

  PIDController pid = new PIDController(kP, 0, 0.005);

  private void tickCalculations() {
    angleToTag = m_vision.getX();
    robotHeading_current = m_swerve.getPigeon2().getAngle();
    angleHeading_new = angleToTag + robotHeading_current;
    camRotation = rot2d(angleHeading_new);

    verticalAngleToTag = m_vision.getY();
    distanceToTag = verticalDistance_camToTag / Math.tan(Math.toRadians(verticalAngleToTag));
  }

  private void failWithError(String error) {
    System.out.println(
        "[ERROR] Command 'shootNoteVisually' failed with error: `" + error + '`');
    end(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // distance = m_vision.getTargetDistance();
    switch (m_vision.getStage()) {
      case FIND_TARGET: {
        failWithError(
            "No AprilTag with ID " + m_vision.getTargetID() + " found!");
        break;
      }
      case GOTO_TARGET: {
        // update current angle and distance values
        tickCalculations();
        // start going toward speaker and rev launcher intake
        m_launcher.runMotors(LauncherConstants.launcherSpeedLauncher);
        m_swerve.setControl(
            pid_aim.withRotationalRate(
                -pid.calculate(
                    m_swerve.getPigeon2().getAngle(),
                    angleHeading_new) *
                    (1.5 * Math.PI)));
        if (Math.abs(pid.getPositionError()) > VisionConstants.txTolerance)
          debounce_count = 0;
          // thanks 
        else if (debounce_count < debounce_limit)
          debounce_count++;
        // we're within the tx tolerance, FINISH THE COMMAND
        else {
          m_swerve.setControl(pid_aim.withRotationalRate(0));
          m_vision.setStage(VisionStage.IDLE);
        }
        break;
      }
      case IDLE: {
        break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (shouldStop)
      m_launcher.runMotors(0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() >= seconds || m_vision.getStage() == VisionStage.IDLE) {
      return true;
    } else {
      return false;
    }
  }
}
