package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.SwerveConstants;
// import frc.robot.utils.SwerveUtils;
import frc.robot.constants.VisionConstants.AprilTag;

public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public Swerve(SwerveDrivetrainConstants swerveConstants, double odometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(swerveConstants, odometryUpdateFrequency, modules);

        configurePathPlanner();
        m_odometry.addVisionMeasurement(null, odometryUpdateFrequency, null);
        if (Utils.isSimulation())
            startSimThread();
    }

    public Swerve(SwerveDrivetrainConstants swerveConstants, SwerveModuleConstants... modules) {
        super(swerveConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation())
            startSimThread();
    }

    // @Override
    // public void periodic() {
    // m_odometry.update(m_pigeon2.getRotation2d(), getModulePositions());
    // }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        SwerveConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                Modules[0].getPosition(true),
                Modules[1].getPosition(true),
                Modules[2].getPosition(true),
                Modules[3].getPosition(true)
        };
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_yawGetter.getValueAsDouble(), m_fieldRelativeOffset);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    // used for scheduling a PathPlanner Path as a Command
    public Command followPathFromFile(
            String pathToFollow,
            double speedConstraint,
            double accelerationConstraint,
            double RotationConstraint,
            double rotationAccelerationConstraint,
            boolean turnFast) {
        // Load the path we want to pathfild to and follow
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathToFollow);

        // register the constraints to use while pathfilding
        // the constraints defined in the path will only be used for this path.
        PathConstraints constraints = new PathConstraints(speedConstraint, accelerationConstraint,
                Units.degreesToRadians(RotationConstraint), Units.degreesToRadians(rotationAccelerationConstraint));

        // Since AutoBuilder is configured, we can use it to build pathfinding Commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                turnFast ?
                // Rotation delay distance in meters. This is how far the robot should travel
                // before attempting to rotate.
                        1.0 // turns after 1 meter if it is turnFast is true
                        : 3.0 // turns after 3 meters if it is turnFast is false)
        );
        return pathfindingCommand;
    }

    // Do not use for the Speaker AprilTags! Those will have a pre-programmed path
    public Command followAprilTagPath(
            AprilTag tag,
            double speedConstraint,
            double accelerationConstraint,
            double RotationConstraint,
            double rotationAccelerationConstraint) {
        Pose2d targetPose = new Pose2d(tag.xPos, tag.yPos, Rotation2d
                .fromDegrees((tag.direction) < 0 ? Math.abs(tag.direction) - 180 : 180 - Math.abs(tag.direction)));

        // Create the constraints the use while pathfinding
        PathConstraints constraints = new PathConstraints(speedConstraint, accelerationConstraint,
                Units.degreesToRadians(RotationConstraint), Units.degreesToRadians(rotationAccelerationConstraint));
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, 0.0);
        return pathfindingCommand;
    }

}
