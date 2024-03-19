package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.entry;
import frc.robot.RobotLimelightHelpers;
import frc.robot.RobotLimelightHelpers.LimelightResults;
import frc.robot.RobotLimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.utils.VisionUtils;

public class Vision extends SubsystemBase implements VisionUtils {

    private NetworkTable nt_limelight;
    private LimelightResults vision_results;

    private double distance;
    private boolean targetFound = false;

    private int target_id = 3;
    private int target_offset = 5;

    private double tx = -100;
    private double ty = -100;

    private VisionStage visionStage;

    public Vision() {
        nt_limelight = NetworkTableInstance.getDefault().getTable("limelight");
        visionStage = VisionStage.INITIALIZED;
    }

    private void tickSmartDashboard() {
        SmartDashboard.putNumber("Distance:", distance);
        SmartDashboard.putBoolean("Target found:", targetFound);
        SmartDashboard.putNumber("tx:", getX());
        SmartDashboard.putNumber("ty:", getY());
        SmartDashboard.putNumber("target tx-offset: (+/-)", target_offset);
        SmartDashboard.putString("Stage:", getStage().toString());
    }

    private void tickTargets() {
        vision_results = RobotLimelightHelpers.getLatestResults(VisionConstants.limelightName);
        boolean tag_visible = false;
        for (LimelightTarget_Fiducial tag : vision_results.targetingResults.targets_Fiducials) {
            // System.out.println("tag is found");
            if (tag.fiducialID == target_id) {
                targetFound = true;
                tx = tag.tx;
                ty = tag.ty;
                tag_visible = true;
            }
        }
        if (!tag_visible) {
            targetFound = false;
            tx = -100;
            ty = -100;
        }
    }
    // State management
    public VisionStage getStage () {
        return visionStage;
    }
    public void setStage (VisionStage stage) {
        visionStage = stage;
    }
    private void tickStage () {
        switch (visionStage) {
            case INITIALIZED: {
                visionStage = targetFound ? VisionStage.GOTO_TARGET : VisionStage.FIND_TARGET;
                break;
            }
        }
    }

    @Override
    public void periodic() {
        distance = getTargetDistance(41, 200.5, 57, getY());
        tickSmartDashboard();
        tickTargets();
        tickStage();
    }

    // Targeting
    public void setTargetID(int id) {
        target_id = id;
    }

    public int getTargetID() {
        return target_id;
    }

    // Gets the distance in inches to an AprilTag target
    public double getTargetDistance(double camera_height, double camera_angle, double target_height,
            double target_verticalOffset) {
        double relative_height = target_height - camera_height;
        double angleToTarget = camera_angle + getY();
        double angleInRadians = Math.toRadians(angleToTarget);
        return (relative_height / Math.tan(angleInRadians));
    }

    private NetworkTableEntry getEntry(String entry) {
        return nt_limelight.getEntry(Entry(entry));
    }

    private double getDouble(String entry) {
        return getEntry(entry).getDouble(0);
    }

    private void setNumber(String entry, int value) {
        getEntry(entry).setNumber(value);
    }

    // x / y offsets of crosshair
    public double getX() {
        return tx;
    }

    public double getY() {
        return ty;
    }

    // get area of LL target
    public double getArea() {
        return getDouble(entry.ta);
    }

    // check if LL is targetting something
    public boolean hasTarget() {
        return getDouble(entry.tv) == 1;
    }

    // get / set LL vision pipeline
    public int getPipeline() {
        return (int) getDouble(entry.pipeline);
    }

    public void setPipeline(int id_pipeline) {
        if (id_pipeline < 0 || id_pipeline > 9)
            throw new IllegalArgumentException("Pipeline exceeds range!");
        setNumber(entry.pipeline, id_pipeline);
    }

    // // get list of april tags the LL sees
    // public void getTags() {
    // int pipeline = getPipeline();
    // setPipeline(pipelines.Fiducial);
    // Double[] tags = getEntry(entry.tid).getDoubleArray(new Double[6]);
    // setPipeline(pipeline);
    // SmartDashboard.putNumber("eeee", getEntry(entry.tid).getDouble(100
    // ));
    // return new TagList(tags);
    // }

    // // gets distance from LL to current target (in inches)
    // public double getRawDistance() {
    // double relativeHeight = VisionConstants.targetToGround -
    // VisionConstants.camToGround;
    // double angleToTarget = VisionConstants.leveledCamAngle + getY();
    // double angleInRads = (angleToTarget * 3.1415926) / 180;
    // return (relativeHeight / Math.tan(angleInRads));
    // }

    // /** gets horizontal distance from current target **/
    // public double getHorizontalDistance() {
    // /** long side of the triangle **/
    // double hypotenuse = getRawDistance();
    // /** the side of the triangle straight ahead of the limelight */
    // double opposite = VisionConstants.targetToGround -
    // VisionConstants.camToGround;
    // /** pythagorean theorem :) */
    // double pyth = (Math.pow(hypotenuse, 2) - Math.pow(opposite, 2));
    // return (Math.sqrt(pyth));
    // }

    // /** gets closest angle of the triangle formed from the raw and horizontal
    // distances **/
    // public double getVerticalAngleToTarget() {

    // double height = VisionConstants.targetToGround -
    // VisionConstants.speakerAprilTagToGround;

    // double distance = getHorizontalDistance();

    // double verticalAngleInRads = Math.atan(height/distance);

    // return Math.toDegrees(verticalAngleInRads);
    // }
}
