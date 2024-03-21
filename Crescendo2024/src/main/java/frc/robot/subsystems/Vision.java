package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants.entry;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.VisionUtils;

public class Vision extends SubsystemBase implements VisionUtils {

    private NetworkTable nt_limelight;

    public Vision() {
        nt_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("tx", getX());
        // SmartDashboard.putNumber("ty", getY());
        // SmartDashboard.putNumber("ta", getArea());
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
        return getDouble(entry.tx);
    }

    public double getY() {
        return getDouble(entry.ty);
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

    // get list of april tags the LL sees
    public void getTags() {
        // int pipeline = getPipeline();
        // setPipeline(pipelines.Fiducial);
        // Double[] tags = getEntry(entry.tid).getDoubleArray(new Double[6]);
        // setPipeline(pipeline);
        // SmartDashboard.putNumber("eeee", getEntry(entry.tid).getDouble(100
        // ));
        // return new TagList(tags);
    }

    // gets distance from LL to current target (in inches)
    public double getRawDistance() {
        double relativeHeight = VisionConstants.targetToGround - VisionConstants.camToGround;
        double angleToTarget = VisionConstants.leveledCamAngle + getY();
        double angleInRads = (angleToTarget * 3.1415926) / 180;
        return (relativeHeight / Math.tan(angleInRads));
    }

    /** gets horizontal distance from current target **/
    public double getHorizontalDistance() {
        /** long side of the triangle **/
        double hypotenuse = getRawDistance();
        /** the side of the triangle straight ahead of the limelight */
        double opposite = VisionConstants.targetToGround - VisionConstants.camToGround;
        /** pythagorean theorem :) */
        double pyth = (Math.pow(hypotenuse, 2) - Math.pow(opposite, 2));
        return (Math.sqrt(pyth));
    }

    /** gets closest angle of the triangle formed from the raw and horizontal distances **/
    public double getVerticalAngleToTarget() {
        
        double height =  VisionConstants.targetToGround - VisionConstants.speakerAprilTagToGround;

        double distance = getHorizontalDistance();

        double verticalAngleInRads = Math.atan(height/distance);

        return Math.toDegrees(verticalAngleInRads);
    }
}
