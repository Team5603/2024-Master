package frc.robot.constants;

public class VisionConstants {
    /* placeholder values!!!!! */
    public final static double camToGround = 20;
    public final static double launcherToGround = 25;
    public final static double targetToGround = 40;
    public final static double speakerAprilTagToGround = 30;
    public final static double leveledCamAngle = 30;
    final static int targetRange_px = 30;

    public final static String limelightName = "limelight";
    public final static double txTolerance = 3;


    public interface pipeline {
        final static int Fiducial = 0;
    }

    public interface entry {
        final static String tx = "tx";
        final static String ta = "ta";
        final static String ty = "ty";
        final static String tv = "tv";
        final static String camMode = "camMode";
        final static String tid = "tid";
        final static String pipeline = "pipeline";
    }

    public static class AprilTag {
        public int tag_id = 0;
        public double xPos, yPos, xOffset, yOffset, direction = 0;

        public AprilTag(int tag_id, double xPos, double yPos, double xOffset, double yOffset, double direction) {
            this.tag_id = tag_id;
            this.xPos = xPos + xOffset;
            this.yPos = yPos + yOffset;
            this.direction = direction;
        }
    }

    public static class SetPose {
        public double xPos, yPos, direction;

        public SetPose(double xPos, double yPos, double direction) {
            this.xPos = xPos;
            this.yPos = yPos;
            this.direction = direction;
        }
    }

    public static class TagData {
        public AprilTag Amp, SpeakerCenter, SpeakerOffset, TrapLeft, TrapRight, TrapBack, SourceLeft, SourceRight,
                SourceCenter;
        SetPose SpeakerMiddlePose, SpeakerLeftPose, SpeakerRightPose;

        public void setAlliance(edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
            switch (alliance) {
                case Blue: {
                    Amp = new AprilTag(6, 1.85, 8.15, -90, 0, 0);
                    SpeakerCenter = new AprilTag(7, 0.05, 5.55, 0, 0, 0);
                    SpeakerOffset = new AprilTag(8, 0, 0, 0, 0, 0);
                    TrapRight = new AprilTag(16, 4.7, 3.75, -120, 0, 0);
                    TrapLeft = new AprilTag(15, 4.7, 4.4, 120, 0, 0);
                    TrapBack = new AprilTag(14, 5.2, 4.1, 0, 0, 0);
                    SourceLeft = new AprilTag(2, 16.25, 0.75, 120, 0, 0);
                    SourceRight = new AprilTag(1, 15.25, 0.12, 120, 0, 0);
                    // Source Center does not exist. This is purely for the logic of the robot; ID
                    // used is 20
                    SourceCenter = new AprilTag(20, 0, 0, 0, 0, 0);

                    SpeakerMiddlePose = new SetPose(1.45, 5.50, 180);
                    SpeakerLeftPose = new SetPose(0.8, 6.75, -125);
                    SpeakerRightPose = new SetPose(0.8, 4.30, 125);
                }
                case Red: {
                    Amp = new AprilTag(5, 14.75, 8.15, -90, 0, 0);
                    SpeakerCenter = new AprilTag(4, 16.5, 5.55, 180, 0, 0);
                    SpeakerOffset = new AprilTag(3, 16.5, 4.75, 180, 0, 0);
                    TrapRight = new AprilTag(12, 11.85, 4.45, 60, 0, 0);
                    TrapLeft = new AprilTag(11, 11.85, 3.75, -60, 0, 0);
                    TrapBack = new AprilTag(13, 0, 4.1, 180, 0, 0);
                    SourceLeft = new AprilTag(10, 1.35, 0.12, 60, 0, 0);
                    SourceRight = new AprilTag(9, 0.3, 0.75, 60, 0, 0);
                    // Source Center does not exist. This is purely for the logic of the robot; ID
                    // used is 21
                    SourceCenter = new AprilTag(21, 0, 0, 0, 0, 0);

                    SpeakerMiddlePose = new SetPose(15.10, 5.50, 0);
                    SpeakerLeftPose = new SetPose(15.85, 4.30, 55);
                    SpeakerRightPose = new SetPose(15.10, 6.75, -55);
                }
            }
        }
    }
}