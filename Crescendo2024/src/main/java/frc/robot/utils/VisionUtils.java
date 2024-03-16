package frc.robot.utils;

public interface VisionUtils {
    public enum VisionStage {
        INITIALIZED,
        IDLE,
        FIND_TARGET,
        GOTO_TARGET,
        SHOOT
    }
    public default String Entry(String entry) {
        switch (entry) {
            case "tx":
            case "ta":
            case "ty":
            case "tv":
            case "camMode":
            case "tid":
            case "pipeline":
                return entry;
            default:
                throw new IllegalArgumentException("NT: Invalid entry!");
        }
    }
}