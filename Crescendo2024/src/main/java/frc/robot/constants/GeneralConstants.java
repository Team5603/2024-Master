package frc.robot.constants;

public class GeneralConstants {
    public class IntakeConstants {
        public static final double liftDownSetpoint = 38.5;
        public static final double liftDownLimit = 39.1;
        public static final double liftUpLimit = 0.1;
        public static final double intakeSpeed = 0.4;
        public static final double intakeSpeedMultiplier = 0.25;
    }
    public class LauncherConstants {
        public static final int angleToEncoderTranslation = 4;
        public static final double launcherSpeedLauncher = 0.75;
        public static final double launcherSpeedAmp = .3;
        public static final double launcherIntakeSpeed = .225;
        public static final double liftDownLimitLow = .75;
        public static final double liftDownLimitHigh = 1;
        public static final double liftUpLimitLow = .25;
                                                                                                   
        public static final double liftUpLimitHigh = 0.75;
        public static final double absoluteEncoderZeroPoint = 0.243;
    }
    public class ClimbConstants {
        public static final double extendSpeedMultiplier = 1;
        public static final double extendEncoderLimit = 10;
    }
}