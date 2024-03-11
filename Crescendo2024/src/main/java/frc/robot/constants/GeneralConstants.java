package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class GeneralConstants {
    public class MotorConstants {
        public static final CurrentLimitsConfigs nonDriveCurrentLimitCTRE = 
        new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);
        public static final int nonDriveCurrentLimitREV = 20;
    }
    public class IntakeConstants {
        public static final int intakeMotor = 38;
        public static final int liftMotor = 40;
        public static final int liftLimitSwitch = 9;

        public static final double liftDownSetpoint = 0.505;
        public static final double liftDownLimitLow = 0.505;
        public static final double liftDownLimitHigh = 0.75;
        public static final double liftUpLimitLow = .76;
        public static final double liftUpLimitHigh = 1;
        public static final double intakeSpeed = 0.5;
        public static final double intakeLiftSpeedMultiplier = 0.35;

        public static final double absoluteEncoderZeroPoint = 0.505;
    }
    public class LauncherConstants {
        public static final int shootLeftMotor = 10;
        public static final int shootRightMotor = 26;
        public static final int sensor = 1;

        public static final int liftLeftMotor = 12;
        public static final int liftRightMotor = 9;
        public static final int liftLimitSwitch = 0;

        public static final int angleToEncoderTranslation = 4;
        public static final double launcherSpeedLauncher = 0.90;
        public static final double launcherSpeedAmp = .3;
        public static final double launcherIntakeSpeed = .225;
        public static final double launcherIntakeSpeedSlower = .15;

        public static final double liftDownLimitLow = .75;
        public static final double liftDownLimitHigh = 1;
        public static final double liftUpLimitLow = .25;
                                                                                                   
        public static final double liftUpLimitHigh = 0.75;
        public static final double absoluteEncoderZeroPoint = 0.243;
    }
    public class ArmConstants {
        public static final int leftArmMotor = 41;
        public static final int rightArmMotor = 39;
        public static final int leftArmLimitSwitch = 7;
        public static final int rightArmLimitSwitch = 8;

        public static final double extendSpeedMultiplier = 1;
        public static final double extendEncoderLimit = 8;
    }
}