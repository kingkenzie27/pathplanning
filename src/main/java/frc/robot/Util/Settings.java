package frc.robot.Util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Settings {


    public static final double kTrackwidthMeters = 0.5842;
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

    public static final double kMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 10;

    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
    new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);

    public static int DRIVE_FR_ID = 1;
    public static int STEER_FR_ID = 2;
    public static int DRIVE_RR_ID = 3;
    public static int STEER_RR_ID = 4;
    public static int DRIVE_RL_ID = 5;
    public static int STEER_RL_ID = 6;
    public static int DRIVE_FL_ID = 7;
    public static int STEER_FL_ID = 8;

    public static int ENCODER_FR_ID = 12;
    public static int ENCODER_RR_ID = 13;
    public static int ENCODER_RL_ID = 14;
    public static int ENCODER_FL_ID = 15;
}
