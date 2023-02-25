package frc.robot.config;

import prime.models.PidConstants;

public class DriveMap {
    // Physical measurements
    public static final double kRobotTrackWidthMeters = 0.6223;
    public static final double kRobotWheelBaseMeters = 0.6096;
    public static final double kRobotWheelBaseCircumferenceMeters = 2.73702;

    public static final double kDriveWheelDiameterMeters = 0.102;
    public static final byte kDriveMotorOutputTeeth = 13;
    public static final byte kDriveMotorDrivenGearTeeth = 42;

    // Calculated values
    public static final double kDriveGearRatio = kDriveMotorDrivenGearTeeth / kDriveMotorOutputTeeth;
    public static final double kDriveWheelCircumference = Math.PI * kDriveWheelDiameterMeters;

    // Measured SysId values
    public static final double driveKs = -0.13939;
    public static final double driveKv = 0.029115;
    public static final double driveKa = 0.0050108;
    public static final PidConstants kDrivePidConstants = new PidConstants(0.0016983);
    public static final PidConstants kSteeringPidConstants = new PidConstants(0.4);

    // FR
    public static int kFrontRightSteeringMotorId = 11;
    public static int kFrontRightDrivingMotorId = 10;
    public static int kFrontRightEncoderAIOChannel = 0;
    public static short kFrontRightEncoderOffset = 701 + 2048;

    // FL
    public static int kFrontLeftSteeringMotorId = 13;
    public static int kFrontLeftDrivingMotorId = 12;
    public static int kFrontLeftEncoderAIOChannel = 1;
    public static short kFrontLeftEncoderOffset = 1961;

    // RR
    public static int kRearRightSteeringMotorId = 14;
    public static int kRearRightDrivingMotorId = 15;
    public static int kRearRightEncoderAIOChannel = 2;
    public static short kRearRightEncoderOffset = 1115 + 2048;

    // RL
    public static int kRearLeftSteeringMotorId = 17;
    public static int kRearLeftDrivingMotorId = 16;
    public static int kRearLeftEncoderAIOChannel = 3;
    public static short kRearLeftEncoderOffset = 2030;

    // Gear ratios
    public static byte driveMotorOutputTeeth = 13;
    public static byte driveMotorDriveGearTeeth = 42;
    public static int falconTotalSensorUnits = 2048;
    public static final double kDriveMaxSpeedMetersPerSecond = 4.938; // 16.2ft per second in meters per second
    public static final double kDriveMaxAngularSpeed = DriveMap.kRobotWheelBaseCircumferenceMeters
            / kDriveMaxSpeedMetersPerSecond;
    public static final double kDriveLowGearCoefficient = 0.1;
}
