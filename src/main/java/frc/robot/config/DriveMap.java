package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.models.FeedForwardConstants;
import frc.robot.models.PidConstants;

public class DriveMap {
    // Drive
    public static final double kRobotTrackWidthInches = 24.5;
    public static final double kRobotTrackWidthMeters = 0.6223;
    public static final double kRobotWheelBaseInches = 24;
    public static final double kRobotWheelBaseMeters = 0.6096;
    public static final double kRobotWheelBaseCircumference = 2.73702;

    // Initialize "locations" of each wheel in terms of x, y translation in meters
    // from the origin (middle of the robot)
    public static final double halfWheelBase = DriveMap.kRobotWheelBaseMeters / 2;
    public static final double halfTrackWidth = DriveMap.kRobotTrackWidthMeters / 2;
    public static final Translation2d frontLeftLocation = new Translation2d(-halfTrackWidth, halfWheelBase);
    public static final Translation2d frontRightLocation = new Translation2d(halfTrackWidth, halfWheelBase);
    public static final Translation2d rearLeftLocation = new Translation2d(-halfTrackWidth, -halfWheelBase);
    public static final Translation2d rearRightLocation = new Translation2d(halfTrackWidth, -halfWheelBase);

    public static final double kDriveWheelDiamter = 0.102;
    public static final byte kDriveMotorOutputTeeth = 13;
    public static final byte kDriveMotorDrivenGearTeeth = 42;
    public static final double kDriveGearRatio = kDriveMotorDrivenGearTeeth / kDriveMotorOutputTeeth;
    public static final double kDriveWheelCircumference = Math.PI * kDriveWheelDiamter;
    public static final double driveKp = 0.0016983;
    public static final FeedForwardConstants kDriveFeedForwardConstants = new FeedForwardConstants(-0.13939, 0.029115,
            0.0050108);
    public static final PidConstants kDrivePidConstants = new PidConstants(driveKp);
    public static final double kDriveMaxSpeedMetersPerSecond = 4.938; // 16.2ft per second in meters per second
    public static final double kDriveMaxAngularSpeed = DriveMap.kRobotWheelBaseCircumference
            / kDriveMaxSpeedMetersPerSecond; // 180 degrees per second, half rotation

    // Steering
    public static final PidConstants kSteeringPidConstants = new PidConstants(0.4);
    public static int kFrontRightEncoderAIOChannel = 0;
    public static int kFrontLeftEncoderAIOChannel = 1;
    public static int kRearRightEncoderAIOChannel = 2;
    public static int kRearLeftEncoderAIOChannel = 3;
    public static short kFrontRightEncoderOffset = 2749;
    public static short kFrontLeftEncoderOffset = 1961;
    public static short kRearRightEncoderOffset = 3163;
    public static short kRearLeftEncoderOffset = 2030;
}