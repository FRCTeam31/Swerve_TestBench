package frc.robot.config;

import frc.robot.prime.models.PidConstants;

public class RobotMap {
  public static final double kRobotTrackWidthInches = 24.5;
  public static final double kRobotTrackWidthMeters = 0.6223;
  public static final double kRobotWheelBaseInches = 24;
  public static final double kRobotWheelBaseMeters = 0.6096;
  public static final double kRobotWheelBaseCircumference = 2.73702;//1

  public static final double kDriveWheelDiamter = 0.102;
  public static final byte kDriveMotorOutputTeeth = 13;
  public static final byte kDriveMotorDrivenGearTeeth = 42;
  public static final double kDriveGearRatio = kDriveMotorDrivenGearTeeth / kDriveMotorOutputTeeth;
  public static final double kDriveWheelCircumference = Math.PI * kDriveWheelDiamter;
  public static final double driveKs = -0.13939;
  public static final double driveKv = 0.029115;
  public static final double driveKa = 0.0050108;
  public static final double driveKp = 0.0016983;
  public static final double kDriveLowGearCoefficient = 0.6;
  public static final PidConstants kDrivePidConstants = new PidConstants(driveKp);

  public static final double steeringKp = 0.4;
  public static final double steeringKi = 0;
  public static final double steeringKd = 0;
  public static final PidConstants kSteeringPidConstants = new PidConstants(steeringKp, steeringKi, steeringKd);

  // Motor CAN Id Constants
  public static final  byte TALON1_CAN_ID = 3;
  public static final byte TALON2_CAN_ID = 2;
  public static final byte FALCON1_CAN_ID = 8;
  public static final byte SPARK1_CAN_ID = 4;

  //Joystick Constants
  public static final byte JOYSTICK1_PORT_NUMBER = 0;
  public static final byte TALON1_AXIS = 0;
  public static final byte TALON2_AXIS = 1;
  public static final byte SPARK1AXIS = 2;
  public static final byte FALCON1_AXIS = 3;
  public static final int SOlENOID_2_ID = 1;
  public static final int SOLENOID_1_ID = 0;
  public static final int FALCON_2_ID = 2;
  public static final int TALON_1_ID = 6;

  /**
   * 10, 12, 15, 16, // Drive controllers: FR, FL, RR, RL
        11, 13, 14, 17, // Rotation Controllers: FR, FL, RR, RL
        0, 1, 2, 3,     // Encoder AOI channels: 
        //FR    FL   RR    RL
        698, 1970, 1132, 3476,
   */

  // FR
  public static int kFrontRightSteeringMotorId = 11;
  public static int kFrontRightDrivingMotorId = 10;
  public static int kFrontRightEncoderAIOChannel = 0;
  // public static short kFrontRightEncoderOffset = 57;
  public static short kFrontRightEncoderOffset = 701 + 2048;

  // FL
  public static int kFrontLeftSteeringMotorId = 13;
  public static int kFrontLeftDrivingMotorId = 12;
  public static int kFrontLeftEncoderAIOChannel = 1;
  // public static short kFrontLeftEncoderOffset = 898;
  public static short kFrontLeftEncoderOffset = 1961;

  // RR
  public static int kRearRightSteeringMotorId = 14;
  public static int kRearRightDrivingMotorId = 15;
  public static int kRearRightEncoderAIOChannel = 2;
  // public static short kRearRightEncoderOffset = -350;
  public static short kRearRightEncoderOffset = 1115 + 2048;

  // RL
  public static int kRearLeftSteeringMotorId = 17;
  public static int kRearLeftDrivingMotorId = 16;
  public static int kRearLeftEncoderAIOChannel = 3;
  // public static short kRearLeftEncoderOffset = 798;
  public static short kRearLeftEncoderOffset = 2030;

  // Gear ratios
  public static byte driveMotorOutputTeeth = 13;
  public static byte driveMotorDriveGearTeeth = 42;
  // public static final double kDriveMaxSpeedMetersPerSecond = 0.27432; // in meters per second
  public static final double kDriveMaxSpeedMetersPerSecond = 4.938; // 16.2ft per second in meters per second
  public static final double kDriveMaxAngularSpeed = RobotMap.kRobotWheelBaseCircumference / kDriveMaxSpeedMetersPerSecond * 0.9; // 180 degrees per second, half rotation
}
