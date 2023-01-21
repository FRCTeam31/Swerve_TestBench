package frc.robot.config;

public class RobotMap {

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
  public static int kFrontRightSteeringMotorId = 10;
  public static int kFrontRightDrivingMotorId = 11;
  public static int kFrontRightEncoderAIOChannel = 0;
  public static short kFrontRightEncoderOffset = 698; // Test bench
  // public static short kFrontLeftEncoderOffset = 202; // Test bench

  // FL
  public static int kFrontLeftSteeringMotorId = 12;
  public static int kFrontLeftDrivingMotorId = 13;
  public static int kFrontLeftEncoderAIOChannel = 1;
  public static short kFrontLeftEncoderOffset = 1970; // Test bench

  // RR
  public static int kRearRightSteeringMotorId = 14;
  public static int kRearRightDrivingMotorId = 15;
  public static int kRearRightEncoderAIOChannel = 2;
  public static short kRearRightEncoderOffset = 1132; // Test bench

  // RL
  public static int kRearLeftSteeringMotorId = 16;
  public static int kRearLeftDrivingMotorId = 17;
  public static int kRearLeftEncoderAIOChannel = 3;
  public static short kRearLeftEncoderOffset = 3476; // Test bench

}
