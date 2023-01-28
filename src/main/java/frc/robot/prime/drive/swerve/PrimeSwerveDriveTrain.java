package frc.robot.prime.drive.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.RobotMap;

public class PrimeSwerveDriveTrain {
  public static final double kMaxSpeed = 1; // in meters per second
  public static final double kMaxAngularSpeed = Math.PI * 0.66; // 180 degrees per second, half rotation

  // Initialize "locations" of each wheel in terms of x, y translation in meters from the origin (middle of the robot)
  double halfWheelBase = RobotMap.kRobotWheelBaseMeters / 2;
  double halfTrackWidth = RobotMap.kRobotTrackWidthMeters / 2;
  final Translation2d frontLeftLocation =   new Translation2d(-halfTrackWidth, halfWheelBase);  // using 1's right now for ease of testing
  final Translation2d frontRightLocation =  new Translation2d(halfTrackWidth,  halfWheelBase);
  final Translation2d rearLeftLocation =    new Translation2d(-halfTrackWidth, -halfWheelBase);
  final Translation2d rearRightLocation =   new Translation2d(halfTrackWidth,  -halfWheelBase);

  // Build serve drive modules with encoder channel & offset, and CAN IDs for drive and steering motors
  // Front Left
  public final PrimeSwerveModule m_frontLeftModule = new PrimeSwerveModule(
    RobotMap.kFrontLeftDrivingMotorId, 
    RobotMap.kFrontLeftSteeringMotorId, 
    RobotMap.kFrontLeftEncoderAIOChannel,
    RobotMap.kFrontLeftEncoderOffset,
    true);

  // Front Right
  public final PrimeSwerveModule m_frontRightModule = new PrimeSwerveModule(
    RobotMap.kFrontRightDrivingMotorId, 
    RobotMap.kFrontRightSteeringMotorId, 
    RobotMap.kFrontRightEncoderAIOChannel, 
    RobotMap.kFrontRightEncoderOffset,
    false);

  // Rear Left
  public final PrimeSwerveModule m_rearLeftModule = new PrimeSwerveModule(
    RobotMap.kRearLeftDrivingMotorId, 
    RobotMap.kRearLeftSteeringMotorId, 
    RobotMap.kRearLeftEncoderAIOChannel, 
    RobotMap.kRearLeftEncoderOffset,
    true);

  // Rear Right
  public final PrimeSwerveModule m_rearRightModule = new PrimeSwerveModule(
    RobotMap.kRearRightDrivingMotorId, 
    RobotMap.kRearRightSteeringMotorId, 
    RobotMap.kRearRightEncoderAIOChannel, 
    RobotMap.kRearRightEncoderOffset,
    false);

  // Build a gyro and a kinematics class for our drive
  // final AHRS m_gyro = new AHRS(Port.kUSB);
  final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    frontLeftLocation,
    rearLeftLocation, 
    rearRightLocation,
    frontRightLocation);

  public PrimeSwerveDriveTrain() {
    
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void drive(double str, double fwd, double rot, boolean fieldRelative) {
    var desiredChassisSpeeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(str, fwd, rot, m_gyro.getRotation2d())
      : new ChassisSpeeds(str, fwd, rot);

    SmartDashboard.putNumber("desired heading:", Math.tan(desiredChassisSpeeds.vyMetersPerSecond / desiredChassisSpeeds.vxMetersPerSecond));

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_rearLeftModule.setDesiredState(swerveModuleStates[1]);
    m_rearRightModule.setDesiredState(swerveModuleStates[2]);
    m_frontRightModule.setDesiredState(swerveModuleStates[3]);
  }
}
