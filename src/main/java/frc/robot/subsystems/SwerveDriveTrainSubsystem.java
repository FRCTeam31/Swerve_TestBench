// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotMap;
import frc.robot.prime.models.PidConstants;
import frc.robot.sensors.navx.AHRS;

public class SwerveDriveTrainSubsystem extends SubsystemBase {
  // Default PID values for steering each module and driving each module
  public static final PidConstants kDrivePidConstants = new PidConstants(0.01);
  public static final PidConstants kSteeringPidConstants = new PidConstants(0.75);

  // Initialize "locations" of each wheel in terms of x, y translation in meters from the origin (middle of the robot)
  double halfWheelBase = RobotMap.kRobotWheelBaseMeters / 2;
  double halfTrackWidth = RobotMap.kRobotTrackWidthMeters / 2;
  final Translation2d frontLeftLocation =   new Translation2d(-halfTrackWidth, halfWheelBase);
  final Translation2d frontRightLocation =  new Translation2d(halfTrackWidth,  halfWheelBase);
  final Translation2d rearLeftLocation =    new Translation2d(-halfTrackWidth, -halfWheelBase);
  final Translation2d rearRightLocation =   new Translation2d(halfTrackWidth,  -halfWheelBase);

  // Build serve drive modules with encoder channel & offset, and CAN IDs for drive and steering motors
  // Front Left
  public final PrimeSwerveModuleSubsystem m_frontLeftModule = new PrimeSwerveModuleSubsystem(
    RobotMap.kFrontLeftDrivingMotorId, 
    RobotMap.kFrontLeftSteeringMotorId, 
    RobotMap.kFrontLeftEncoderAIOChannel,
    RobotMap.kFrontLeftEncoderOffset);

  // Front Right
  public final PrimeSwerveModuleSubsystem m_frontRightModule = new PrimeSwerveModuleSubsystem (
    RobotMap.kFrontRightDrivingMotorId, 
    RobotMap.kFrontRightSteeringMotorId, 
    RobotMap.kFrontRightEncoderAIOChannel, 
    RobotMap.kFrontRightEncoderOffset);

  // Rear Left
  public final PrimeSwerveModuleSubsystem m_rearLeftModule = new PrimeSwerveModuleSubsystem (
    RobotMap.kRearLeftDrivingMotorId, 
    RobotMap.kRearLeftSteeringMotorId, 
    RobotMap.kRearLeftEncoderAIOChannel, 
    RobotMap.kRearLeftEncoderOffset);

  // Rear Right
  public final PrimeSwerveModuleSubsystem m_rearRightModule = new PrimeSwerveModuleSubsystem (
    RobotMap.kRearRightDrivingMotorId, 
    RobotMap.kRearRightSteeringMotorId, 
    RobotMap.kRearRightEncoderAIOChannel, 
    RobotMap.kRearRightEncoderOffset);

  // Build a gyro and a kinematics class for our drive
  final AHRS m_gyro = new AHRS(Port.kUSB);
  final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    frontLeftLocation, 
    frontRightLocation, 
    rearLeftLocation, 
    rearRightLocation);

    SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(m_kinematics, 
      m_gyro.getRotation2d(), 
      new SwerveModulePosition[]{
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_rearLeftModule.getPosition(),
        m_rearRightModule.getPosition(),
      },
      new Pose2d(0, 0, Rotation2d.fromDegrees(90)));


    

  /** Creates a new SwerveDriveTrainSubsystem. */
  public SwerveDriveTrainSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var gyroAngle = m_gyro.getRotation2d();
    SmartDashboard.putNumber("Drivetrain gyro angle", gyroAngle.getDegrees());

    var robotPose = m_Odometry.update(gyroAngle, new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
      m_rearLeftModule.getPosition(), m_rearRightModule.getPosition()
    });
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void drive(double strafe, double forward, double rotation, boolean fieldRelative) {
    var desiredChassisSpeeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(strafe, forward, rotation, m_gyro.getRotation2d())
      : new ChassisSpeeds(strafe, forward, rotation);

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RobotMap.kDriveMaxSpeedMetersPerSecond);

    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_frontRightModule.setDesiredState(swerveModuleStates[1]);
    m_rearLeftModule.setDesiredState(swerveModuleStates[2]);
    m_rearRightModule.setDesiredState(swerveModuleStates[3]);
  }
}
