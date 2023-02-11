// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.DriveMap;
import frc.robot.models.PidConstants;
import frc.robot.sensors.navx.AHRS;

public class SwerveModule extends SubsystemBase {
  // Default PID values for steering each module and driving each module
  public static final PidConstants kDrivePidConstants = new PidConstants(0.01);
  public static final PidConstants kSteeringPidConstants = new PidConstants(0.75);

  // Initialize "locations" of each wheel in terms of x, y translation in meters from the origin (middle of the robot)
  double halfWheelBase = DriveMap.kRobotWheelBaseMeters / 2;
  double halfTrackWidth = DriveMap.kRobotTrackWidthMeters / 2;
  final Translation2d frontLeftLocation =   new Translation2d(-halfTrackWidth, halfWheelBase);
  final Translation2d frontRightLocation =  new Translation2d(halfTrackWidth,  halfWheelBase);
  final Translation2d rearLeftLocation =    new Translation2d(-halfTrackWidth, -halfWheelBase);
  final Translation2d rearRightLocation =   new Translation2d(halfTrackWidth,  -halfWheelBase);

  // Build serve drive modules with encoder channel & offset, and CAN IDs for drive and steering motors
  // Front Left
  public final Drivetrain m_frontLeftModule = new Drivetrain(
    DriveMap.kFrontLeftDrivingMotorId, 
    DriveMap.kFrontLeftSteeringMotorId, 
    DriveMap.kFrontLeftEncoderAIOChannel,
    DriveMap.kFrontLeftEncoderOffset);

  // Front Right
  public final Drivetrain m_frontRightModule = new Drivetrain (
    DriveMap.kFrontRightDrivingMotorId, 
    DriveMap.kFrontRightSteeringMotorId, 
    DriveMap.kFrontRightEncoderAIOChannel, 
    DriveMap.kFrontRightEncoderOffset);

  // Rear Left
  public final Drivetrain m_rearLeftModule = new Drivetrain (
    DriveMap.kRearLeftDrivingMotorId, 
    DriveMap.kRearLeftSteeringMotorId, 
    DriveMap.kRearLeftEncoderAIOChannel, 
    DriveMap.kRearLeftEncoderOffset);

  // Rear Right
  public final Drivetrain m_rearRightModule = new Drivetrain (
    DriveMap.kRearRightDrivingMotorId, 
    DriveMap.kRearRightSteeringMotorId, 
    DriveMap.kRearRightEncoderAIOChannel, 
    DriveMap.kRearRightEncoderOffset);

  // Build a gyro and a kinematics class for our drive
  final AHRS m_gyro = new AHRS(Port.kUSB);
  final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    frontLeftLocation, 
    frontRightLocation, 
    rearLeftLocation, 
    rearRightLocation);

  /** Creates a new SwerveDriveTrainSubsystem. */
  public SwerveModule() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Drivetrain gyro angle", m_gyro.getAngle());
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void drive(double strafe, double forward, double rotation, boolean fieldRelative) {
    var desiredChassisSpeeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(strafe, forward, rotation, m_gyro.getRotation2d())
      : new ChassisSpeeds(strafe, forward, rotation);

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveMap.kDriveMaxSpeedMetersPerSecond);

    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_frontRightModule.setDesiredState(swerveModuleStates[1]);
    m_rearLeftModule.setDesiredState(swerveModuleStates[2]);
    m_rearRightModule.setDesiredState(swerveModuleStates[3]);
  }
}
