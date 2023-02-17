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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.DriveMap;
import frc.robot.prime.models.PidConstants;
import frc.robot.sensors.navx.AHRS;

public class Drivetrain extends SubsystemBase {
  // Default PID values for steering each module and driving each module
  public static final PidConstants kDrivePidConstants = new PidConstants(0.01);
  public static final PidConstants kSteeringPidConstants = new PidConstants(0.75);
  private final Field2d mField = new Field2d();

  // Initialize "locations" of each wheel in terms of x, y translation in meters
  // from the origin (middle of the robot)
  double halfWheelBase = DriveMap.kRobotWheelBaseMeters / 2;
  double halfTrackWidth = DriveMap.kRobotTrackWidthMeters / 2;
  final Translation2d frontLeftLocation = new Translation2d(-halfTrackWidth, halfWheelBase);
  final Translation2d frontRightLocation = new Translation2d(halfTrackWidth, halfWheelBase);
  final Translation2d rearLeftLocation = new Translation2d(-halfTrackWidth, -halfWheelBase);
  final Translation2d rearRightLocation = new Translation2d(halfTrackWidth, -halfWheelBase);

  // Build serve drive modules with encoder channel & offset, and CAN IDs for
  // drive and steering motors
  public SwerveModule mFrontLeftModule;
  public SwerveModule mFrontRightModule;
  public SwerveModule mRearLeftModule;
  public SwerveModule mRearRightModule;

  // Build a gyro and a kinematics class for our drive
  final AHRS mGyro = new AHRS(Port.kUSB);
  final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
      frontLeftLocation,
      frontRightLocation,
      rearLeftLocation,
      rearRightLocation);

  SwerveDriveOdometry mOdometry;

  /** Creates a new SwerveDriveTrainSubsystem. */
  public Drivetrain(SwerveModule frontLeftModule, SwerveModule frontRightModule, SwerveModule rearLeftModule,
      SwerveModule rearRightModule) {
    SmartDashboard.putData("Field", mField);
    mFrontLeftModule = frontLeftModule;

    new SwerveDriveOdometry(mKinematics,
        mGyro.getRotation2d(),
        new SwerveModulePosition[] {
            mFrontLeftModule.getPosition(),
            mFrontRightModule.getPosition(),
            mRearLeftModule.getPosition(),
            mRearRightModule.getPosition(),
        },
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var gyroAngle = mGyro.getRotation2d();
    SmartDashboard.putNumber("Drivetrain gyro angle", gyroAngle.getDegrees());

    var robotPose = mOdometry.update(gyroAngle, new SwerveModulePosition[] {
        mFrontLeftModule.getPosition(), mFrontRightModule.getPosition(),
        mRearLeftModule.getPosition(), mRearRightModule.getPosition()
    });

    mField.setRobotPose(robotPose);
  }

  public void resetGyro() {
    mGyro.reset();
  }

  public void drive(double strafe, double forward, double rotation, boolean fieldRelative) {
    var desiredChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(strafe, forward, rotation, mGyro.getRotation2d())
        : new ChassisSpeeds(strafe, forward, rotation);

    var swerveModuleStates = mKinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveMap.kDriveMaxSpeedMetersPerSecond);

    mFrontLeftModule.setDesiredState(swerveModuleStates[0]);
    mFrontRightModule.setDesiredState(swerveModuleStates[1]);
    mRearLeftModule.setDesiredState(swerveModuleStates[2]);
    mRearRightModule.setDesiredState(swerveModuleStates[3]);
  }
}
