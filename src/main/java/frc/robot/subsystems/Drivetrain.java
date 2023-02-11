// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.DriveMap;
import frc.robot.sensors.navx.AHRS;

public class Drivetrain extends SubsystemBase {
  // Default PID values for steering each module and driving each module
  private SwerveModule mFrontLeftModule;
  private SwerveModule mFrontRightModule;
  private SwerveModule mRearLeftModule;
  private SwerveModule mRearRightModule;

  // Build a gyro and a kinematics class for our drive
  final AHRS mGyro = new AHRS(Port.kUSB);
  final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
    DriveMap.frontLeftLocation, 
    DriveMap.frontRightLocation, 
    DriveMap.rearLeftLocation, 
    DriveMap.rearRightLocation);

  SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(mKinematics, 
    mGyro.getRotation2d(), 
    new SwerveModulePosition[]{
      mFrontLeftModule.getPosition(),
      mFrontRightModule.getPosition(),
      mRearLeftModule.getPosition(),
      mRearRightModule.getPosition(),
    },
    new Pose2d(0, 0, Rotation2d.fromDegrees(90)));

  Field2d mField = new Field2d();

  /** Creates a new SwerveDriveTrainSubsystem. */
  public Drivetrain(SwerveModule flModule, SwerveModule frModule, SwerveModule rlModule, SwerveModule rrModule) {
    mFrontLeftModule = flModule;
    mFrontRightModule = frModule;
    mRearLeftModule = rlModule;
    mRearRightModule = rrModule;
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
