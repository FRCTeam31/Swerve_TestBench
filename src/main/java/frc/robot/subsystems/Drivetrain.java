// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.DriveMap;
import frc.robot.sensors.navx.AHRS;

public class Drivetrain extends SubsystemBase {
  // Default PID values for steering each module and driving each module
  private SwerveModule m_frontLeftModule;
  private SwerveModule m_frontRightModule;
  private SwerveModule m_rearLeftModule;
  private SwerveModule m_rearRightModule;

  // Build a gyro and a kinematics class for our drive
  final AHRS m_gyro = new AHRS(Port.kUSB);
  final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    DriveMap.frontLeftLocation, 
    DriveMap.frontRightLocation, 
    DriveMap.rearLeftLocation, 
    DriveMap.rearRightLocation);

  /** Creates a new SwerveDriveTrainSubsystem. */
  public Drivetrain(SwerveModule flModule, SwerveModule frModule, SwerveModule rlModule, SwerveModule rrModule) {
    m_frontLeftModule = flModule;
    m_frontRightModule = frModule;
    m_rearLeftModule = rlModule;
    m_rearRightModule = rrModule;
  }

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
