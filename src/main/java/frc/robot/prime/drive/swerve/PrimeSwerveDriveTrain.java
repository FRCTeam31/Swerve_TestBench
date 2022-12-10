package frc.robot.prime.drive.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.config.RobotMap;
import frc.robot.prime.models.PidConstants;

public class PrimeSwerveDriveTrain {
  public static final double kMaxSpeed = 4.572; // in meters per second, 15 feet per second
  public static final double kMaxAngularSpeed = Math.PI; // 180 degrees per second, half rotation

  public static final PidConstants drivePidConstants = new PidConstants(1, 0, 0);
  public static final PidConstants steeringPidConstants = new PidConstants(1, 0, 0);

  final Translation2d frontLeftLocation = new Translation2d(1, 1);  // using 1's right now for ease of testing
  final Translation2d frontRightLocation = new Translation2d(1, -1);
  final Translation2d rearLeftLocation = new Translation2d(-1, 1);
  final Translation2d rearRightLocation = new Translation2d(-1, -1);

  final PrimeSwerveModule m_frontLeftModule = new PrimeSwerveModule(
    RobotMap.kFalcon1Id, 
    RobotMap.kTalon1Id, 
    RobotMap.kEncoder1AIOChannel, 
    RobotMap.kEncoderBasePositionOffset,
    drivePidConstants,
    steeringPidConstants);

  final AHRS m_gyro = new AHRS(Port.kUSB);

  final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation);

  public PrimeSwerveDriveTrain() {
    m_gyro.reset();
  }

  public void drive(double str, double fwd, double rot, boolean fieldRelative) {
    var chassisSpeeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(str, fwd, rot, m_gyro.getRotation2d())
      : new ChassisSpeeds(str, fwd, rot);

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
  }
}
