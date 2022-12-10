package frc.robot.prime.drive.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.prime.models.PidConstants;
import frc.robot.sensors.MA3Encoder;

public class PrimeSwerveModule {
  static final double kWheelRadius = 5d;

  static final double kMaxAngularSpeed = PrimeSwerveDriveTrain.kMaxAngularSpeed;
  static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // in radians per second squared

  final TalonSRX m_steeringMotor;
  final WPI_TalonFX m_driveMotor;

  static final short kEncoderResolution = 4096;
  final MA3Encoder m_encoder;

  final ProfiledPIDController m_steeringPIDController;

  public PrimeSwerveModule(
    byte driveMotorId,
    byte steeringMotorId,
    byte encoderAioChannel,
    short encoderBasePositionOffset,
    PidConstants drivePidConstants,
    PidConstants steeringPidConstants
  ) {
    // Set up the steering motor
    m_steeringMotor = new TalonSRX(steeringMotorId);
    m_steeringMotor.configFactoryDefault();
    m_steeringMotor.setNeutralMode(NeutralMode.Brake);

    // Set up the drive motor
    m_driveMotor = new WPI_TalonFX(driveMotorId);
    m_driveMotor.setNeutralMode(NeutralMode.Coast);
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // The integrated sensor in the Falcon is the falcon's encoder
    m_driveMotor.config_kP(0, drivePidConstants.kP);
    m_driveMotor.config_kI(0, drivePidConstants.kI);
    m_driveMotor.config_kD(0, drivePidConstants.kD);
    m_driveMotor.configOpenloopRamp(0.1);

    // Set up our encoder
    m_encoder = new MA3Encoder(encoderAioChannel, encoderBasePositionOffset);

    // Create a PID controller to calculate steering motor output
    m_steeringPIDController = new ProfiledPIDController(
      steeringPidConstants.kP, 
      steeringPidConstants.kI, 
      steeringPidConstants.kD, 
      new TrapezoidProfile.Constraints(kMaxAngularSpeed, kModuleMaxAngularAcceleration));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), m_encoder.getRotation2d());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the state to avoid turning wheels further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, m_encoder.getRotation2d());

    m_driveMotor.set(ControlMode.Velocity, desiredState.speedMetersPerSecond);
    
    final var steeringOutput = m_steeringPIDController.calculate(m_encoder.getRotation2d().getRadians(), desiredState.angle.getRadians());
    m_steeringMotor.set(ControlMode.PercentOutput, steeringOutput);
  }
}
