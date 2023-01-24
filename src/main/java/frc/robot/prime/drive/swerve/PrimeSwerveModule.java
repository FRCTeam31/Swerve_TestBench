package frc.robot.prime.drive.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.MA3Encoder;

public class PrimeSwerveModule {
  static final short kEncoderResolution = 4096;
  static final double kWheelRadius = 5d;
  static final double kMaxAngularSpeed = PrimeSwerveDriveTrain.kMaxAngularSpeed;
  static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // in radians per second squared

  // final TalonSRX m_steeringMotor;
  public final WPI_TalonFX m_steeringMotor;
  public final WPI_TalonFX m_driveMotor;
  public final MA3Encoder m_encoder;
  final PIDController m_steeringPIDController;

  public PrimeSwerveModule(
    int driveMotorId,
    int steeringMotorId,
    int encoderAioChannel,
    short encoderBasePositionOffset,
    boolean invertDrive
  ) {
    // Set up the steering motor
    m_steeringMotor = new WPI_TalonFX(steeringMotorId);
    // m_steeringMotor = new TalonSRX(steeringMotorId);
    m_steeringMotor.configFactoryDefault();
    m_steeringMotor.setNeutralMode(NeutralMode.Brake);
    m_steeringMotor.setInverted(true);
    m_steeringMotor.configOpenloopRamp(0.2);


    // Set up the drive motor
    m_driveMotor = new WPI_TalonFX(driveMotorId);
    m_driveMotor.setNeutralMode(NeutralMode.Coast);
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // The integrated sensor in the Falcon is the falcon's encoder
    m_driveMotor.config_kP(0, PrimeSwerveDriveTrain.kDrivePidConstants.kP);
    m_driveMotor.config_kI(0, PrimeSwerveDriveTrain.kDrivePidConstants.kI);
    m_driveMotor.config_kD(0, PrimeSwerveDriveTrain.kDrivePidConstants.kD);
    m_driveMotor.configOpenloopRamp(0.2);
    m_driveMotor.configClosedloopRamp(0.2);
    m_driveMotor.setInverted(invertDrive);

    // Set up our encoder
    m_encoder = new MA3Encoder(encoderAioChannel, encoderBasePositionOffset);

    // Create a PID controller to calculate steering motor output
    m_steeringPIDController = new PIDController(
      PrimeSwerveDriveTrain.kSteeringPidConstants.kP, 
      PrimeSwerveDriveTrain.kSteeringPidConstants.kI, 
      PrimeSwerveDriveTrain.kSteeringPidConstants.kD
    );
    m_steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_steeringPIDController.setTolerance(1 / 1024);

    SmartDashboard.putData("Steering PID", m_steeringPIDController);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), m_encoder.getRotation2d());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the state to avoid turning wheels further than 90 degrees
    var encoderRotation = m_encoder.getRotation2d().rotateBy(new Rotation2d(90));
    desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);

    m_driveMotor.set(ControlMode.Velocity, desiredState.speedMetersPerSecond);
    var currentPositionRadians = encoderRotation.getRadians();
    var desiredPositionRadians = desiredState.angle.getRadians();
    
    var steeringOutput = m_steeringPIDController.calculate(currentPositionRadians, desiredPositionRadians);
    steeringOutput = MathUtil.applyDeadband(steeringOutput, 0.1);
    m_steeringMotor.set(ControlMode.PercentOutput, MathUtil.clamp(steeringOutput, -1, 1));
  }

  public void driveSimple(double fwd, double rot) {
    m_driveMotor.set(fwd);
    m_steeringMotor.set(rot);
  }
}
