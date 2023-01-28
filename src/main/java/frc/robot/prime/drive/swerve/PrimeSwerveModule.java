package frc.robot.prime.drive.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.RobotMap;
import frc.robot.sensors.MA3Encoder;

public class PrimeSwerveModule {
  static final short kEncoderResolution = 2048;
  static final double kWheelRadius = 4d;
  static final double kMaxAngularSpeed = PrimeSwerveDriveTrain.kMaxAngularSpeed;
  static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // in radians per second squared

  public final WPI_TalonFX m_steeringMotor;
  public final WPI_TalonFX m_driveMotor;
  public final MA3Encoder m_encoder;
  final PIDController m_steeringPIDController;
  final PIDController m_drivePIDController;
  final SimpleMotorFeedforward m_driveFeedforward;

  public PrimeSwerveModule(
    int driveMotorId,
    int steeringMotorId,
    int encoderAioChannel,
    short encoderBasePositionOffset,
    boolean invertDrive
  ) {
    // Set up the steering motor
    m_steeringMotor = new WPI_TalonFX(steeringMotorId);
    m_steeringMotor.configFactoryDefault();
    m_steeringMotor.setNeutralMode(NeutralMode.Brake);
    m_steeringMotor.setInverted(true);
    m_steeringMotor.configOpenloopRamp(0.2);


    // Set up the drive motor
    m_driveMotor = new WPI_TalonFX(driveMotorId);
    m_driveMotor.configFactoryDefault();
    m_driveMotor.clearStickyFaults();
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // The integrated sensor in the Falcon is the falcon's encoder
    // m_driveMotor.config_kP(0, RobotMap.kDrivePidConstants.kP);
    m_driveMotor.configOpenloopRamp(0.2);
    // m_driveMotor.configClosedloopRamp(0.2);
    m_driveMotor.setInverted(invertDrive);

    // Set up our encoder
    m_encoder = new MA3Encoder(encoderAioChannel, encoderBasePositionOffset);

    // Create a PID controller to calculate steering motor output
    m_driveFeedforward = new SimpleMotorFeedforward(RobotMap.driveKs, RobotMap.driveKv, RobotMap.driveKa);
    m_steeringPIDController = new PIDController(
      RobotMap.kSteeringPidConstants.kP, 
      RobotMap.kSteeringPidConstants.kI, 
      RobotMap.kSteeringPidConstants.kD
    );
    m_steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_steeringPIDController.setTolerance(0.1);

    SmartDashboard.putData("Steering PID", m_steeringPIDController);

    m_drivePIDController = new PIDController(RobotMap.kDrivePidConstants.kP, 0, 0);
    SmartDashboard.putData("Driving PID", m_drivePIDController);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), m_encoder.getRotation2d());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the state to avoid turning wheels further than 90 degrees
    var encoderRotation = m_encoder.getRotation2d().rotateBy(Rotation2d.fromDegrees(105));
    desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Drive motor logic
    var currentVelocityInRotationsPer20ms = RobotMap.kDriveGearRatio * ((m_driveMotor.getSelectedSensorVelocity(0) / 5) / kEncoderResolution);
    var currentVelocityInMetersPer20ms = RobotMap.kDriveWheelCircumference * currentVelocityInRotationsPer20ms;
    var desiredVelocity = (desiredState.speedMetersPerSecond / 50) * 2048;
    var driveFeedforward = m_driveFeedforward.calculate(currentVelocityInMetersPer20ms, desiredVelocity, 0.2);
    var driveFeedback = m_drivePIDController.calculate(currentVelocityInMetersPer20ms, desiredVelocity);
    var desiredMotorVelocity = driveFeedback + driveFeedforward;
    m_driveMotor.set(ControlMode.PercentOutput, MathUtil.applyDeadband(desiredMotorVelocity, 0.15)); // Multipied by two to match falcon PID period
    
    // Steering motor logic
    var currentPositionRadians = encoderRotation.getRadians();
    var desiredPositionRadians = desiredState.angle.getRadians();
    var steeringOutput = m_steeringPIDController.calculate(currentPositionRadians, desiredPositionRadians);
    steeringOutput = MathUtil.applyDeadband(steeringOutput, 0.05);
    m_steeringMotor.set(ControlMode.PercentOutput, MathUtil.clamp(steeringOutput, -1, 1));
  }

  public void driveSimple(double fwd, double rot) {
    m_driveMotor.set(fwd);
    m_steeringMotor.set(rot);
  }
}
