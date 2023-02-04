package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.sensors.MA3Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrimeSwerveModuleSubsystem  extends SubsystemBase{
 private WPI_TalonFX m_steeringMotor;
 private WPI_TalonFX m_driveMotor;
 private MA3Encoder m_encoder;
 private PIDController m_steeringPIDController;   

 public PrimeSwerveModuleSubsystem (
    int driveMotorId,
    int steeringMotorId,
    int encoderAioChannel,
    int encoderBasePositionOffset,
    boolean invertDrive
 ){
   //  m_steeringMotor = new WPI_TalonFX(steeringMotorId);
   //  m_steeringMotor.configFactoryDefault();

   //  m_driveMotor = new WPI_TalonFX(driveMotorId);
   //  m_driveMotor.configFactoryDefault();
   //  m_driveMotor.setInverted(TalonFXInvertType.Clockwise);
 
   //  m_encoder = new MA3Encoder(encoderAioChannel, encoderBasePositionOffset );
    
   //  m_steeringPIDController = new PIDController(
   //    SwerveDriveTrainSubsystem.kSteeringPidConstants.kP, 
   //    SwerveDriveTrainSubsystem.kSteeringPidConstants.kI, 
   //    SwerveDriveTrainSubsystem.kSteeringPidConstants.kD
   //  );

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

   m_drivePIDController = new PIDController(RobotMap.kDrivePidConstants.kP, 0, 0);
 }

 public void setDesiredState(SwerveModuleState desiredState){
    var encoderRotation = m_encoder.getRotation2d().rotateBy(new Rotation2d(90));
   //  desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);

    m_driveMotor.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond / SwerveDriveTrainSubsystem.kMaxSpeed);
    var currentPositionRadians = encoderRotation.getRadians();
    var desiredPositionRadians = desiredState.angle.getRadians();

    var steeringOutput = m_steeringPIDController.calculate(currentPositionRadians, desiredPositionRadians);
    steeringOutput += steeringOutput < 0 ? 0.2 : -0.2;
    m_steeringMotor.set(ControlMode.PercentOutput, -steeringOutput);
 }

 public void driveSimple(double fwd, double rot){
    m_driveMotor.set(fwd);
    m_steeringMotor.set(rot);
 }
}
