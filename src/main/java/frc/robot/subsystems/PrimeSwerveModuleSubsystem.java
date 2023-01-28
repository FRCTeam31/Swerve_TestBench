package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.prime.drive.swerve.PrimeSwerveDriveTrain;
import frc.robot.prime.drive.swerve.PrimeSwerveModule;
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

    m_steeringMotor = new WPI_TalonFX(steeringMotorId);
    m_driveMotor = new WPI_TalonFX(driveMotorId);
    m_encoder = new MA3Encoder(encoderAioChannel, encoderBasePositionOffset );
    m_steeringPIDController = new PIDController(
      PrimeSwerveDriveTrain.kSteeringPidConstants.kP, 
      PrimeSwerveDriveTrain.kSteeringPidConstants.kI, 
      PrimeSwerveDriveTrain.kSteeringPidConstants.kD
    );
 }
 public void setDesiredState(SwerveModuleState desiredState){
    var encoderRotation = m_encoder.getRotation2d().rotateBy(new Rotation2d(90));
    desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);

    m_driveMotor.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond / PrimeSwerveDriveTrain.kMaxSpeed);
    var currentPositionRadians = encoderRotation.getRadians();
    var desiredPositionRadians = desiredState.angle.getRadians();

    var steeringOutput = m_steeringPIDController.calculate(currentPositionRadians, desiredPositionRadians);
    steeringOutput = MathUtil.applyDeadband(steeringOutput, 0.1);
    
    
 }

 public void driveSimple(double fwd, double rot){
    m_driveMotor.set(fwd);
    m_steeringMotor.set(rot);
 }
    
    
}
