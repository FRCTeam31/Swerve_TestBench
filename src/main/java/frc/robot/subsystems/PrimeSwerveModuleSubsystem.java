package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.config.RobotMap;
import frc.robot.sensors.MA3Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrimeSwerveModuleSubsystem extends PIDSubsystem {
 private WPI_TalonFX mSteeringMotor;
 private WPI_TalonFX mDriveMotor;
 public MA3Encoder mEncoder;
 private PIDController mSteeringPIDController;   
 private PIDController mDrivePIDController;
 private SimpleMotorFeedforward mDriveFeedforward;   
   
 public PrimeSwerveModuleSubsystem (
    int driveMotorId,
    int steeringMotorId,
    int encoderAioChannel,
    int encoderBasePositionOffset
 ){
   // Congfigure PID
   super(new PIDController(RobotMap.kDrivePidConstants.kP, RobotMap.kDrivePidConstants.kI, RobotMap.kDrivePidConstants.kD));
   getController().enableContinuousInput(-Math.PI, Math.PI);

   // Set up the steering motor
   mSteeringMotor = new WPI_TalonFX(steeringMotorId);
   mSteeringMotor.configFactoryDefault();
   mSteeringMotor.clearStickyFaults();
   mSteeringMotor.setNeutralMode(NeutralMode.Brake);
   mSteeringMotor.setInverted(TalonFXInvertType.Clockwise);
   mSteeringMotor.configOpenloopRamp(0.2);


   // Set up the drive motor
   mDriveMotor = new WPI_TalonFX(driveMotorId);
   mDriveMotor.configFactoryDefault();
   mDriveMotor.clearStickyFaults();
   mDriveMotor.setNeutralMode(NeutralMode.Brake);
   mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // The integrated sensor in the Falcon is the falcon's encoder
   mDriveMotor.configOpenloopRamp(0.2);
   mDriveMotor.setInverted(TalonFXInvertType.Clockwise);

   // Set up our encoder
   mEncoder = new MA3Encoder(encoderAioChannel, encoderBasePositionOffset, true);

   // Create a PID controller to calculate steering motor output
   mDriveFeedforward = new SimpleMotorFeedforward(RobotMap.driveKs, RobotMap.driveKv, RobotMap.driveKa);
   // mDrivePIDController = new PIDController(RobotMap.kDrivePidConstants.kP, 0, 0);

   // mSteeringPIDController = new PIDController(
   //   RobotMap.kSteeringPidConstants.kP, 
   //   RobotMap.kSteeringPidConstants.kI, 
   //   RobotMap.kSteeringPidConstants.kD
   // );
   mSteeringPIDController.enableContinuousInput(-Math.PI, Math.PI);
  //  mSteeringPIDController.setTolerance(0.1);
 }

 /**
  * Sets the desired state of the module.
  * @param desiredState The state of the module that we'd like to be at in this period
  */
 public void setDesiredState(SwerveModuleState desiredState){
   // Optimize the state to avoid turning wheels further than 90 degrees
   var encoderRotation = mEncoder.getRotation2d().rotateBy(Rotation2d.fromDegrees(-90));
   desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);

   // Drive motor logic
   // var currentVelocityInRotationsPer20ms = RobotMap.kDriveGearRatio * ((mDriveMotor.getSelectedSensorVelocity(0) / 5) / mEncoder.kPositionsPerRotation);
   // var currentVelocityInMetersPer20ms = RobotMap.kDriveWheelCircumference * currentVelocityInRotationsPer20ms;
   var desiredVelocity = (desiredState.speedMetersPerSecond / 50) * 2048;
   var driveFeedForward = getController().calculate(getMeasurement(), desiredVelocity);
   // var driveFeedforward = mDriveFeedforward.calculate(currentVelocityInMetersPer20ms, desiredVelocity, 0.2);
   // var driveFeedback = mDrivePIDController.calculate(currentVelocityInMetersPer20ms, desiredVelocity);
      var driveFeedback = mDrivePIDController.calculate(getMeasurement(), desiredVelocity);
   var desiredMotorVelocity = driveFeedback + driveFeedForward;
   mDriveMotor.set(ControlMode.PercentOutput, MathUtil.applyDeadband(desiredMotorVelocity, 0.15));
   
   // Steering motor logic
   var currentPositionRadians = encoderRotation.getRadians();
   var desiredPositionRadians = desiredState.angle.getRadians();
   var steeringOutput = -mSteeringPIDController.calculate(currentPositionRadians, desiredPositionRadians);
   steeringOutput = MathUtil.applyDeadband(steeringOutput, 0.05);
   mSteeringMotor.set(ControlMode.PercentOutput, MathUtil.clamp(steeringOutput, -1, 1));
 }

 /**
  * Used for testing
  */
 public void driveSimple(double fwd, double rot){
    mDriveMotor.set(fwd);
    mSteeringMotor.set(rot);
 }

@Override
protected void useOutput(double output, double setpoint) {
   // TODO Auto-generated method stub
   
}

@Override
protected double getMeasurement() {
   // TODO Auto-generated method stub
   var currentVelocityInRotationsPer20ms = RobotMap.kDriveGearRatio * ((mDriveMotor.getSelectedSensorVelocity(0) / 5) / mEncoder.kPositionsPerRotation);
   var currentVelocityInMetersPer20ms = RobotMap.kDriveWheelCircumference * currentVelocityInRotationsPer20ms;
   return currentVelocityInMetersPer20ms;
}
}
