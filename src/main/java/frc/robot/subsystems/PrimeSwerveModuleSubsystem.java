package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.config.RobotMap;
import frc.robot.prime.utilities.CTREConverter;
import frc.robot.sensors.MA3Encoder;
import prime.movers.LazyWPITalonFX;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PrimeSwerveModuleSubsystem extends PIDSubsystem {
 private LazyWPITalonFX mSteeringMotor;
 private LazyWPITalonFX mDriveMotor;
 public MA3Encoder mEncoder;
   
 public PrimeSwerveModuleSubsystem (
    int driveMotorId,
    int steeringMotorId,
    int encoderAioChannel,
    int encoderBasePositionOffset
 ){
   super(new PIDController(RobotMap.kSteeringPidConstants.kP, RobotMap.kSteeringPidConstants.kI, RobotMap.kSteeringPidConstants.kD));

   // Set up the steering motor
   mSteeringMotor = new LazyWPITalonFX(steeringMotorId);
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
   TalonFXConfiguration  driveMotorConfig = new TalonFXConfiguration();
   driveMotorConfig.slot0.kP = RobotMap.kDrivePidConstants.kP;
   mDriveMotor.configAllSettings(driveMotorConfig);





   getController().enableContinuousInput(-Math.PI, Math.PI);
   getController().setTolerance(0.1);
  enable();
 }

 public SwerveModulePosition getPosition() {
  return new SwerveModulePosition(
    CTREConverter.falconToMeters(
      mDriveMotor.getSelectedSensorPosition(), 
      RobotMap.kDriveWheelCircumference, 
      RobotMap.kDriveGearRatio
    ), 
    mEncoder.getRotation2d());
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
      
   var currentVelocityInRotationsPer20ms = RobotMap.kDriveGearRatio * ((mDriveMotor.getSelectedSensorVelocity(0) / 5) / mEncoder.kPositionsPerRotation);
   var currentVelocityInMetersPer20ms = RobotMap.kDriveWheelCircumference * currentVelocityInRotationsPer20ms;
   var desiredVelocity20ms = (desiredState.speedMetersPerSecond / 50) * RobotMap.falconTotalSensorUnits; 
   var desiredRotationsPer20ms = desiredVelocity20ms / RobotMap.kDriveWheelCircumference;
   var desiredVelocity = (desiredRotationsPer20ms * RobotMap.falconTotalSensorUnits * 5);

   mDriveMotor.set(ControlMode.Velocity, desiredVelocity);  
   // Steering motor logic
   var desiredPositionRadians = desiredState.angle.getRadians();

   getController().setSetpoint(desiredPositionRadians);
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
   mSteeringMotor.set(ControlMode.PercentOutput, MathUtil.clamp(-output, -1, 1));
}

@Override
protected double getMeasurement() {
   // TODO Auto-generated method stub
   var encoderRotation = mEncoder.getRotation2d().rotateBy(Rotation2d.fromDegrees(-90));
   var currentPositionRadians = encoderRotation.getRadians();
   return currentPositionRadians;


  
}
}
