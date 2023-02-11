package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.config.DriveMap;
import frc.robot.motion.LazyTalonFX;
import frc.robot.sensors.MA3Encoder;
import frc.robot.utilities.CTREConverter;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class SwerveModule extends PIDSubsystem {
    private LazyTalonFX mSteeringMotor;
    private LazyTalonFX mDriveMotor;
    public MA3Encoder mEncoder;
    private PIDController mSteeringPIDController;
    private PIDController mDrivePIDController;
    private SimpleMotorFeedforward mDriveFeedforward;

    public SwerveModule(
            int driveMotorId,
            int steeringMotorId,
            int encoderAioChannel,
            int encoderBasePositionOffset) {
        super(new PIDController(DriveMap.kSteeringPidConstants.kP, DriveMap.kSteeringPidConstants.kI,
                DriveMap.kSteeringPidConstants.kD));

        // Set up the steering motor
        mSteeringMotor = new LazyTalonFX(steeringMotorId);
        mSteeringMotor.configFactoryDefault();
        mSteeringMotor.clearStickyFaults();
        mSteeringMotor.setNeutralMode(NeutralMode.Brake);
        mSteeringMotor.setInverted(TalonFXInvertType.Clockwise);
        mSteeringMotor.configOpenloopRamp(0.2);

        // Set up the drive motor
        mDriveMotor = new LazyTalonFX(driveMotorId);
        mDriveMotor.configFactoryDefault();
        mDriveMotor.clearStickyFaults();
        mDriveMotor.setNeutralMode(NeutralMode.Brake);
        mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // The integrated sensor in the
                                                                                   // Falcon is the falcon's encoder
        mDriveMotor.configOpenloopRamp(0.2);
        mDriveMotor.setInverted(TalonFXInvertType.Clockwise);

        // Set up our encoder
        mEncoder = new MA3Encoder(encoderAioChannel, encoderBasePositionOffset, true);

        // Create a PID controller to calculate steering motor output
        mDriveFeedforward = new SimpleMotorFeedforward(DriveMap.kDriveFeedForwardConstants.kS,
                DriveMap.kDriveFeedForwardConstants.kV, DriveMap.kDriveFeedForwardConstants.kA);
        mDrivePIDController = new PIDController(DriveMap.kDrivePidConstants.kP, 0, 0);

        mSteeringPIDController = new PIDController(
                DriveMap.kSteeringPidConstants.kP,
                DriveMap.kSteeringPidConstants.kI,
                DriveMap.kSteeringPidConstants.kD);
        getController().enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Sets the desired state of the module.
     * 
     * @param desiredState The state of the module that we'd like to be at in this
     *                     period
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the state to avoid turning wheels further than 90 degrees
        var encoderRotation = mEncoder.getRotation2d().rotateBy(Rotation2d.fromDegrees(-90));
        desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);

        // Drive motor logic
        var currentVelocityInRotationsPer20ms = DriveMap.kDriveGearRatio
                * ((mDriveMotor.getSelectedSensorVelocity(0) / 5) / mEncoder.kPositionsPerRotation);
        var currentVelocityInMetersPer20ms = DriveMap.kDriveWheelCircumference * currentVelocityInRotationsPer20ms;
        var desiredVelocity = (desiredState.speedMetersPerSecond / 50) * 2048;
        var driveFeedforward = mDriveFeedforward.calculate(currentVelocityInMetersPer20ms, desiredVelocity, 0.2);
        var driveFeedback = mDrivePIDController.calculate(currentVelocityInMetersPer20ms, desiredVelocity);
        var desiredMotorVelocity = driveFeedback + driveFeedforward;
        mDriveMotor.set(ControlMode.PercentOutput, MathUtil.applyDeadband(desiredMotorVelocity, 0.15));

        // Steering motor logic
        var currentPositionRadians = encoderRotation.getRadians();
        var desiredPositionRadians = desiredState.angle.getRadians();
        var steeringOutput = -mSteeringPIDController.calculate(currentPositionRadians, desiredPositionRadians);
        steeringOutput = MathUtil.applyDeadband(steeringOutput, 0.1);
        mSteeringMotor.set(ControlMode.PercentOutput, MathUtil.clamp(steeringOutput, -1, 1));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(CTREConverter.falconToMeters(
                mDriveMotor.getSelectedSensorPosition(),
                DriveMap.kDriveWheelCircumference,
                DriveMap.kDriveGearRatio),
                mEncoder.getRotation2d());
    }

    @Override
    protected void useOutput(double output, double setpoint) {

    }

    @Override
    protected double getMeasurement() {
        var encoderRotation = mEncoder.getRotation2d().rotateBy(Rotation2d.fromDegrees(-90));
        var currentPositionRadians = encoderRotation.getRadians();

        return currentPositionRadians;
    }
}
