package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazyCANSparkMax;

public class Flywheel extends SubsystemBase {
    public final int kMaxRpm = 3000;

    private final int kRightNeoId = 18;
    private final int kLeftNeoId = 19;
    private double _lastRpmSet = 0;
    private boolean _flywheelEnabled = false;

    private LazyCANSparkMax neoRight;
    private LazyCANSparkMax neoLeft;
    private SparkMaxPIDController mPidController;

    public Flywheel() {
        neoRight = new LazyCANSparkMax(kRightNeoId, MotorType.kBrushless);
        neoRight.setClosedLoopRampRate(3);

        neoLeft = new LazyCANSparkMax(kLeftNeoId, MotorType.kBrushless);
        neoLeft.follow(neoRight, true);

        neoRight.restoreFactoryDefaults();
        neoLeft.restoreFactoryDefaults();

        mPidController = neoRight.getPIDController();
        mPidController.setOutputRange(0, kMaxRpm);
        mPidController.setP(0.001);
    }

    @Override
    public void periodic() {
        if (_flywheelEnabled) {
            mPidController.setReference(_lastRpmSet, ControlType.kVelocity);
        } else {
            if (neoRight.get() != 0) {
                mPidController.setReference(0, ControlType.kVelocity);
                neoRight.stopMotor();
            }
        }
    }

    public void setEnabled(boolean enabled) {
        _flywheelEnabled = enabled;
    }

    public boolean getEnabled() {
        return _flywheelEnabled;
    }

    public void setSpeed(double rpm) {
        if (rpm != _lastRpmSet) {
            mPidController.setReference(rpm, ControlType.kVelocity);
            _lastRpmSet = rpm;
        }
    }
}
