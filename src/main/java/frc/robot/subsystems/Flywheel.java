package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final int rightNeoId = 18;
    private final int leftNeoId = 19;
    private CANSparkMax neoRight;
    private CANSparkMax neoLeft;
    private SparkMaxPIDController mPidController;
    public final static int kMaxRpm = 3000;
    private int _lastRpmSet = 0;

    public Flywheel() {
        neoRight = new CANSparkMax(rightNeoId, MotorType.kBrushless);
        neoRight.setClosedLoopRampRate(3);

        neoLeft = new CANSparkMax(leftNeoId, MotorType.kBrushless);
        neoLeft.follow(neoRight, true);

        neoRight.restoreFactoryDefaults();
        neoLeft.restoreFactoryDefaults();

        mPidController = neoRight.getPIDController();
        mPidController.setOutputRange(0, kMaxRpm);
        mPidController.setP(0.001);
    }

    public void setEnabled(boolean enabled) {
        if (enabled) {
            mPidController.setReference(_lastRpmSet, ControlType.kVelocity);
        } else {
            mPidController.setReference(0, ControlType.kDutyCycle);
            neoRight.stopMotor();
        }
    }

    public void setSpeed(int rpm) {
        if (rpm != _lastRpmSet) {
            mPidController.setReference(rpm, ControlType.kVelocity);
            _lastRpmSet = rpm;
        }
    }
}
