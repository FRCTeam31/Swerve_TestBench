package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretRotation extends SubsystemBase {
    private WPI_TalonSRX mRotator;

    public TurretRotation() {
        mRotator = new WPI_TalonSRX(22);
        mRotator.clearStickyFaults();
        mRotator.setSafetyEnabled(false);

    }

    public void rotate(double speed) {
        mRotator.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        mRotator.stopMotor();
    }

}
