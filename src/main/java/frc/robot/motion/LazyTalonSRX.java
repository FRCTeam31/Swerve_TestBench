package frc.robot.motion;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LazyTalonSRX extends TalonSRX {
    private double mLastOutput = Double.NaN;
    private ControlMode mLastControlMode = null;

    public LazyTalonSRX(int deviceNumber) {
        super(deviceNumber);
    }
    
    public double getLastOutput() {
        return mLastOutput;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastOutput || mode != mLastControlMode) {
            mLastOutput = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }
}
