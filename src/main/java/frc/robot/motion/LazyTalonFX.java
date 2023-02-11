package frc.robot.motion;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class LazyTalonFX extends WPI_TalonFX {
    private double mLastOutput = Double.NaN;
    private ControlMode mLastControlMode = null;

    public LazyTalonFX(int deviceNumber) {
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
