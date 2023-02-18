package prime.movers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class LazyWPITalonFX extends WPI_TalonFX {
    protected double mLastValue = Double.NaN;
    protected ControlMode mLastControlMode = null;

    public LazyWPITalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    public LazyWPITalonFX(int deviceNumber, String name) {
        super(deviceNumber, name);
    }

    public double getLastValue() {
        return mLastValue;
    }

    public ControlMode getLastControlMode() {
        return mLastControlMode;
    }
    
    @Override
    public void set(ControlMode mode, double value) {
        if (value == mLastValue && mode == mLastControlMode) return;

        mLastValue = value;
        mLastControlMode = mode;
        super.set(mode, value);
    }
}
