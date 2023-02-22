package prime.movers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class LazyWPITalonSRX extends WPI_TalonSRX {
    protected double mLastValue = Double.NaN;
    protected ControlMode mLastControlMode = null;

    public LazyWPITalonSRX(int deviceNumber) {
        super(deviceNumber);
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
