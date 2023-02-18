package prime.movers;

import com.revrobotics.CANSparkMax;

public class LazyCANSparkMax extends CANSparkMax {
    protected double mLastSpeed = Double.NaN;

    public LazyCANSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    public double getLastSpeed() {
        return mLastSpeed;
    }

    @Override
    public void set(double speed) {
        if (speed == mLastSpeed)
            return;

        mLastSpeed = speed;
        set(speed);
    }
}
