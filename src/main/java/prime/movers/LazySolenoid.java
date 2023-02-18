package prime.movers;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class LazySolenoid extends Solenoid {
    protected boolean mLastValue = false;

    public LazySolenoid(PneumaticsModuleType moduleType, int channel) {
        super(moduleType, channel);
        set(false);
    }

    public LazySolenoid(int module, PneumaticsModuleType moduleType, int channel) {
        super(module, moduleType, channel);
        set(false);
    }
    
    public Boolean getLastValue() {
        return mLastValue;
    }

    @Override
    public void set(boolean on) {
        if (on == mLastValue) return;
     
        mLastValue = on;
        set(on);
    }

}
