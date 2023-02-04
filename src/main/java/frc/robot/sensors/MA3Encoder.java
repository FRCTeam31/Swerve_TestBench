package frc.robot.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

public class MA3Encoder extends AnalogInput {
    public final int kPositionsPerRotation = 4096;
    public int BasePositionOffset = 0;
    private boolean mInverted = false;

    public MA3Encoder(int analogChannel, int basePositionOffset, boolean inverted) {
        super(analogChannel);
        BasePositionOffset = basePositionOffset;
        mInverted = inverted;
    }

    @Override
    public int getValue() {
        var valWithOffset = super.getValue() - BasePositionOffset;

        return mInverted
            ? kPositionsPerRotation - valWithOffset
            : valWithOffset;
    }

    public int getRawValue() {
        return super.getValue();
    }
    
    public double getAngle() {
        return ((double)getValue() / (double)kPositionsPerRotation) * 360d;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }
}
