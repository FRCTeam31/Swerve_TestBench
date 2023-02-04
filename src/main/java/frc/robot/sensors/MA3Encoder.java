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
        var offsetValue = super.getValue() + BasePositionOffset;

        return mInverted
            ? kPositionsPerRotation - offsetValue
            : offsetValue;
    }
    
    public double getAngle() {
        return (getValue() / kPositionsPerRotation) * 360;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }
}
