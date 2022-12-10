package frc.robot.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

public class MA3Encoder extends AnalogInput {
    public static final double ENC_POSITIONS_PER_REVOLUTION = 4096d;
    public int BasePositionOffset = 0;

    public MA3Encoder(int analogChannel, int basePositionOffset) {
        super(analogChannel);
        BasePositionOffset = basePositionOffset;
    }

    @Override
    public int getValue() {
        return super.getValue() - BasePositionOffset;
    }
    
    public double getAngle() {
        return (getValue() / ENC_POSITIONS_PER_REVOLUTION) * 360;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }
}
