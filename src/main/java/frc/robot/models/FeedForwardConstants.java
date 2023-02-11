package frc.robot.models;

public class FeedForwardConstants {
    public double kS = 0;
    public double kV = 0;
    public double kA = 0;

    public FeedForwardConstants(double s) {
        kS = s;
    }

    public FeedForwardConstants(double s, double v) {
        kS = s;
        kV = v;
    }

    public FeedForwardConstants(double s, double v, double a) {
        kS = s;
        kV = v;
        kA = a;
    }
}
