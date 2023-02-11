package frc.robot.models;

public class PidConstants {
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;

    public PidConstants(double p) {
        kP = p;
    }

    public PidConstants(double p, double i) {
        kP = p;
        kI = i;
    }

    public PidConstants(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }
}
