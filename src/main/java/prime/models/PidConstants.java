package prime.models;

public class PidConstants {
  public double kP = 0;
  public double kI = 0;
  public double kD = 0;
  public double kF = 0;

  public PidConstants(double p) {
    kP = p;
    kI = 0;
    kD = 0;
    kF = 0;
  }

  public PidConstants(double p, double i) {
    kP = p;
    kI = i;
    kD = 0;
    kF = 0;
  }

  public PidConstants(double p, double i, double d) {
      kP = p;
      kI = i;
      kD = d;
      kF = 0;
  }

  public PidConstants(double p, double i, double d, double f) {
      kP = p;
      kI = i;
      kD = d;
      kF = f;
  }
}
