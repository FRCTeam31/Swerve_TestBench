package prime.utilities;

public class UnitConverter {
    private static final double IN_M_FACTOR = 39.37;

    public static double inchesToMeters(double inches) {
        return inches / IN_M_FACTOR;
    }

    public static double metersToInches(double meters) {
        return meters * IN_M_FACTOR;
    }
}
