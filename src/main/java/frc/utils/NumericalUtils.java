package frc.utils;

public class NumericalUtils {

    public static boolean isWithinTolerance(int value, int desired, int tolerance, ToleranceType toleranceType) {
        if (Math.abs(desired - value) < tolerance) {
            return true;
        }
        switch (toleranceType) {
            case GREATER_ALLOWED:
                return desired < value;
            case LESS_ALLOWED:
                return desired > value;
            case ONLY_WITHIN_TOLERANCE:
                break;
        }
        return false;
    }

    public static boolean isWithinTolerance(double value, double desired, double tolerance, ToleranceType toleranceType) {
        if (Math.abs(desired - value) < tolerance) {
            return true;
        }
        switch (toleranceType) {
            case GREATER_ALLOWED:
                return desired < value;
            case LESS_ALLOWED:
                return desired > value;
            case ONLY_WITHIN_TOLERANCE:
                break;
        }
        return false;
    }

    public enum ToleranceType {
        GREATER_ALLOWED,
        LESS_ALLOWED,
        ONLY_WITHIN_TOLERANCE
    }

}
